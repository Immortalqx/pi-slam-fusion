#include "Mapper.h"
#include "MapFrame.h"
#include "Matcher.h"
#include "MapPoint.h"

#include <algorithm>
#include <unordered_map>

#include <GSLAM/core/Timer.h>
#include <GSLAM/core/Random.h>
#include <GSLAM/core/Event.h>

#ifdef HAS_EIGEN3
#include <Eigen/Dense>
#endif

namespace zhangmi{

using GSLAM::PointID;
using GSLAM::FrameID;
using GSLAM::Point3d;
using GSLAM::Point2d;
using GSLAM::SE3;
using GSLAM::PointPtr;
using GSLAM::FramePtr;
using std::vector;
using std::pair;
using std::make_pair;

typedef GSLAM::FrameConnection FrameConnection;
typedef SPtr<FrameConnection>  FrameConnectionPtr;

class MapFrameConnection : public GSLAM::FrameConnection
{
public:
    virtual std::string type()const{return "MapFrameConnection";}
    virtual int  matchesNum(){return _sharedPoints.size();}

    virtual bool getMatches(std::vector<std::pair<int,int> >& matches){return false;}
    virtual bool getChild2Parent(GSLAM::SIM3& sim3){return false;}
    virtual bool getChild2Parent(GSLAM::SE3& se3){return false;}
    virtual bool getInformation(double* info){return false;}

    virtual bool setMatches(std::vector<std::pair<int,int> >& matches){return false;}
    virtual bool setChild2Parent(GSLAM::SIM3& sim3){return false;}
    virtual bool setChild2Parent(GSLAM::SE3& se3){return false;}
    virtual bool setInformation(double* info){return false;}

    vector<PointID>          _sharedPoints;
    vector<pair<int,int> >   _matches;
};

class Mapper : public ::Mapper
{
public:
    Mapper(const GSLAM::MapPtr &map);
    virtual ~Mapper(){stop();}

    virtual bool insertKeyFrame(const SPtr<MapFrame>& frame){
        _curFrame=frame;
        handleCurrentFrame();
//        _insertionEnabled.wait();
        return true;
    }

    virtual void stop(){_shouldStop=true;}

    virtual void mappingThread();

private:
    void  handleCurrentFrame();
    void  makeKeyFrame();
    void  mapPointCulling();
    void  createNewMapPoints();
    void  dataAssociation();
    int   fuse(const GSLAM::FramePtr& frame,const std::vector<GSLAM::PointPtr>& mappoints);
    void  localOptimization();


    bool eraseMapPoint(const PointPtr& mpt);
    bool fuseMapPoint(const PointPtr& from,const PointPtr& to);
    bool triangulate(double* t1,double* t2,const pi::Point3d& xn1,
                     const pi::Point3d& xn2,pi::Point3d& pt);

    SPtr<GSLAM::Optimizer>     _optimizer;
    SPtr<MapFrame>             _curFrame,_lastFrame;
    SPtr<Matcher>              _matcher;
    GSLAM::Event               _insertionEnabled;

    std::list<PointPtr>        _recentPoints;
    bool                       _shouldStop;
    std::stringstream          _logger;
};

Mapper::Mapper(const GSLAM::MapPtr &map)
    : ::Mapper(map){
    _matcher=Matcher::create(svar.GetString("Matcher","flann"));
    _optimizer=GSLAM::Optimizer::create(svar.GetString("Optimizer",""));

    if(!_optimizer)
    {
        LOG(FATAL)<<"No valid Optimizer!";
    }
    if(!_matcher){
        LOG(FATAL)<<"No valid Matcher!";
    }
}

void Mapper::mappingThread(){
    _shouldStop=false;
return ;
    while(!_shouldStop)
    {
        if(_curFrame);
        GSLAM::Rate::sleep(0.001);
    }
}

void  Mapper::handleCurrentFrame()
{
    _logger.str("");

    makeKeyFrame();
    mapPointCulling();
    createNewMapPoints();
    dataAssociation();

    localOptimization();

    if(_lastFrame)
    {
        _lastFrame->setImage(GSLAM::GImage());
        if(_handle)
        {
            _handle->handle(new GSLAM::ScenceRadiusEvent(_lastFrame->getMedianDepth()*10));
            _handle->handle(new GSLAM::ScenceCenterEvent(_lastFrame->getPose().get_translation()));
        }
    }
    _lastFrame=_curFrame;

    LOG(INFO)<<_logger.str();
    _insertionEnabled.set();
    _curFrame.reset();
}

void  Mapper::makeKeyFrame()
{
    _map->insertMapFrame(_curFrame);
    std::map<FrameID,FrameConnectionPtr> parents;

    std::map<PointID,size_t> observes;// featureID
    _curFrame->getObservations(observes);
    for(pair<PointID,size_t> obs : observes)
    {
        const PointPtr& pt=_curFrame->keyPoint(obs.second)._mapPoint;
        std::map<FrameID,size_t> ptObserves;
        pt->getObservations(ptObserves);
        for(pair<FrameID,size_t> ptObs:ptObserves)
        {
            auto it=parents.find(ptObs.first);
            if(it==parents.end())
            {
                MapFrameConnection* con=new MapFrameConnection();
                con->_sharedPoints.push_back(pt->id());
                con->_matches.push_back(pair<int,int>(ptObs.second,obs.second));
                parents.insert(make_pair(ptObs.first,FrameConnectionPtr(con)));
            }
            else
            {
                MapFrameConnection* con=dynamic_cast<MapFrameConnection*>(it->second.get());
                con->_sharedPoints.push_back(pt->id());
                con->_matches.push_back(pair<int,int>(ptObs.second,obs.second));
            }
        }
        pt->addObservation(_curFrame->id(),obs.second);
    }

    for(pair<FrameID,FrameConnectionPtr> it:parents)
    {
        if(it.second->matchesNum()>30)
        {
            FramePtr parent=_map->getFrame(it.first);
            if(!parent) continue;
            parent->addChildren(_curFrame->id(),it.second);
            _curFrame->addParent(it.first,it.second);
        }
    }
}

void  Mapper::mapPointCulling()
{
    SCOPE_TIMER;
    int cullCount=0;
    std::list<GSLAM::PointPtr>::iterator lit=_recentPoints.begin();
    while(lit!=_recentPoints.end())
    {
        GSLAM::MapPoint* pMP = (GSLAM::MapPoint*)(*lit).get();
        if(pMP->refKeyframeID()+3<=_curFrame->id())
        {
            if(pMP->observationNum()<=2)
            {
                cullCount++;
                eraseMapPoint(*lit);
            }
            lit = _recentPoints.erase(lit);
        }
        else
            lit++;
    }
    _logger<<_curFrame->id()<<",ER:"<<cullCount;
}

void  Mapper::createNewMapPoints()
{
    SCOPE_TIMER;
    using namespace std;
    /// 1. Collect nearby keyframes
    size_t createdCount=_recentPoints.size();
    std::map<GSLAM::FrameID,SPtr<GSLAM::FrameConnection> > parents;
    _curFrame->getParents(parents);

    GSLAM::SE3    nowTcw=_curFrame->getPose();
    GSLAM::SE3    nowTwc=nowTcw.inverse();
    pi::Point3d   Ow1=nowTcw.get_translation();
    GSLAM::Camera cam=_curFrame->getCamera(1);
    float         mDepth=_curFrame->getMedianDepth();
    double t1[12],t2[12];
    GSLAM::SE3 Iden;Iden.getMatrix(t1);

    list<pi::Point2i> waitTriList;
    for(int iy=0;iy<FRAME_GRID_ROWS;iy++)
        for(int ix=0;ix<FRAME_GRID_COLS;ix++)
        {
            const std::vector<std::size_t>& pts=_curFrame->getGrid(ix,iy);
            if(pts.empty()) continue;
            bool needTriangulate=true;
            for(const std::size_t& i:pts)
            {
                if(_curFrame->keyPoint(i)._mapPoint.get())
                {
                    needTriangulate=false;
                    break;
                }
            }
            if(needTriangulate)
                waitTriList.push_back(pi::Point2i(ix,iy));
        }

    bool triangulateHomography=svar.GetInt("TriangulateHomography",1);
    int  waitTriListCount=waitTriList.size();
    for(auto con:parents)
    {
        if(con.second->matchesNum()<30) continue;

        GSLAM::FramePtr refKF=_map->getFrame(con.first);
        if(!refKF.get()) continue;

        GSLAM::SE3  kf_Tcw=nowTwc*refKF->getPose();
        GSLAM::SE3  kf_Twc=kf_Tcw.inverse();
        pi::Point3d Ow2=kf_Tcw.get_translation();
        pi::Point3d vBaseline=Ow2;
        kf_Twc.getMatrix(t2);

        if(vBaseline.norm()/mDepth<0.01) continue;

        for(list<pi::Point2i>::iterator it=waitTriList.begin();it!=waitTriList.end();)
        {
            pi::Point2i gridIdx=*it;
            const std::vector<std::size_t>& kpts=_curFrame->getGrid(gridIdx.x,gridIdx.y);
            bool triangulated=false;
            for(const size_t& featId1:kpts)
            {
                if(_curFrame->keyPoint(featId1)._mapPoint.get()) continue;
                /// A. Find match
                const GSLAM::Point2f& pt2d1=_curFrame->keyPoint(featId1)._pt.pt;
                int featId2;
                GSLAM::GImage desc=_curFrame->getDescriptor(featId1);
                if(triangulateHomography)
                {
                    pi::Point2d pt2d2_est=cam.Project(kf_Twc*(cam.UnProject(pt2d1.x,pt2d1.y)*mDepth));
                    if(!_matcher->findMatchWindow(desc,refKF,
                                        pt2d2_est.x,pt2d2_est.y,cam.width()/64,featId2)) continue;
                }
                else
                {

                }
                const GSLAM::Point2f& pt2d2=((MapFrame*)refKF.get())->keyPoint(featId2)._pt.pt;

                /// B. Check parallax between rays
                const pi::Point3d &xn1 = cam.UnProject(pt2d1.x,pt2d1.y);
                const pi::Point3d &ray1= xn1;
                const pi::Point3d &xn2 = cam.UnProject(pt2d2.x,pt2d2.y);
                const pi::Point3d &ray2= kf_Tcw.get_rotation()*xn2;
                const double cosParallaxRays = (ray1*ray2)/(ray1.norm()*ray2.norm());

                if(cosParallaxRays<0 ||cosParallaxRays>0.9998)//FIXME: fixed in 20150714
                    continue;

                //  Linear Triangulation Method
                pi::Point3d p3d;
                if(!triangulate(t1,t2,xn1,xn2,p3d)) continue;

                /// C. Triangulation in front of cameras?
                pi::Point3d pc1=(p3d);
                pi::Point3d pc2=(kf_Twc*p3d);
                if(pc1.z<=0) continue;
                if(pc2.z<=0) continue;

                /// D. Check reprojection errors are not too large
                float sigmaSquare = 1;//_curFrame->keyPoint(featId1)._ptUn.size;
                sigmaSquare*=5.991*sigmaSquare;

                pi::Point2d e=cam.Project(pc1)-pi::Point2d(pt2d1.x,pt2d1.y);
                if((e.x*e.x+e.y*e.y)>sigmaSquare)
                    continue;

                e=cam.Project(pc2)-pi::Point2d(pt2d2.x,pt2d2.y);
                if((e.x*e.x+e.y*e.y)>sigmaSquare)
                    continue;

                /// E. Check scale consistency

                /// G. Finally found a new mappoint
                p3d=nowTcw*p3d;
                PointPtr pMP(new MapPoint(_map->getPid(),p3d,(Ow1-p3d).normalize(),
                                                _curFrame->keyPoint(featId1)._color,_curFrame->id()));
                pMP->setDescriptor(desc.clone());
                _curFrame->addObservation(pMP,featId1,true);
                refKF->addObservation(pMP,featId2,true);
                _map->insertMapPoint(pMP);
                _recentPoints.push_back(pMP);
                triangulated=true;
                it=waitTriList.erase(it);

                break;// triangulate one mappoint for one grid
            }
            if(!triangulated)
                it++;
        }
    }

    createdCount=_recentPoints.size()-createdCount;
    _logger<<",NEW:"<<createdCount<<"/"<<waitTriListCount;

}

void  Mapper::dataAssociation()
{
    SCOPE_TIMER;
    vector<GSLAM::PointPtr> curPoints;

    std::map<GSLAM::PointID,size_t> observes;
    _curFrame->getObservations(observes);
    curPoints.reserve(observes.size());
    for(auto obs:observes)
    {
        const GSLAM::PointPtr& pt=_curFrame->keyPoint(obs.second)._mapPoint;
        curPoints.push_back(pt);
    }

    std::map<GSLAM::FrameID,SPtr<GSLAM::FrameConnection> >    parents;
    if(!_curFrame->getParents(parents)) return;

    int fusedCount=0;
    for(auto& con:parents)
    {
        GSLAM::FramePtr fr=_map->getFrame(con.first);
        if(!fr.get()) continue;
        fusedCount+=fuse(fr,curPoints);
    }

    _logger<<",FUSE:"<<fusedCount;
}

int Mapper::fuse(const GSLAM::FramePtr& frame,const std::vector<GSLAM::PointPtr>& mappoints)
{
    GSLAM::SE3 w2c=frame->getPose();
    GSLAM::Point3Type Ow=w2c.get_translation();
    w2c=w2c.inverse();
    std::map<GSLAM::PointID,size_t> observes;
    frame->getObservations(observes);
    GSLAM::Camera cam=frame->getCamera(1);
    MapFrame* fr=(MapFrame*)frame.get();

    int nfused=0;
    for(const GSLAM::PointPtr& p:mappoints)
    {
        if(observes.count(p->id())) continue;
        const pi::Point3d p3d=p->getPose();
        const pi::Point3d pc3=w2c*p3d;
        if(pc3.z<0) continue;

        const pi::Point2d pc2=cam.Project(pc3);
        if(pc2.x<=0||pc2.y<=0||pc2.x>=cam.width()||pc2.y>=cam.height()) continue;

        // Viewing angle should be less than 60 deg
        pi::Point3d normal1 = p->getNormal();
        pi::Point3d nVec    = Ow-p3d;
        if(normal1*nVec < 0.5*nVec.norm())
            continue;

        int idx;
        if(!_matcher->findMatchWindow(((MapPoint*)p.get())->getDescriptor(),frame,
                            pc2.x,pc2.y,svar.GetDouble("FuseRadius",cam.width()/64),
                            idx,false)) continue;

        if(fr->getKeyPointObserve(idx))//existed mappoint
        {
            if(!fuseMapPoint(fr->keyPoint(idx)._mapPoint,p))
            {
//                DLOG(ERROR)<<"Failed to fuse mappoint.";
            }
        }
        else// new observation!
        {
            fr->addObservation(p,idx,true);
        }
        nfused++;
    }
    return nfused;
}

void  Mapper::localOptimization()
{
    SCOPE_TIMER;

    using namespace GSLAM;
    if(!_optimizer)
    {
        _optimizer=GSLAM::Optimizer::create(svar.GetString("BundlePlugin","libgslam_optimizer"));
        svar.GetDouble("Optimizer.MaxSolverTime",1e3)=0.5;
    }
    if(!_optimizer) return;

    // collect local frames
    typedef std::unordered_map<GSLAM::FrameID,uint> FrameIDMap;
    typedef std::unordered_map<GSLAM::PointID,uint> PointIDMap;

    GSLAM::BundleGraph graph;

    vector<GSLAM::FramePtr> KeyFrameEles;
    FrameIDMap frameId_map;
    vector<GSLAM::PointPtr> PointEles;
    PointIDMap pointId_map;

    GSLAM::Camera cam=_curFrame->getCamera(1);

    graph.keyframes.reserve(_map->frameNum());
    KeyFrameEles.reserve(_map->frameNum());

    graph.mappoints.reserve(_map->pointNum());
    PointEles.reserve(_map->pointNum());

    GSLAM::SE3 l2w=_curFrame->getPose();
    GSLAM::SE3 w2l=l2w.inverse();
    //1. Collect Local Keyframes and insert current MapPoints
    //1.1. Insert current frame
    graph.keyframes.push_back({w2l*_curFrame->getPose(),UPDATE_KF_SE3});
    KeyFrameEles.push_back(_curFrame);
    frameId_map.insert(std::make_pair(_curFrame->id(),0));

    //1.2. Collect connected keyframes
    std::map<GSLAM::FrameID,SPtr<GSLAM::FrameConnection> >    parents;
    if(!((::MapFrame*)_curFrame.get())->getParents(parents)) return;
    for(auto& con:parents)
    {
        GSLAM::FramePtr kf=_map->getFrame(con.first);
        if(!kf.get()) continue;
        frameId_map.insert(make_pair(con.first,graph.keyframes.size()));
        graph.keyframes.push_back({w2l*kf->getPose(),UPDATE_KF_SE3});
        KeyFrameEles.push_back(kf);
    }
    int unFixKFNum=KeyFrameEles.size();

    timer.enter("Mapper::localBundle::1.Collect");

    //1.3. Collect local mappoints
    for(auto& fr:KeyFrameEles)
    {
        std::map<GSLAM::PointID,size_t> observations;
        if(!fr->getObservations(observations))
        {
            DLOG(ERROR)<<("Failed to get valid Observation!");
            continue;
        }

        for(auto& obs:observations)
        {
            PointIDMap::iterator p_it=pointId_map.find(obs.first);
            //Check if the mappoint had be added
            if(p_it==pointId_map.end())
            {
                const GSLAM::PointPtr& pt=((::MapFrame*)fr.get())->keyPoint(obs.second)._mapPoint;
                assert(pt.get()&&pt->id()==obs.first);


                {
                    pointId_map.insert(make_pair(obs.first,graph.mappoints.size()));
                    graph.mappoints.push_back({w2l*pt->getPose(),true});
                    PointEles.push_back(pt);
                }
            }
        }
    }

    //1.4. Collect observes and Fixed MapFrames
    float invSigma2=(1000./cam.width());
    invSigma2*=invSigma2;
    GSLAM::BundleEdge bobs;
    for(uint  pt_bundle_id=0;pt_bundle_id<PointEles.size();pt_bundle_id++)
    {
        GSLAM::PointPtr& pt=PointEles[pt_bundle_id];
        if(pt->observationNum()<2)
        {
            eraseMapPoint(pt);
            continue;
        }

        std::map<GSLAM::FrameID,size_t> ptObserves;
        if(!pt->getObservations(ptObserves))
        {
            DLOG(ERROR)<<("No valid observation!");
            continue;
        }

        for(auto& obs:ptObserves)
        {
            FrameIDMap::iterator id_it=frameId_map.find(obs.first);
            GSLAM::FramePtr fr;
            if(id_it==frameId_map.end())// New KeyFrame
            {
                fr=_map->getFrame(obs.first);
                assert(fr.get());
                if(!fr.get())
                {
                    DLOG(ERROR)<<("Can't find KeyFrame in the map");
                    continue;
                }
                bobs.frameId=KeyFrameEles.size();
                frameId_map.insert(make_pair(obs.first,graph.keyframes.size()));
                graph.keyframes.push_back({w2l*fr->getPose(),UPDATE_KF_NONE});
                KeyFrameEles.push_back(fr);
            }
            else
            {
                fr=KeyFrameEles[id_it->second];
                bobs.frameId=id_it->second;
            }
            GSLAM::KeyPoint kp=((::MapFrame*)fr.get())->keyPoint(obs.second)._pt;
            bobs.measurement=cam.UnProject(kp.pt.x,kp.pt.y);
            bobs.pointId=pt_bundle_id;
            graph.mappointObserves.push_back(bobs);
        }
    }

    if(svar.GetInt("GPS.Fitted"))
    {
        double* gpsInfo=(double*)svar.GetPointer("GPSInfo");
        GSLAM::SIM3 gpsSim3;
        for(size_t i=0;i<KeyFrameEles.size();i++)
        {
            ::MapFrame* fr=(::MapFrame*)KeyFrameEles[i].get();
            if(fr->getPrioryPose(gpsSim3))
                graph.gpsGraph.push_back({i,w2l*gpsSim3.get_se3(),gpsInfo});
        }
    }

    timer.leave("Mapper::localBundle::1.Collect");

    //    static TimerEventThread timer(svar.GetInt("MaxBundleOptimizeTime",200)
    //                                  ,&_stopBARequrested);
    //    timer.start();


        _logger<<",BA:"<<graph.keyframes.size()
              <<"-"<<graph.mappoints.size()
             <<"-"<<graph.mappointObserves.size();

    timer.enter("Mapper::localBundle::2.Optimize");
    if(!_optimizer->optimize(graph))
    {
        return;
    }
    timer.leave("Mapper::localBundle::2.Optimize");

    timer.enter("Mapper::localBundle::3.Update");
    //Update MapFrame poses
    for(int i=0,iend=KeyFrameEles.size();i<iend;i++)
    {
        GSLAM::FramePtr& frame_ele=KeyFrameEles[i];
        frame_ele->setPose(l2w*graph.keyframes[i].estimation.get_se3());
    }
    //Update MapPoint poses
    for(int i=0,iend=graph.mappoints.size();i<iend;i++)
    {
        GSLAM::PointPtr& point_ele=PointEles[i];
        point_ele->setPose(l2w*graph.mappoints[i].first);
    }

    //Remove bad edges
//    int removeBad=svar.GetInt("KeyFrameHandle.RemoveBadEdgeLocally",0);
//    vector<GSLAM::PointPtr> points2remove;
//    if(removeBad)
//    {

//        vector<double> errors;
//        double         sumError=0;
//        errors.reserve(graph.mappointObserves.size());
//        for(int i=0;i<graph.mappointObserves.size();i++)
//        {
//            BundleEdge& obs=graph.mappointObserves[i];
//            pi::Point2d    err=cam.Project(KeyFrames[obs.frame_id]*Points[obs.point_id])-obs.p_img;
//            double         error=err.x*err.x+err.y*err.y;
//            errors.push_back(error);
//            sumError+=error;
//        }

//        double averageError=sumError/errors.size();
//        double errorThreshold=averageError*removeBad;
//        int    eraseObsCount=0;
//        for(int i=0;i<errors.size();i++)
//        {
//            if(errors[i]>errorThreshold)
//            {
//                GSLAM::PointPtr& pt=PointEles[Observes[i].point_id];
//                GSLAM::FramePtr& fr=KeyFrameEles[Observes[i].frame_id];
//                fr->eraseObservation(pt,true);
//                eraseObsCount++;
//                if(pt->observationNum()<2)
//                {
//                    points2remove.push_back(pt);
//                }
//            }
//        }
//        for(auto& pt:points2remove)
//            eraseMapPoint(pt);
//        if(_verbose)
//            _logger<<",EO:"<<eraseObsCount
//                  <<",EP:"<<points2remove.size();


//    }

//    timer.leave("Mapper::localBundle::3.Update");

}

bool Mapper::eraseMapPoint(const GSLAM::PointPtr& mpt)
{
    std::map<GSLAM::FrameID,size_t> observes;
    if(!mpt->getObservations(observes)) return false;
    bool erasedAll=true;
    for(auto& obs:observes)
    {
        GSLAM::FramePtr fr=_map->getFrame(obs.first);
        if(!fr.get()) {erasedAll=false;continue;}
        if(!((MapFrame*)fr.get())->eraseObservation(mpt,true)) {erasedAll=false;continue;}
    }
    //    assert(erasedAll);
    if(erasedAll||true)
    {
        _map->eraseMapPoint(mpt->id());
        return true;
    }
    else
        return false;
}

bool Mapper::fuseMapPoint(const GSLAM::PointPtr& from,const GSLAM::PointPtr& to)
{
    std::map<GSLAM::FrameID,size_t> observes,toObserves;
    from->getObservations(observes);
    to->getObservations(toObserves);
    if(!_map->eraseMapPoint(from->id()))
    {
        return false;
    }
    // add to the new mappoint
    bool fusedAll=true;
    for(auto& obs:observes)
    {
        if(toObserves.count(obs.first)) continue;
        GSLAM::FramePtr fr=_map->getFrame(obs.first);
        if(((MapFrame*)fr.get())->keyPoint(obs.second)._mapPoint.get()) continue;
        if(!fr.get()) {fusedAll=false;continue;}
        if(!fr->addObservation(to,obs.second,true)) {fusedAll=false;continue;}
    }
    //    assert(fusedAll);
    if(fusedAll) return true;
    else
    {
        return false;
    }
}

bool Mapper::triangulate(double* t1,double* t2,const pi::Point3d& xn1,
                 const pi::Point3d& xn2,pi::Point3d& pt)
{
#ifdef HAS_EIGEN3
    {
        Eigen::Matrix4d A;
        A<<xn1[0]*t1[8]-t1[0], xn1[0]*t1[9]-t1[1], xn1[0]*t1[10]-t1[2], xn1[0]*t1[11]-t1[3],
           xn1[1]*t1[8]-t1[4], xn1[1]*t1[9]-t1[5], xn1[1]*t1[10]-t1[6], xn1[1]*t1[11]-t1[7],
           xn2[0]*t2[8]-t2[0], xn2[0]*t2[9]-t2[1], xn2[0]*t2[10]-t2[2], xn2[0]*t2[11]-t2[3],
           xn2[1]*t2[8]-t2[4], xn2[1]*t2[9]-t2[5], xn2[1]*t2[10]-t2[6], xn2[1]*t2[11]-t2[7];

        Eigen::JacobiSVD<Eigen::Matrix4d> svd(A,Eigen::ComputeFullU | Eigen::ComputeFullV);
        auto v=svd.matrixV();
        Eigen::Vector4d x3D;
        x3D<<v(0,3),v(1,3),v(2,3),v(3,3);

        if(v(3,3)==0) return false;

        pt=pi::Point3d(x3D[0]/x3D[3],x3D[1]/x3D[3],x3D[2]/x3D[3]);
        return true;
    }
#endif

#ifdef HAS_OPENCV
    {
        cv::Mat A=(cv::Mat_<double>(4,4) <<
                   xn1[0]*t1[8]-t1[0], xn1[0]*t1[9]-t1[1], xn1[0]*t1[10]-t1[2], xn1[0]*t1[11]-t1[3],
                xn1[1]*t1[8]-t1[4], xn1[1]*t1[9]-t1[5], xn1[1]*t1[10]-t1[6], xn1[1]*t1[11]-t1[7],
                xn2[0]*t2[8]-t2[0], xn2[0]*t2[9]-t2[1], xn2[0]*t2[10]-t2[2], xn2[0]*t2[11]-t2[3],
                xn2[1]*t2[8]-t2[4], xn2[1]*t2[9]-t2[5], xn2[1]*t2[10]-t2[6], xn2[1]*t2[11]-t2[7]);

//         std::cout<<A<<std::endl;

        cv::Mat w,u,vt;
        cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);

        cv::Mat x3D = vt.row(3).t();

        if(x3D.at<double>(3)==0)
            return false;

        // Euclidean coordinates
        x3D = x3D.rowRange(0,3)/x3D.at<double>(3);
        pt=pi::Point3d(x3D.at<double>(0),x3D.at<double>(1),x3D.at<double>(2));
        return true;
    }
#endif

    return false;
}


}


typedef zhangmi::Mapper MapperDemo;
REGISTER_MAPPER(MapperDemo,zhangmi);
