#include "Mapper.h"
#include "MapFrame.h"
#include "Matcher.h"
#include "MapPoint.h"
#include "LoopCloser.h"
#include "optimizerG2O/Optimizer.h"
#include "../Estimator.h"
#include <algorithm>
#include <unordered_map>
#include <unordered_set>
#include <queue>

#include <GSLAM/core/Timer.h>
#include <GSLAM/core/Random.h>
#include <GSLAM/core/Event.h>
#include <GSLAM/core/Vocabulary.h>
#include <GSLAM/core/Estimator.h>
#include <GSLAM/core/Messenger.h>

#ifdef HAS_EIGEN3
#include <Eigen/Dense>
#endif

#ifdef HAS_OPENCV
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#endif

#include "../../src/RANSAC.h"

namespace demo{

using GSLAM::PointID;
using GSLAM::FrameID;
using GSLAM::Point3d;
using GSLAM::Point2d;
using GSLAM::Point3f;
using GSLAM::SE3;
using GSLAM::PointPtr;
using GSLAM::FramePtr;
using std::vector;
using std::pair;
using std::make_pair;
using std::string;

typedef GSLAM::FrameConnection FrameConnection;
typedef SPtr<FrameConnection>  FrameConnectionPtr;



class MapFrameConnection : public GSLAM::FrameConnection
{
public:
    virtual std::string type()const{return "MapFrameConnection";}
    virtual int  matchesNum(){return _sharedPoints.size();}

    virtual bool getMatches(std::vector<std::pair<int,int> >& matches){matches=_matches;return true;}
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

    virtual bool insertKeyFrame(const SPtr<MapFrame>& frame) {
        int onlineMode=svar.GetInt("SLAM.isOnline", 0);
        if( svar.GetInt("SLAM.forceOffline", 1) ) onlineMode = 0; // FIXME: force to use offline mode

        if(onlineMode)
        {
            if(_workThread.taskNumLeft()||!svar.GetInt("Mapper.Idle")){
                _abordBundle=true;
                return false;
            }
            auto ret=_workThread.Add([this,frame](){
                svar.GetInt("Mapper.Idle")=0;
                _curFrame=frame;
                handleCurrentFrame();
                svar.GetInt("Mapper.Idle")=1;
            });
        }
        else
        {
            // offline processing, do keyframe insertation in call's thread
            _abordBundle=false;
            _curFrame=frame;

            svar.GetInt("Mapper.Idle")=0;
            handleCurrentFrame();
            svar.GetInt("Mapper.Idle")=1;
        }

        return true;
    }


    virtual void stop(){
        _shouldStop=true;
        if(_loopCloser)
        {
            _loopCloser->stop();
        }
    }

    virtual void mappingThread();

    virtual void call(const std::string &command, void *arg=NULL);

private:
    void  handleCurrentFrame();
    void  makeKeyFrame();
    void  mapPointCulling();
    void  mapFrameCulling();
    void  createNewMapPoints();
    void  createNewMapPointsBow();
    void  dataAssociation();
    int   fuse(const GSLAM::FramePtr &frame);
    int   fuse(const GSLAM::FramePtr& frame,const std::vector<GSLAM::PointPtr>& mappoints);
    void  localOptimization();
    void  localOptimizationGraphBA();
    void  fitGps();
    void  gpsFitting();
    void  updateNormAndDes();
    bool  closeLoop();


    bool eraseMapPoint(const PointPtr& mpt);
    bool eraseMapFrame(const FramePtr& fr);
    bool fuseMapPoint(const PointPtr& from,const PointPtr& to);
    bool triangulate(double* t1,double* t2,const pi::Point3d& xn1,
                     const pi::Point3d& xn2,pi::Point3d& pt);


    GSLAM::SO3 PYR2Rotation(double pitch,double yaw,double roll)
    {
        if(fabs(180-fabs(roll))<10) roll+=180;
        GSLAM::SO3 camera2IMU(-0.5,0.5,-0.5,0.5);
        GSLAM::SO3 imu2world;
        imu2world.FromEulerAngle(-pitch,90.-yaw,roll);
        return imu2world*camera2IMU;
    }

    bool getGPSEdge(const FramePtr& fr,GSLAM::SE3& se3,double* info)
    {
        if(fr->getGPSNum()==0) return false;
        GSLAM::Point3d lla;
        if(!fr->getGPSLLA(lla)) return false;

        Point3d gpsSigma;
        if(!fr->getGPSLLASigma(gpsSigma)) {gpsSigma=Point3d(5,5,10);}
        if(gpsSigma.norm()>100) return false;

        // Local to ECEF
        GSLAM::SE3 local2ECEF;
        local2ECEF.get_translation()=GSLAM::GPS<>::GPS2XYZ(lla.y,lla.x,lla.z);
        double D2R=3.1415925/180.;
        double lon=lla.x*D2R;
        double lat=lla.y*D2R;
        GSLAM::Point3d up(cos(lon)*cos(lat), sin(lon)*cos(lat), sin(lat));
        GSLAM::Point3d east(-sin(lon), cos(lon), 0);
        GSLAM::Point3d north=up.cross(east);
        double R[9]={east.x, north.x, up.x,
                     east.y, north.y, up.y,
                     east.z, north.z, up.z};
        local2ECEF.get_rotation().fromMatrix(R);

        Point3d pyr(-90,0,0),pyrSigma(1e100,1e100,1e100);
        GSLAM::SE3 camera2local;
        if(!fr->getPitchYawRoll(pyr)){
            GSLAM::SE3 localPose=local2ECEF.inverse()*fr->getPose();
            localPose.getRotation().getMatrix(R);
            Point3d yaxiz=Point3d(0,0,-1).cross(Point3d(R[0],R[3],R[6]));
            R[0];R[1]=yaxiz.x;R[2]=0;
            R[3];R[4]=yaxiz.y;R[5]=0;
            R[6]=0;R[7]=yaxiz.z;R[8]=-1;
            localPose.get_rotation().fromMatrix(R);
            camera2local=GSLAM::SE3(localPose.get_rotation(),
                                    GSLAM::Point3d(0,0,0));

            pyrSigma=Point3d(1e2,1e10,1e2);
        }
        else
        {
            // Camera to local : east up north
            camera2local=GSLAM::SE3(PYR2Rotation(pyr.x,pyr.y,pyr.z),
                                    GSLAM::Point3d(0,0,0));
            if(!fr->getPYRSigma(pyrSigma))
            {
                pyrSigma=Point3d(1,100,1);
            }
        }

        // Camera to ECEF
        se3= local2ECEF*camera2local;

        typedef Eigen::Matrix<double,6,6> Matrix6d;
        typedef Eigen::Vector3d           Vector3d;
        Eigen::Map<Matrix6d> infoM(info);
        infoM=Matrix6d::Identity();
        double gpsK=svar.GetDouble("GPS.K",1e-4);
        infoM(0,0)=gpsK/(gpsSigma.x*gpsSigma.x);
        infoM(1,1)=gpsK/(gpsSigma.y*gpsSigma.y);
        infoM(2,2)=gpsK/(gpsSigma.z*gpsSigma.z);
        infoM(3,3)=1./(pyrSigma.x*pyrSigma.x);
        infoM(4,4)=1./(pyrSigma.z*pyrSigma.z);
        infoM(5,5)=1./(pyrSigma.y*pyrSigma.y);
        return true;
    }

    std::queue<SPtr<MapFrame> > _keyframeCandidates;
    std::map<GSLAM::PointID,GSLAM::PointPtr> _updatedPts;
    SPtr<GSLAM::Optimizer>     _optimizer;
    SPtr<MapFrame>             _curFrame,_lastFrame;
    SPtr<Matcher>              _matcher;
    LoopCloserPtr              _loopCloser;
    GSLAM::Event               _insertionEnabled;

    std::list<PointPtr>        _recentPoints;
    bool                       _shouldStop,_abordBundle;
    std::stringstream          _logger;

    GSLAM::Publisher           _pubMapFusionMap;
    GSLAM::detail::ThreadPool          _workThread;
};

Mapper::Mapper(const GSLAM::MapPtr &map)
    : ::Mapper(map),_workThread(1)
{
    _matcher=Matcher::create(svar.GetString("Mapper.Matcher","bow"));

    // create optimization obj
    // FIXME: because Optimizer is integrated in this project, therefore, we just create it
    _optimizer=GSLAM::Optimizer::create(svar.GetString("BundlePlugin","libgslam_optimizerDIYSLAM"));

    if(!_optimizer)
        _optimizer=createOptimizerInstance();

    svar.GetDouble("Optimizer.MaxSolverTime",1e3)=0.5;
    svar.GetPointer("BundleStopFlagPointer")=&_abordBundle;

    if(!_optimizer)
    {
        LOG(FATAL)<<"No valid Optimizer!";
    }
    if(!_matcher){
        LOG(FATAL)<<"No valid Matcher!";
    }

    _loopCloser=LoopCloser::create(map,svar.GetString("LoopCloser","se3graph"));
    if(!_loopCloser)
        LOG(WARNING)<<"No LoopCloser.";
    _pubMapFusionMap=GSLAM::Messenger::instance().advertise<GSLAM::Map>("fitted_map",0);
}

void Mapper::mappingThread()
{
    _shouldStop=false;
    _insertionEnabled.set();

    while(true)
    {
        if(_shouldStop) break;

        if(_keyframeCandidates.empty())
        {
            _insertionEnabled.set();
            svar.GetInt("Mapper.Idle")=1;
            GSLAM::Rate::sleep(0.001);
            continue;
        }

        svar.GetInt("Mapper.Idle")=0;
        _curFrame=_keyframeCandidates.front();
        handleCurrentFrame();
        _keyframeCandidates.pop();
        _insertionEnabled.set();
        svar.GetInt("Mapper.Idle")=1;
    }

    if(_handle)
        _handle->handle(_lastFrame);
}

void Mapper::call(const std::string &command, void *arg)
{
    if( command == "Mapper.WaitFinised" )
    {
        GSLAM::Rate r(30);
        int& mapperIdle = svar.GetInt("Mapper.Idle", 1);

        while( mapperIdle == 0 )
        {
            r.sleep();
        }
    }
}


void  Mapper::handleCurrentFrame()
{
    GSLAM::ScopedTimer handleTimer("Mapper::handleCurrentFrame");
    _logger.str("");
    _logger<<"New KeyFrame:"<<_curFrame->id();

    mapPointCulling();
    makeKeyFrame();
    if(_map->frameNum()>2)
    {
        int createMapPointsMode=svar.GetInt("SLAM.MapPointCreationMode",1);
        if(createMapPointsMode&1)
            createNewMapPointsBow();
        if(createMapPointsMode&2)
            createNewMapPoints();
        dataAssociation();
        closeLoop();
    }
    if(_map->frameNum()>=2&&_lastFrame)
    {
        localOptimization();
        updateNormAndDes();
        mapFrameCulling();
    }

    if(_lastFrame)
    {
        if(!svar.GetInt("GPS.LocalOptimize",1)){
            int nFrame=svar.GetInt("GPS.NFrame2FitGPS");
            if(_map->frameNum()==nFrame){
                fitGps();
            }
            else{
                if(_handle)
                    _handle->handle(_lastFrame);
                else
                    _lastFrame->setImage(GSLAM::GImage());
            }
        }
        else {
            if(_handle)
                _handle->handle(_lastFrame);
            else
                _lastFrame->setImage(GSLAM::GImage());
        }
    }
    _lastFrame=_curFrame;

    if(svar.GetInt("SLAM.Verbose")&2)
        LOG(INFO)<<_logger.str();

    //_insertionEnabled.set();
    _curFrame.reset();
}

void  Mapper::makeKeyFrame()
{
    _updatedPts.clear();
    _map->insertMapFrame(_curFrame);
    if(_map->frameNum()<=2) return;
    std::map<FrameID,FrameConnectionPtr> parents;

    std::map<PointID,size_t> observes;// featureID
    _curFrame->getObservations(observes);
    for(pair<PointID,size_t> obs : observes)
    {
        const PointPtr& pt=_curFrame->keyPoint(obs.second)._mapPoint;
        if(!pt->observationNum()){
            _curFrame->eraseObservation(pt);
            continue;
        }
        if(!pt->addObservation(_curFrame->id(),obs.second))
        {
//            LOG(WARNING)<<"This should not happen.";
        }
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
        _updatedPts.insert(std::make_pair(pt->id(),pt));
    }

    for(pair<FrameID,FrameConnectionPtr> it:parents)
    {
        if(it.second->matchesNum()>10)
        {
            if(it.first==_curFrame->id()) continue;
            FramePtr parent=_map->getFrame(it.first);
            if(!parent) continue;
            auto con=_curFrame->getParent(parent->id());
            if(con)
            {
                MapFrameConnection* curCon=dynamic_cast<MapFrameConnection*>(it.second.get());
                vector<pair<int,int> >   matches;
                con->getMatches(matches);
//                LOG(INFO)<<"Make Tracked Connection "<<curCon->_matches.size()<<"->"<<matches.size();
                curCon->_matches=matches;

            }
//            else{
//                LOG(INFO)<<"New Connection.";
//            }
            parent->addChildren(_curFrame->id(),it.second);
            _curFrame->addParent(it.first,it.second);
        }
    }
}

void  Mapper::mapFrameCulling()
{
    if(!svar.GetInt("Mapper.MapFrameCulling",0)) return;
    GSLAM::FrameConnectionMap parents;
    if(!_curFrame->getParents(parents)) return;
    for(auto parent:parents)
    {
        GSLAM::FramePtr fr=_map->getFrame(parent.first);
        if(!fr) continue;
        assert(fr->id()==_curFrame->id());

        std::map<GSLAM::PointID,size_t> observations=fr->getObservations();

        int nRedundantObservations=0;
        int nMPs=0;
        for(auto obs:observations)
        {
            PointPtr pt=_map->getPoint(obs.first);
            if(!pt) continue;
            nMPs++;
            if(pt->observationNum()>=4) nRedundantObservations++;
        }

        if(nRedundantObservations>0.9*nMPs){
            eraseMapFrame(fr);
            LOG(INFO)<<"Erased frame "<<fr->id();
        }
    }
}

void  Mapper::mapPointCulling()
{
    GSLAM::ScopedTimer mTimer("Mapper::mapPointCulling");
    if(_map->frameNum()<=2){
        _recentPoints.clear();
        return;
    }
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
    _logger<<",ER:"<<cullCount;
//    _logger<<"New KeyFrame:"<<_curFrame->id();
}

void  Mapper::createNewMapPointsBow()
{
    GSLAM::ScopedTimer createTimer("Mapper::createNewMapPointsBow");
    /// 1. Collect nearby keyframes
    size_t createdCount=_recentPoints.size();
    std::map<GSLAM::FrameID,SPtr<GSLAM::FrameConnection> > parents;
    if(!_curFrame->getParents(parents))
    {
        LOG(ERROR)<<"Failed to get connects!";
        return;
    }

    if(parents.empty())
    {
        LOG(ERROR)<<"Parents is empty!";
        return ;
    }

    std::vector<std::pair<int,GSLAM::FrameID> > connects;
    connects.reserve(parents.size());
    for(auto con:parents)
        connects.push_back(std::make_pair(con.second->matchesNum(),con.first));
    std::sort(connects.begin(),connects.end());

    GSLAM::SE3    curc2w=_curFrame->getPose();
    GSLAM::SE3    curw2c=curc2w.inverse();
    pi::Point3d   curt=curc2w.get_translation();
    GSLAM::Camera curCam=_curFrame->getCamera();
    float         mDepth=_curFrame->getMedianDepth();
    double iden[12],cur2refM[12];
    GSLAM::SE3 Iden;
    Iden.getMatrix(iden);
    int mapperMatchMode=svar.GetInt("Mapper.MatchEpipolar",0);
    int triangulateKFNum=svar.GetInt("Mapper.TriangulateKFNum",4);
    bool triangulateCheck=svar.GetInt("Mapper.TriangulateCheck",1);
    for(int i=connects.size()-1,iend=connects.size()-triangulateKFNum;i>=0&&i>iend;i--)
    {
        std::pair<int,GSLAM::FrameID> con=connects[i];
        GSLAM::FramePtr refKF=_map->getFrame(con.second);
        if(!refKF.get()) continue;

        GSLAM::SE3  ref2cur  =curw2c*refKF->getPose();
        GSLAM::SE3  cur2ref  =ref2cur.inverse();
        pi::Point3d vBaseline=ref2cur.get_translation();
        GSLAM::Camera refCam =refKF->getCamera();
        cur2ref.getMatrix(cur2refM);

        if(vBaseline.norm()/mDepth<0.01) continue;

        auto connection=_curFrame->getParent(refKF->id());
        std::vector<std::pair<int,int> > matches;
        if(connection) connection->getMatches(matches);
        if(matches.size()==connection->matchesNum()){
//            LOG(INFO)<<"Matches:"<<matches.size()<<",Shared:"<<connection->matchesNum();
//            continue;
            if(mapperMatchMode)
                _matcher->match4triangulation(refKF,_curFrame,matches);
            else
                _matcher->match4initialize(refKF,_curFrame,matches);
        }
//        else{
//            LOG(INFO)<<"Using Tracked Matches.";
//        }

        int successCount=0,count1=0,count2=0;
        for(auto& m:matches)
        {
            if(_curFrame->keyPoint(m.second)._mapPoint&&triangulateCheck) continue;
            GSLAM::Point2f refkp,curkp;
            refKF->getKeyPoint(m.first,refkp);
            _curFrame->getKeyPoint(m.second,curkp);

            /// B. Check parallax between rays
            const pi::Point3d &curP3d = curCam.UnProject(curkp.x,curkp.y);
            const pi::Point3d &curRay = curP3d;
            const pi::Point3d &refP3d = refCam.UnProject(refkp.x,refkp.y);
            const pi::Point3d &refRay = ref2cur.get_rotation()*refP3d;
            const double cosParallaxRays = (curRay*refRay)/(curRay.norm()*refRay.norm());

            if(cosParallaxRays<0 ||cosParallaxRays>0.9998)//FIXME: fixed in 20150714
                continue;
            count1++;

            //  Linear Triangulation Method
            pi::Point3d p3d;
            if(!triangulate(iden,cur2refM,curP3d,refP3d,p3d)) continue;

            /// C. Triangulation in front of cameras?
            if(p3d.z<=1e-5||p3d.z>=mDepth*10) continue;
            pi::Point3d ptinref=(cur2ref*p3d);
            if(ptinref.z<=1e-5||ptinref.z>=mDepth*10) continue;
            count2++;

            /// D. Check reprojection errors are not too large
            if(0)
            {
                float sigmaSquare = 4;//_curFrame->keyPoint(featId1)._ptUn.size;
                sigmaSquare*=5.991*sigmaSquare;

                pi::Point2d e=curCam.Project(p3d)-pi::Point2d(curkp.x,curkp.y);
                if((e.x*e.x+e.y*e.y)>sigmaSquare)
                    continue;

                e=refCam.Project(ptinref)-pi::Point2d(refkp.x,refkp.y);
                if((e.x*e.x+e.y*e.y)>sigmaSquare)
                    continue;
            }
            else
            {
                if((p3d/p3d.z-curP3d).norm()>=0.02) continue;
                if((ptinref/ptinref.z-refP3d).norm()>=0.02) continue;
            }

            /// E. Check scale consistency

            /// G. Finally found a new mappoint
            p3d=curc2w*p3d;
            PointPtr pMP(new MapPoint(_map->getPid(),p3d,(curt-p3d).normalize(),
                                      _curFrame->keyPoint(m.second)._color,_curFrame->id()));
            pMP->setDescriptor(_curFrame->getDescriptor(m.second).clone());
            _curFrame->addObservation(pMP,m.second,true);
            refKF->addObservation(pMP,m.first,true);
            _map->insertMapPoint(pMP);
            _recentPoints.push_back(pMP);
            successCount++;

            //计算Plane
            if (!ransac.is_finished())
                ransac.solve(p3d);
        }
        _logger<<",ref:"<<refKF->id()<<"-"<<successCount
              <<"/"<<count2<<"/"<<count1<<"/"<<matches.size();
    }

    createdCount=_recentPoints.size()-createdCount;
    _logger<<", Created "<<createdCount<<" pt";

#ifdef HAS_OPENCV
    if(svar.GetInt("DebugCreateNewMapPoints"))
    {
        cv::Mat img=_curFrame->getImage();
        for(int i=0;i<_curFrame->keyPointNum();i++)
        {
            GSLAM::Point2f pt;
            _curFrame->getKeyPoint(i,pt);
            cv::Scalar color(255,0,0);
            auto mpt=_curFrame->keyPoint(i)._mapPoint;
            if(mpt){
                if(mpt->refKeyframeID()==_curFrame->id()) color=cv::Scalar(0,0,255);// New MapPoints
                else color=cv::Scalar(0,255,0);// Tracked MapPoints
            }

            cv::circle(img,cv::Point2f(pt.x,pt.y),std::ceil(img.cols/500.),color,std::ceil(img.cols/500.));
        }

        //cv::imwrite("debugCreate_"+std::to_string(_curFrame->id())+".jpg",img);
    }
#endif
}

void  Mapper::createNewMapPoints()
{
    GSLAM::ScopedTimer st("Mapper::createNewMapPoints");
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
                                        pt2d2_est.x,pt2d2_est.y,cam.width()/32,featId2)) continue;
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
//    _logger<<",NEW:"<<createdCount<<"/"<<waitTriListCount;

#ifdef HAS_OPENCV
    if(svar.GetInt("DebugCreateNewMapPoints"))
    {
        cv::Mat img=_curFrame->getImage();
        for(int i=0;i<_curFrame->keyPointNum();i++)
        {
            GSLAM::Point2f pt;
            _curFrame->getKeyPoint(i,pt);
            cv::Scalar color(255,0,0);
            auto mpt=_curFrame->keyPoint(i)._mapPoint;
            if(mpt){
                if(mpt->refKeyframeID()==_curFrame->id()) color=cv::Scalar(0,0,255);// New MapPoints
                else color=cv::Scalar(0,255,0);// Tracked MapPoints
            }

            cv::circle(img,cv::Point2f(pt.x,pt.y),std::ceil(img.cols/500.),color,std::ceil(img.cols/500.));
        }

        //cv::imwrite("debugCreate_"+std::to_string(_curFrame->id())+".jpg",img);
    }
#endif

}

void  Mapper::dataAssociation()
{
    GSLAM::ScopedTimer mtimer("Mapper::dataAssociation");

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

    if(svar.GetInt("Mapper.AssociateSecond",1))
    {
        std::map<GSLAM::FrameID,SPtr<GSLAM::FrameConnection> >    fuseKFs=parents;
        for(auto parent:parents){
            GSLAM::FramePtr fr=_map->getFrame(parent.first);
            if(!fr) continue;
            std::map<GSLAM::FrameID,SPtr<GSLAM::FrameConnection> > secondParents;
            if(!fr->getParents(secondParents)) continue;
            for(std::pair<GSLAM::FrameID,SPtr<GSLAM::FrameConnection> > con:secondParents){
                if(fuseKFs.count(con.first)) continue;
                fuseKFs.insert(con);
            }
        }
    }

    if(svar.GetInt("Mapper.AssociateLoop",1))
    {
        GSLAM::LoopCandidates candidates;
        if(!_map->obtainCandidates(_curFrame,candidates)) ;

        int i=0;
        for(GSLAM::LoopCandidate can:candidates)
        {
            if(i++>2) break;
            if(parents.find(can.frameId)!=parents.end()) continue;
            parents.insert(std::make_pair(can.frameId,SPtr<GSLAM::FrameConnection>()));
        }
    }
//    if(i) _logger<<",Loop"<<i;

    int fusedCount=0;
    if(svar.GetInt("Mapper.AssociateLocal",1)){
        std::unordered_set<PointPtr> localPoints;
        for(auto& con:parents)
        {
            GSLAM::FramePtr fr=_map->getFrame(con.first);
            if(!fr.get()) continue;
            std::map<GSLAM::PointID,size_t> observes;
            fr->getObservations(observes);
            SPtr<MapFrame> mf=std::dynamic_pointer_cast<MapFrame>(fr);
            for(pair<GSLAM::PointID,size_t> it:observes){
                PointPtr mp=mf->keyPoint(it.second)._mapPoint;
                if(!mp) continue;
                localPoints.insert(mp);
            }
        }
        fusedCount+=fuse(_curFrame,std::vector<PointPtr>(localPoints.begin(),localPoints.end()));
        _logger<<",FUSE1:"<<fusedCount;
    }

    for(auto& con:parents)
    {
        GSLAM::FramePtr fr=_map->getFrame(con.first);
        if(!fr.get()) continue;
        if(con.second)
            fusedCount+=fuse(fr,curPoints);
        else fusedCount+=fuse(fr);
    }

    _logger<<",FUSE:"<<fusedCount;

    // update connectes
    {
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
                if(ptObs.first==_curFrame->id()) continue;
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
        }

        for(pair<FrameID,FrameConnectionPtr> it:parents)
        {
            if(it.first==_curFrame->id()) continue;
            if(it.second->matchesNum()>10)
            {
                FramePtr parent=_map->getFrame(it.first);
                if(!parent) continue;
                parent->addChildren(_curFrame->id(),it.second);
                _curFrame->addParent(it.first,it.second);
            }
        }
    }
}

int  Mapper::fuse(const GSLAM::FramePtr &frame)// fuse a frame like tracking
{
    SPtr<MapFrame> refFrame=std::dynamic_pointer_cast<MapFrame>(frame);
    if(!refFrame) return 0;
    vector<pair<int,int> > matches;
    bool bOK=_matcher->match4initialize(refFrame,_curFrame,matches);
//    _logger<<",trackref"<<frame->id()<<","<<matches.size();
    if(!bOK)
        return 0;

    int fuseCount=0;
    for(auto m:matches)
    {
        GSLAM::PointID refId=refFrame->getKeyPointObserve(m.first);
        GSLAM::PointID curId=_curFrame->getKeyPointObserve(m.second);
        if(refId==curId) continue;
        if(!refId&&!curId) continue;
        GSLAM::PointPtr refPoint=_map->getPoint(refId);
        GSLAM::PointPtr curPoint=_map->getPoint(curId);
        if(refPoint&&curPoint) fuseMapPoint(curPoint,refPoint);
        if(refPoint&&!curPoint) _curFrame->addObservation(refPoint,m.second,true);
        if(!refPoint&&curPoint) refFrame->addObservation(curPoint,m.first,true);
        fuseCount++;
    }


    return fuseCount;
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
                            pc2.x,pc2.y,cam.width()*0.004,
                            idx,false)) continue;

        if(fr->getKeyPointObserve(idx))//existed mappoint
        {
            auto from=fr->keyPoint(idx)._mapPoint;
            if(from==p) continue;
            if(!fuseMapPoint(from,p))
            {
//                DLOG(ERROR)<<"Failed to fuse mappoint.";
            }
        }
        else// new observation!
        {
            fr->addObservation(p,idx,true);
        }
        _updatedPts.insert(std::make_pair(p->id(),p));
        nfused++;
    }
    return nfused;
}

void  Mapper::localOptimizationGraphBA()
{
    using namespace GSLAM;
    if(!_optimizer)
    {
        LOG(ERROR)<<"No valid optimizer!";
        return;
    }
    GSLAM::ScopedTimer mTimer("Mapper::localOptimization");

    // collect local frames
    typedef std::unordered_map<GSLAM::FrameID,uint> FrameIDMap;
    typedef std::unordered_map<GSLAM::PointID,uint> PointIDMap;

    GSLAM::BundleGraph graph;

    vector<GSLAM::FramePtr> KeyFrameEles;
    FrameIDMap frameId_map;
    vector<GSLAM::PointPtr> PointEles;
    PointIDMap pointId_map;

    graph.keyframes.reserve(_map->frameNum());
    KeyFrameEles.reserve(_map->frameNum());

    graph.mappoints.reserve(_map->pointNum());
    PointEles.reserve(_map->pointNum());

    // FIXME: use _lastFrame is better?
    GSLAM::SE3 l2w=_curFrame->getPose();//svar.GetInt("GPS.Fitted")?GSLAM::SE3():
    GSLAM::SE3 w2l=l2w.inverse();

    //1. Collect Local Keyframes and insert current MapPoints
    //1.1. Insert current frame
//    graph.keyframes.push_back({w2l*_curFrame->getPose(),UPDATE_KF_SE3});// FIXME: why this not work on windows
    graph.keyframes.push_back({GSLAM::SE3(w2l.get_rotation()*_curFrame->getPose().get_rotation(),
                               w2l.get_translation() + w2l.get_rotation()*_curFrame->getPose().get_translation()),UPDATE_KF_SE3});
    KeyFrameEles.push_back(_curFrame);
    frameId_map.insert(std::make_pair(_curFrame->id(),0));

    std::set<GSLAM::PointPtr> points2remove;
    int fixedCount=0;
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
    GSLAM::BundleEdge bobs;
    for(uint  pt_bundle_id=0;pt_bundle_id<PointEles.size();pt_bundle_id++)
    {
        GSLAM::PointPtr& pt=PointEles[pt_bundle_id];
        if(pt->observationNum()<2)
        {
//            eraseMapPoint(pt);
            points2remove.insert(pt);
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
                graph.keyframes.push_back({GSLAM::SE3(w2l.get_rotation()*fr->getPose().get_rotation(),
                                           w2l.get_translation() + w2l.get_rotation()*fr->getPose().get_translation()),UPDATE_KF_SE3});
                KeyFrameEles.push_back(fr);
                fixedCount++;
            }
            else
            {
                fr=KeyFrameEles[id_it->second];
                bobs.frameId=id_it->second;
            }
            GSLAM::KeyPoint kp=((::MapFrame*)fr.get())->keyPoint(obs.second)._pt;
            bobs.measurement=fr->getCamera().UnProject(kp.pt.x,kp.pt.y);
            bobs.pointId=pt_bundle_id;
            graph.mappointObserves.push_back(bobs);
        }
    }

    int moveSize=svar.GetInt("Mapper.PoseGraph")?KeyFrameEles.size():0;
    for(int i=1;i<moveSize;i++){
        FramePtr& fr=KeyFrameEles[i];
        auto parents=fr->getParents();
        if(parents.empty()) continue;

        for(auto parent:parents){
            auto it=frameId_map.find(parent.first);
            FramePtr ref;
            size_t   idx;
            if(it==frameId_map.end()){
                ref=_map->getFrame(parent.first);
                idx=graph.keyframes.size();
                frameId_map.insert(make_pair(parent.first,graph.keyframes.size()));
                graph.keyframes.push_back({GSLAM::SE3(w2l.get_rotation()*ref->getPose().get_rotation(),
                                           w2l.get_translation() + w2l.get_rotation()*ref->getPose().get_translation()),UPDATE_KF_NONE});
                KeyFrameEles.push_back(ref);
            }
            else{
                idx=it->second;
                ref=KeyFrameEles[idx];
            }

            graph.se3Graph.push_back({(GSLAM::FrameID) idx, (GSLAM::FrameID) i,
                                      ref->getPose().inverse()*fr->getPose()});
        }
    }

    vector<double>  informations(KeyFrameEles.size()*36);
    if( svar.GetInt("GPS.Fitted") && svar.GetInt("GPS.LocalOptimize",1) > 0 )
    {
        GSLAM::SE3 gpsSe3;
        if(svar.GetInt("GPS.RelaxLocalDOF",1)){
            FrameID minFrameId=0,minIndex=-1;
            for(int i=0;i<graph.keyframes.size();++i){
                KeyFrameEstimzation& kf=graph.keyframes[i];
                FramePtr& fr=KeyFrameEles[i];
                if(minIndex<0||minFrameId>fr->id())
                {
                    if(minIndex>=0)
                        graph.keyframes[minIndex].dof=UPDATE_KF_SE3;
                    minFrameId=fr->id();
                    minIndex=i;
//                    graph.keyframes[minIndex].dof=UPDATE_KF_NONE;
                }
                else
                    graph.keyframes[i].dof=UPDATE_KF_SE3;
            }
        }
        for(size_t i=0;i<KeyFrameEles.size();i++)
        {
            FramePtr& fr=KeyFrameEles[i];
            double*   info=&informations[36*i];
            if(!getGPSEdge(fr,gpsSe3,info)) continue;
            graph.gpsGraph.push_back({i,w2l*gpsSe3,info});
        }
    }
    else if(!fixedCount){
        FrameID minId=INT_MAX,minIdx=0;
        for(size_t i=0;i<KeyFrameEles.size();i++)
        {
            FramePtr& fr=KeyFrameEles[i];
            if(fr->id()<minId) {
                minId=fr->id();
                minIdx=i;
            }
            graph.keyframes[i].dof=UPDATE_KF_NONE;
        }
    }
//    timer.leave("Mapper::localBundle::1.Collect");

    //    static TimerEventThread timer(svar.GetInt("MaxBundleOptimizeTime",200)
    //                                  ,&_stopBARequrested);
    //    timer.start();

    LOG(INFO)<<"LocalOptimizationGraphBA:"<<",BA:"<<graph.keyframes.size()
            <<"-"<<graph.mappoints.size()
           <<"-"<<graph.mappointObserves.size()
          <<"-"<<graph.se3Graph.size()
         <<"-"<<graph.gpsGraph.size();

        _logger<<",BA:"<<graph.keyframes.size()
              <<"-"<<graph.mappoints.size()
             <<"-"<<graph.mappointObserves.size();

//    timer.enter("Mapper::localBundle::2.Optimize");
    if(!_optimizer->optimize(graph))
    {
        LOG(ERROR)<<"Failed to optimize graph!";
        return;
    }
//    timer.leave("Mapper::localBundle::2.Optimize");

//    timer.enter("Mapper::localBundle::3.Update");
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
    int removeBad=svar.GetInt("KeyFrameHandle.RemoveBadEdgeLocally",9);
    if(removeBad)
    {
        vector<double> errors;
        double         sumError=0;
        errors.reserve(graph.mappointObserves.size());
        for(int i=0;i<graph.mappointObserves.size();i++)
        {
            BundleEdge& obs=graph.mappointObserves[i];
            Point3d     p3d=graph.keyframes[obs.frameId].estimation.get_se3().inverse()*graph.mappoints[obs.pointId].first;
            Point2d     err=Point2d(p3d.x/p3d.z,p3d.y/p3d.z)-Point2d(obs.measurement.x,obs.measurement.y);
            double     error=err.x*err.x+err.y*err.y;
            errors.push_back(error);
            sumError+=error;
        }

        double averageError=sumError/errors.size();
        double errorThreshold=std::min(averageError*removeBad,1e-5);
        int    eraseObsCount=0;
        for(int i=0;i<errors.size();i++)
        {
            if(errors[i]>errorThreshold)
            {
                GSLAM::PointPtr& pt=PointEles[graph.mappointObserves[i].pointId];
                GSLAM::FramePtr& fr=KeyFrameEles[graph.mappointObserves[i].frameId];
                fr->eraseObservation(pt,true);
                eraseObsCount++;
                if(pt->observationNum()<2)
                {
                    points2remove.insert(pt);
                }
            }
        }
        _logger<<",EO:"<<eraseObsCount
              <<",EP:"<<points2remove.size();
        for(auto& pt:points2remove)
            eraseMapPoint(pt);
    }

//    if(_map->frameNum()==5)
//        gpsFitting();
//    timer.leave("Mapper::localBundle::3.Update");

}
void  Mapper::localOptimization()
{
    if(svar.GetInt("Mapper.PoseGraph"))
        return localOptimizationGraphBA();
    using namespace GSLAM;
    if(!_optimizer)
    {
        LOG(ERROR)<<"No valid optimizer!";
        return;
    }
    GSLAM::ScopedTimer mTimer("Mapper::localOptimization");

    // collect local frames
    typedef std::unordered_map<GSLAM::FrameID,uint> FrameIDMap;
    typedef std::unordered_map<GSLAM::PointID,uint> PointIDMap;

    GSLAM::BundleGraph graph;

    vector<GSLAM::FramePtr> KeyFrameEles;
    FrameIDMap frameId_map;
    vector<GSLAM::PointPtr> PointEles;
    PointIDMap pointId_map;

    graph.keyframes.reserve(_map->frameNum());
    KeyFrameEles.reserve(_map->frameNum());

    graph.mappoints.reserve(_map->pointNum());
    PointEles.reserve(_map->pointNum());

    // FIXME: use _lastFrame is better?
    GSLAM::SE3 l2w=_curFrame->getPose();//svar.GetInt("GPS.Fitted")?GSLAM::SE3():
    GSLAM::SE3 w2l=l2w.inverse();
    //1. Collect Local Keyframes and insert current MapPoints
    //1.1. Insert current frame
//    graph.keyframes.push_back({w2l*_curFrame->getPose(),UPDATE_KF_SE3});// FIXME: why this not work on windows
    graph.keyframes.push_back({GSLAM::SE3(w2l.get_rotation()*_curFrame->getPose().get_rotation(),
                               w2l.get_translation() + w2l.get_rotation()*_curFrame->getPose().get_translation()),UPDATE_KF_SE3});
    KeyFrameEles.push_back(_curFrame);
    frameId_map.insert(std::make_pair(_curFrame->id(),0));

    //1.2. Collect connected keyframes
    if(svar.GetInt("Mapper.OptimizePrecisely",1)){
        std::map<GSLAM::FrameID,SPtr<GSLAM::FrameConnection> >    parents;
        if(!((::MapFrame*)_curFrame.get())->getParents(parents))
        {
            LOG(ERROR)<<"Failed to obtain parents!";
            return;
        }
        for(auto& con:parents)
        {
            GSLAM::FramePtr kf=_map->getFrame(con.first);
            if(!kf.get()) continue;
            frameId_map.insert(make_pair(con.first,graph.keyframes.size()));
            graph.keyframes.push_back({w2l*kf->getPose(),UPDATE_KF_SE3});
            KeyFrameEles.push_back(kf);
        }
    }
    int unFixKFNum=KeyFrameEles.size();

//    timer.enter("Mapper::localBundle::1.Collect");

    std::set<GSLAM::PointPtr> points2remove;
    int fixedCount=0;
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
                if(!pt) continue;
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
    GSLAM::BundleEdge bobs;
    for(uint  pt_bundle_id=0;pt_bundle_id<PointEles.size();pt_bundle_id++)
    {
        GSLAM::PointPtr& pt=PointEles[pt_bundle_id];
        if(pt->observationNum()<2)
        {
//            eraseMapPoint(pt);
            points2remove.insert(pt);
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
                graph.keyframes.push_back({GSLAM::SE3(w2l.get_rotation()*fr->getPose().get_rotation(),
                                           w2l.get_translation() + w2l.get_rotation()*fr->getPose().get_translation()),UPDATE_KF_NONE});
                KeyFrameEles.push_back(fr);
                fixedCount++;
            }
            else
            {
                fr=KeyFrameEles[id_it->second];
                bobs.frameId=id_it->second;
            }
            GSLAM::KeyPoint kp=((::MapFrame*)fr.get())->keyPoint(obs.second)._pt;
            bobs.measurement=fr->getCamera().UnProject(kp.pt.x,kp.pt.y);
            bobs.pointId=pt_bundle_id;
            graph.mappointObserves.push_back(bobs);
        }
    }

    vector<double>  informations(KeyFrameEles.size()*36);
    if( svar.GetInt("GPS.Fitted") && svar.GetInt("GPS.LocalOptimize",1) > 0 )
    {
        GSLAM::SE3 gpsSe3;
        if(svar.GetInt("GPS.RelaxLocalDOF",1)){
            FrameID minFrameId=0,minIndex=-1;
            for(int i=0;i<graph.keyframes.size();++i){
                KeyFrameEstimzation& kf=graph.keyframes[i];
                FramePtr& fr=KeyFrameEles[i];
                if(minIndex<0||minFrameId>fr->id())
                {
                    if(minIndex>=0)
                        graph.keyframes[minIndex].dof=UPDATE_KF_SE3;
                    minFrameId=fr->id();
                    minIndex=i;
//                    graph.keyframes[minIndex].dof=UPDATE_KF_NONE;
                }
                else
                    graph.keyframes[i].dof=UPDATE_KF_SE3;
            }
        }
        for(size_t i=0;i<KeyFrameEles.size();i++)
        {
            FramePtr& fr=KeyFrameEles[i];
            double*   info=&informations[36*i];
            if(!getGPSEdge(fr,gpsSe3,info)) continue;
            graph.gpsGraph.push_back({i,w2l*gpsSe3,info});
        }
    }
    else if(!fixedCount){
        FrameID minId=INT_MAX,minIdx=0;
        for(size_t i=0;i<KeyFrameEles.size();i++)
        {
            FramePtr& fr=KeyFrameEles[i];
            if(fr->id()<minId) {
                minId=fr->id();
                minIdx=i;
            }
            graph.keyframes[i].dof=UPDATE_KF_NONE;
        }
    }
//    timer.leave("Mapper::localBundle::1.Collect");

    //    static TimerEventThread timer(svar.GetInt("MaxBundleOptimizeTime",200)
    //                                  ,&_stopBARequrested);
    //    timer.start();


        _logger<<",BA:"<<graph.keyframes.size()
              <<"-"<<graph.mappoints.size()
             <<"-"<<graph.mappointObserves.size();

//    timer.enter("Mapper::localBundle::2.Optimize");
    if(!_optimizer->optimize(graph))
    {
        LOG(ERROR)<<"Failed to optimize graph!";
        return;
    }
//    timer.leave("Mapper::localBundle::2.Optimize");

//    timer.enter("Mapper::localBundle::3.Update");
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
    int removeBad=svar.GetInt("KeyFrameHandle.RemoveBadEdgeLocally",9);
    if(removeBad)
    {
        vector<double> errors;
        double         sumError=0;
        errors.reserve(graph.mappointObserves.size());
        for(int i=0;i<graph.mappointObserves.size();i++)
        {
            BundleEdge& obs=graph.mappointObserves[i];
            Point3d     p3d=graph.keyframes[obs.frameId].estimation.get_se3().inverse()*graph.mappoints[obs.pointId].first;
            Point2d     err=Point2d(p3d.x/p3d.z,p3d.y/p3d.z)-Point2d(obs.measurement.x,obs.measurement.y);
            double     error=err.x*err.x+err.y*err.y;
            errors.push_back(error);
            sumError+=error;
        }

        double averageError=sumError/errors.size();
        double errorThreshold=std::min(averageError*removeBad,1e-5);
        int    eraseObsCount=0;
        for(int i=0;i<errors.size();i++)
        {
            if(errors[i]>errorThreshold)
            {
                GSLAM::PointPtr& pt=PointEles[graph.mappointObserves[i].pointId];
                GSLAM::FramePtr& fr=KeyFrameEles[graph.mappointObserves[i].frameId];
                fr->eraseObservation(pt,true);
                eraseObsCount++;
                if(pt->observationNum()<2)
                {
                    points2remove.insert(pt);
                }
                else if(pt->refKeyframeID()+3<=_curFrame->id())
                {
                    if(pt->observationNum()<=2)
                    {
                        points2remove.insert(pt);
                    }
                }
            }
        }
        _logger<<",EO:"<<eraseObsCount
              <<",EP:"<<points2remove.size();
        for(auto& pt:points2remove)
            eraseMapPoint(pt);
    }

//    if(_map->frameNum()==5)
//        gpsFitting();
//    timer.leave("Mapper::localBundle::3.Update");

}

void  Mapper::fitGps()
{
    auto _curMap=_map;
    if( !_curMap ) return;

    GSLAM::FrameArray frames=_curMap->getFrames();
    SPtr<GSLAM::Estimator>  estimator=createEstimatorInstance();
    if(!estimator) return;
    GSLAM::SIM3 sim3;
    std::vector<GSLAM::Point3d> from,to;
    for(GSLAM::FramePtr fr:frames){
        GSLAM::Point3d xyz;
        if(!fr->getGPSECEF(xyz)) continue;
        from.push_back(fr->getPose().get_translation());
        to.push_back(xyz);
    }

    if(from.size()<3) return;
    if(!estimator->findSIM3(sim3,from,to)) return;


    for(GSLAM::FramePtr fr:frames){
        GSLAM::Point3d xyz;
        if(!fr->getGPSECEF(xyz)) continue;

        GSLAM::Point3d pyr;
        bool hasPyr=fr->getPitchYawRoll(pyr);

        GSLAM::Point3d lla;
        if(!fr->getGPSLLA(lla)) continue;
        GSLAM::SIM3 local2ECEF;
        local2ECEF.get_translation()=xyz;
        double D2R=3.1415925/180.;
        double lon=lla.x*D2R;
        double lat=lla.y*D2R;
        GSLAM::Point3d up(cos(lon)*cos(lat), sin(lon)*cos(lat), sin(lat));
        GSLAM::Point3d east(-sin(lon), cos(lon), 0);
        GSLAM::Point3d north=up.cross(east);
        double R[9]={east.x, north.x, up.x,
                     east.y, north.y, up.y,
                     east.z, north.z, up.z};
        local2ECEF.get_rotation().fromMatrix(R);

        if(hasPyr)
        {
            auto camera2local=GSLAM::SE3(PYR2Rotation(pyr.x,pyr.y,pyr.z),
                                    GSLAM::Point3d(0,0,0));
            to.push_back(local2ECEF*camera2local*GSLAM::Point3d(0,0,-1.));
        }
        else
            to.push_back(local2ECEF*GSLAM::Point3d(0,0,1.));

        from.push_back(fr->getPose()*GSLAM::Point3d(0,0,-1./sim3.get_scale()));
    }

    if(!estimator->findSIM3(sim3,from,to)) return;

    for(GSLAM::FramePtr fr:frames){
        fr->setPose(sim3*fr->getPoseScale());
    }

    GSLAM::PointArray points=_curMap->getPoints();
    for(GSLAM::PointPtr pt:points){
        pt->setPose(sim3*pt->getPose());
    }

    if(_handle) _handle->handle(_curMap);
    _pubMapFusionMap.publish(_curMap);
}

void  Mapper::gpsFitting()
{
    using namespace GSLAM;

    if(!svar.GetInt("GPS.Fitted")) return ;

    SPtr<Optimizer> opt=Optimizer::create("libgslam_optimizer");
    if(!opt) return;

    GSLAM::ScopedTimer mt("Mapper::gpsFitting");
    // collect local frames
    typedef std::unordered_map<GSLAM::FrameID,uint> FrameIDMap;
    using namespace GSLAM;

    GSLAM::BundleGraph graph;

    vector<GSLAM::FramePtr> KeyFrameEles;
    FrameIDMap frameId_map;

    graph.keyframes.reserve(_map->frameNum());
    KeyFrameEles.reserve(_map->frameNum());

    graph.keyframes.push_back({_curFrame->getPoseScale(),UPDATE_KF_SE3});
    KeyFrameEles.push_back(_curFrame);
    frameId_map.insert(std::make_pair(_curFrame->id(),0));

    //1.2. Collect connected keyframes
    std::map<GSLAM::FrameID,SPtr<GSLAM::FrameConnection> >    parents;
    if(!((::MapFrame*)_curFrame.get())->getParents(parents)) return;
    for(auto& con:parents)
    {
        GSLAM::FramePtr kf=_map->getFrame(con.first);
        if(!kf.get()) continue;
        GSLAM::FrameID frid=graph.keyframes.size();
        graph.sim3Graph.push_back({frid,0,kf->getPoseScale().inv()*_curFrame->getPoseScale()});
        frameId_map.insert(make_pair(con.first,frid));
        graph.keyframes.push_back({kf->getPose(),UPDATE_KF_SE3});
        KeyFrameEles.push_back(kf);

        std::map<GSLAM::FrameID,SPtr<GSLAM::FrameConnection> >    pparents;
        if(!kf->getParents(pparents)) continue;
        for(auto& pp:pparents)
        {
            GSLAM::FramePtr ppk=_map->getFrame(pp.first);
            if(!ppk.get()) continue;
            auto it=frameId_map.find(pp.first);
            if(it==frameId_map.end())
            {
                GSLAM::FrameID ppid=graph.keyframes.size();
                graph.keyframes.push_back({ppk->getPose(),UPDATE_KF_NONE});
                KeyFrameEles.push_back(ppk);
                frameId_map.insert(make_pair(con.first,ppid));
                graph.sim3Graph.push_back({ppid,frid,ppk->getPoseScale().inv()*kf->getPoseScale()});
            }
            else
            {
                graph.sim3Graph.push_back({frameId_map[ppk->id()],frid,ppk->getPoseScale().inv()*kf->getPoseScale()});
            }
        }
    }

    double* gpsInfo=(double*)svar.GetPointer("GPSInfo");
    GSLAM::SIM3 gpsSim3;
    for(size_t i=0;i<KeyFrameEles.size();i++)
    {
        ::MapFrame* fr=(::MapFrame*)KeyFrameEles[i].get();
        if(fr->getPrioryPose(gpsSim3))
            graph.gpsGraph.push_back({i,gpsSim3.get_se3(),gpsInfo});
    }

//    PlyObject ply("GPSFitting.ply");
//    GSLAM::SE3 w2l=_curFrame->getPose().inverse();

//    for(KeyFrameEstimzation kf:graph.keyframes)
//        ply.addPoint(w2l*kf.estimation.get_translation(),Point3ub(0,0,255));

//    for(SIM3Edge e:graph.sim3Graph)
//        ply.addLine(w2l*graph.keyframes[e.firstId].estimation.get_translation(),
//                    w2l*graph.keyframes[e.secondId].estimation.get_translation(),Point3ub(0,0,255));

//    for(GPSEdge e:graph.gpsGraph)
//        ply.addLine(w2l*graph.keyframes[e.frameId].estimation.get_translation(),
//                    w2l*e.measurement.get_translation()
//                    ,Point3ub(255,0,0));

    opt->optimize(graph);

//    Rate::sleep(2);
    for(int i=0,iend=KeyFrameEles.size();i<iend;i++)
    {
        if(graph.keyframes[i].dof==UPDATE_KF_NONE) continue;
        GSLAM::FramePtr& frame_ele=KeyFrameEles[i];
        GSLAM::SIM3 sim3=graph.keyframes[i].estimation;
        GSLAM::SIM3 update=sim3*frame_ele->getPose().inverse();
        std::map<GSLAM::PointID,size_t> observations;
        if(!frame_ele->getObservations(observations)) continue;
        for(auto it:observations)
        {
            GSLAM::PointPtr pt=_map->getPoint(it.first);
            if(!pt) continue;
            if(pt->refKeyframeID()!=frame_ele->id()) continue;
//            ply.addPoint(w2l*pt->getPose(),Point3ub(255,0,0));
//            ply.addPoint(w2l*(update*pt->getPose()),Point3ub(0,255,0));
            pt->setPose(update*pt->getPose());
        }
        frame_ele->setPose(sim3.get_se3());
    }
//    for(KeyFrameEstimzation kf:graph.keyframes)
//        ply.addPoint(w2l*kf.estimation.get_translation(),Point3ub(0,255,0));
//    Rate::sleep(2);
}

bool Mapper::eraseMapFrame(const FramePtr& fr)
{
    std::map<GSLAM::PointID,size_t> observes=fr->getObservations();
    bool erasedAll=true;
    for(auto& obs:observes)
    {
        auto mpt=_map->getPoint(obs.first);
        if(!mpt) {erasedAll=false;continue;}
        if(!fr->eraseObservation(mpt,true)) {erasedAll=false;continue;}
    }
    auto parents=fr->getParents();
    auto children=fr->getChildren();
    for(auto parent:parents){
        FramePtr p=_map->getFrame(parent.first);
        if(!p){erasedAll=false;continue;}
        p->eraseChild(fr->id());
    }
    for(auto child:children){
        FramePtr c=_map->getFrame(child.first);
        if(!c) {erasedAll=false;continue;}
        c->eraseParent(fr->id());
    }
    //    assert(erasedAll);
    if(erasedAll)
    {
        _map->eraseMapFrame(fr->id());
        return true;
    }
    else
    {
        _map->eraseMapFrame(fr->id());
        DLOG(ERROR)<<"Failed to erase mapframe.";
        return false;
    }
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
        if(!fr->eraseObservation(mpt,true)) {erasedAll=false;continue;}
    }
    //    assert(erasedAll);
    if(erasedAll)
    {
        _map->eraseMapPoint(mpt->id());
        return true;
    }
    else
    {
        DLOG(ERROR)<<"Failed to erase mappoint.";
        return false;
    }
}

bool Mapper::fuseMapPoint(const GSLAM::PointPtr& from,const GSLAM::PointPtr& to)
{
    std::map<GSLAM::FrameID,size_t> observes,toObserves;
    from->getObservations(observes);
    to->getObservations(toObserves);
    // add to the new mappoint
    bool fusedAll=true;
    for(std::pair<GSLAM::FrameID,size_t> obs:observes)
    {
        if(toObserves.count(obs.first)) continue;
        GSLAM::FramePtr fr=_map->getFrame(obs.first);
        if(!fr.get()) {fusedAll=false;continue;}
        std::map<GSLAM::FrameID,size_t> observesCopy=observes;
        from->getObservations(observesCopy);

        if(!fr->eraseObservation(from,true)) {fusedAll=false;continue;}
        if(!fr->addObservation(to,obs.second,true)) {fusedAll=false;continue;}
    }

    if(!_map->eraseMapPoint(from->id()))
    {
        return false;
    }

    //    assert(fusedAll);
    if(fusedAll) return true;
    else
    {
        DLOG(ERROR)<<"Not all observations are fused.";
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
        if(v(3,3)==0) return false;
        Eigen::Vector4d x3D;
        x3D<<v(0,3),v(1,3),v(2,3),v(3,3);


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

void  Mapper::updateNormAndDes()
{
    if(!svar.GetInt("Mapper.UpdateNormalDescriptor")) return;
    GSLAM::ScopedTimer tm("Mapper::updateNormAndDes");
    for(std::pair<PointID,PointPtr> it:_updatedPts){
        PointPtr& pt=it.second;
        Point3d   position=pt->getPose();
        std::map<GSLAM::FrameID,size_t> observations;
        if(!pt->getObservations(observations)) continue;
        std::vector<GSLAM::GImage> descriptors;
        GSLAM::Point3d             norm(0,0,0);
        descriptors.reserve(observations.size());

        for(std::pair<GSLAM::FrameID,size_t> obs:observations)
        {
            GSLAM::FramePtr fr=_map->getFrame(obs.first);
            if(!fr) continue;
            descriptors.push_back(fr->getDescriptor(obs.second));
            norm=norm+(fr->getPose().get_translation()-position).normalize();
        }
        if(descriptors.empty()) continue;
        norm=norm.normalize();
        GSLAM::GImage mean;
        GSLAM::Vocabulary::meanValue(descriptors,mean);
        pt->setNormal(norm);
        pt->setDescriptor(mean);
    }
}

bool  Mapper::closeLoop()
{
    if(!_loopCloser) return false;
    if(!svar.GetInt("Mapper.CloseLoop")) return false;
    return _loopCloser->insertKeyFrame(_curFrame);
}

}


typedef demo::Mapper MapperDemo;
REGISTER_MAPPER(MapperDemo,demo);
