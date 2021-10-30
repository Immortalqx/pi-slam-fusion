
#ifdef HAS_OPENCV
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#endif

#include "Tracker.h"

#include <set>
#include <sstream>

#include <GSLAM/core/Timer.h>
#include <GSLAM/core/Svar.h>
#include <GSLAM/core/Event.h>
#include <GSLAM/core/Estimator.h>
#include <GSLAM/core/VecParament.h>

#include "MapFrame.h"
#include "MapPoint.h"
#include "Mapper.h"
#include "OpenGL.h"
#include "Initializer.h"
#include "Relocalizer.h"
#include "Matcher.h"
#include "LoopDetector.h"
#include "LoopCloser.h"
#include "Estimator.h"
#include "optimizerG2O/Optimizer.h"


namespace demo
{

using std::vector;
using std::pair;
using GSLAM::SE3;
using GSLAM::Point3d;
using GSLAM::Point2d;
using GSLAM::Point2f;
using GSLAM::Camera;
using GSLAM::PointPtr;
using GSLAM::FramePtr;
using GSLAM::PointID;
using GSLAM::FrameID;

enum TrackerStatus
{
    StatusInitializing,
    StatusTracking,
    StatusLosted
};

class Tracker : public ::Tracker
{
public:
    Tracker(const GSLAM::MapPtr& map,const SPtr<Mapper>& mapper);
    bool track(const SPtr<MapFrame>& frame);

    void draw();
private:
    bool initialize();
    bool trackLastFrame();
    bool trackRefKeyframe(FramePtr ref);
    bool trackLocalMap();
    bool relocalize();
    bool addKeyframeIfNeeded();
    bool fitGPS(vector<SPtr<MapFrame> > frames,GSLAM::SIM3& local2ecef);

    class ScopedLogger
    {
    public:
        ScopedLogger(std::stringstream& sst):_sst(sst),_verbose(svar.GetInt("SLAM.Verbose")){
            _sst.str("");
        }
        ~ScopedLogger(){
            if(_sst.str().size()&&(_verbose&0x01))
                LOG(INFO)<<_sst.str();
        }

        std::stringstream& _sst;
        int                _verbose;
    };
    SPtr<GSLAM::Estimator>  _estimator;
    SPtr<Initializer>       _initializer;
    SPtr<Relocalizer>       _relocalizer;
    SPtr<Matcher>           _matcher;

    GSLAM::MutexRW          _mutex;
    SPtr<MapFrame>          _lastKF,_lastFrame,_curFrame;
    int                     _status;
    std::stringstream       _logger;
};

class TrackFrameConnection : public GSLAM::FrameConnection
{
public:
    TrackFrameConnection(const vector<pair<int,int> >& matches)
        :_matches(matches){}

    virtual std::string type()const{return "TrackFrameConnection";}
    virtual int  matchesNum(){return 0;}

    virtual bool getMatches(std::vector<std::pair<int,int> >& matches){matches=_matches;return true;}

    vector<pair<int,int> > _matches;
};


Tracker::Tracker(const GSLAM::MapPtr& map,const SPtr<Mapper>& mapper)
    : ::Tracker(map,mapper),_status(StatusInitializing)
{
    _estimator=GSLAM::Estimator::create(svar.GetString("Estimator",""));
    _initializer=Initializer::create(svar.GetString("Initializer","svd"));
    _matcher=Matcher::create(svar.GetString("Matcher","flann"));
    _relocalizer=Relocalizer::create(svar.GetString("Relocalizer","demo"));

    if(!_estimator)
    {
        _estimator=createEstimatorInstance();
        if(!_estimator)
            LOG(FATAL)<<"No valid Estimator!";
    }
    if(!_initializer)
    {
        LOG(FATAL)<<"No valid Initializer!";
    }
    if(!_matcher){
        LOG(FATAL)<<"No valid Matcher!";
    }
}

bool Tracker::track(const SPtr<MapFrame>& frame)
{
    ScopedLogger logger(_logger);
    _curFrame=frame;
    _logger<<"Frame "<<frame->id()
          <<","<<frame->keyPointNum()<<" kpts";
    if(_status==StatusInitializing) return initialize();
    if(_status==StatusLosted) return relocalize();

    bool bOK=true;
    bOK=trackLastFrame();
    if(!bOK) bOK=trackRefKeyframe(_lastKF);
    if(!bOK) {
        if(svar.GetInt("SLAM.RestartWhenLost",1))_status=StatusInitializing;
        else _status=StatusLosted;
        LOG(ERROR)<<"Losted at frame "<<_curFrame->id();
        return false;
    }

    bOK=trackLocalMap();

//    if(!bOK) {
//        _status=StatusLosted;
//        LOG(ERROR)<<"Losted at frame "<<_curFrame->id();
//        return false;
//    }

    if(addKeyframeIfNeeded()) {
        _logger<<",KF";
        _lastKF=_curFrame;
    }

    {
        GSLAM::WriteMutex mutex(_mutex);
        _lastFrame=_curFrame;
    }
    if(_handle) _handle->handle(new GSLAM::CurrentFrameEvent(_curFrame));

    return bOK;
}

bool Tracker::fitGPS(vector<SPtr<MapFrame> > frames,GSLAM::SIM3& local2ecef)
{
    if(frames.size()!=2) return false;
    SPtr<MapFrame> fr1=frames.front();
    SPtr<MapFrame> fr2=frames[1];

    while(1)// fit GPS with Optimizer
    {
        // now if gps information is available, use it to fitting
        GSLAM::SIM3 prioryFirst,priorySecond;
        if(!fr1->getPrioryPose(prioryFirst)) break;
        if(!fr2->getPrioryPose(priorySecond)) break;

        GSLAM::SE3 child2parent=fr1->getPose().inverse()*fr2->getPose();
        // estimate initial status
        double distanceGPS=(priorySecond.get_translation()-prioryFirst.get_translation()).norm();
        double distanceEst=child2parent.get_translation().norm();
        double scale=distanceGPS/distanceEst;

        local2ecef.get_se3()=priorySecond.get_se3()*fr2->getPose().inverse();
        local2ecef.get_scale()=scale;

        fr1->setPose(local2ecef.get_se3()*fr1->getPose());
        fr2->setPose(local2ecef.get_se3()*fr2->getPose());
        return true;
    }

    // fit with SIM3 Estimator

    if(!_estimator) return false;
    // need at least 3 point

//    std::vector<Point3d> from={fr1->getPose().get_translation(),fr2->getPose().get_translation(),};
//    std::vector<Point3d> to={fr1->get};

    return false;

}

bool Tracker::initialize()
{
    if(!_lastKF) { _lastKF=_curFrame; return true;}

    GSLAM::ScopedTimer mtimer("Tracker::initialize");
    vector<pair<int,int> > matches;
    _logger<<",Init with "<<_lastKF->id();
    if(!_matcher->match4initialize(_lastKF,_curFrame,matches)||matches.size()<std::max(100,_lastKF->keyPointNum()/10))
    {
        _logger<<",Match"<<matches.size();
        _lastKF=_curFrame;return false;
    }
    _logger<<",Match"<<matches.size();

    vector<pair<Point2f,Point2f> > corrs;
    vector<int>                    c2midx;
    corrs.reserve(matches.size());
    c2midx.reserve(matches.size());

    for(int i=0;i<matches.size();i++){
        pair<int,int>& m=matches[i];
        c2midx.push_back(i);
        Point3d& p1=_lastKF->keyPoint(m.first)._ptUn;
        Point3d& p2=_curFrame->keyPoint(m.second)._ptUn;
        corrs.push_back(std::make_pair(Point2f(p1.x,p1.y),Point2f(p2.x,p2.y)));
    }

    SE3 cur2lastkf;
    vector<pair<int,Point3d> > mpts;
    if(!_initializer->initialize(corrs,Camera(),cur2lastkf,mpts)) {
        _logger<<",Failed";
        return false;
    }

    // GPS fit
    _lastKF->setPose(SE3());
    _curFrame->setPose(cur2lastkf);
    if(svar.GetInt("GPS.EnableFitGPS",1))
    {
        GSLAM::SIM3 local2ecef;
        if(fitGPS({_lastKF,_curFrame},local2ecef)) {

            for(pair<int,Point3d>& pt:mpts)
            {
                pt.second=local2ecef*pt.second;
            }
            svar.GetInt("GPS.Fitted",0)=1;
        }
        else svar.GetInt("GPS.Fitted")=0;
    }

    // Create Map
    _map->clear();
    _map->insertMapFrame(_lastKF);
    _map->insertMapFrame(_curFrame);

    Point3d t=_curFrame->getPose().get_translation();
    for(pair<int,Point3d>& pt:mpts)
    {
        int mi=c2midx[pt.first];
        pair<int,int> m=matches[mi];
        Point3d nVec=(t-pt.second).normalize();
        GSLAM::Point3ub color;
        _curFrame->getKeyPointColor(m.second,color);

        GSLAM::PointPtr mpt(new MapPoint(_map->getPid(),pt.second,nVec,color,_curFrame->id()));
        mpt->setDescriptor(_curFrame->getDescriptor(m.second).clone());

        _lastKF->addObservation(mpt,m.first,true);
        _curFrame->addObservation(mpt,m.second,true);
        _map->insertMapPoint(mpt);
    }
    LOG(INFO)<<"Created map between frame "<<_lastKF->id()<<" and "<<_curFrame->id()
            <<", with "<<mpts.size()<<" mapoints.";
    _status=StatusTracking;

    _mapper->insertKeyFrame(_curFrame);
    if(_handle)
    {
        _handle->handle(_lastKF);
        _handle->handle(_map);
    }

    {
        GSLAM::WriteMutex mutex(_mutex);
        _lastFrame=_curFrame;
        _lastKF=_curFrame;
    }


    return true;
}

bool Tracker::trackLastFrame()
{
    if(svar.GetInt("DisableTrackLastFrame",0)) return false;
    if(_curFrame->timestamp()-_lastFrame->timestamp()>1.) return false;
    GSLAM::ScopedTimer mtimer("Tracker::trackLastFrame");

    std::map<GSLAM::PointID,size_t> last_observations;
    if(!_lastFrame->getObservations(last_observations)) return false;

    int minMatchesNumber=std::max(30,int(last_observations.size()*0.1));
    if(last_observations.size()<minMatchesNumber) return false;

    float radius=0.05*_curFrame->getCamera().width();
    vector<pair<int,int> >       matches;
    vector<pair<int,PointPtr> >  observations;
    vector<bool>                 tracked(_curFrame->keyPointNum(),false);
    matches.reserve(_lastFrame->observationNum());
    observations.reserve(_lastFrame->observationNum());
    for(std::pair<GSLAM::PointID,size_t> obs : last_observations)
    {
        GSLAM::Point2f pt;
        if(!_lastFrame->getKeyPoint(obs.second,pt)) continue;
        int idx;
        if(!_matcher->findMatchWindow(_lastFrame->getDescriptor(obs.second),
                                      _curFrame,pt.x,pt.y,radius,idx)) continue;
        if(tracked[idx]) continue;
        matches.push_back(pair<int,int>(obs.second,idx));
        observations.push_back(pair<int,PointPtr>(idx,_map->getPoint(obs.first)));
        tracked[idx]=true;
    }

    _logger<<",trackLastFrame,Match"<<matches.size();
    if(matches.size()<minMatchesNumber)
    {
#ifdef HAS_OPENCV
    if(svar.GetInt("DebugTrackLastFrame",0))
    {
        cv::Mat img=_curFrame->getImage();
        for(auto m:matches)
        {
            GSLAM::Point2f pt1,pt2;
            _lastFrame->getKeyPoint(m.first,pt1);
            _curFrame->getKeyPoint(m.second,pt2);
            cv::line(img,cv::Point2f(pt1.x,pt1.y),cv::Point2f(pt2.x,pt2.y),cv::Scalar(255,0,0),2);
        }

        //cv::imwrite("trackLastFrame_"+std::to_string(_lastFrame->id())+"-"+std::to_string(_curFrame->id())+".jpg",img);
    }
#endif
    }
    _logger<<",Obs"<<observations.size();

    if( observations.size() < minMatchesNumber )
    {
        DLOG(ERROR)<<"Observation number "<<observations.size()<<" < "<<minMatchesNumber;
        return false;
    }
    minMatchesNumber=std::max(30,int(observations.size()*0.3));

    GSLAM::SE3      local2world=_lastFrame->getPose();
    GSLAM::SE3      world2local=local2world.inverse();
    vector<Point3d> objectPoints;
    vector<Point2d> imagePoints;
    objectPoints.reserve(observations.size());
    imagePoints.reserve(observations.size());

    for(pair<int,GSLAM::PointPtr>& obs : observations)
    {
        if(!obs.second) continue;
        objectPoints.push_back(world2local*obs.second->getPose());
        imagePoints.push_back(*(Point2d*)&_curFrame->keyPoint(obs.first)._ptUn);
    }

    SE3             local2cur;
    vector<int>     inliers;

    SPtr<GSLAM::Optimizer> optimizer=createOptimizerInstance();
//    optimizer->optimizePnP();
    bool bOK=_estimator->findPnPRansac(local2cur,objectPoints,imagePoints,
                                       Camera(),true,100,0.01,minMatchesNumber,&inliers);

    if(false)
    {
        inliers.clear();
        inliers.reserve(objectPoints.size());
        vector<double> errors;errors.reserve(objectPoints.size());
        for(int i=0;i<objectPoints.size();i++)
        {
            Point3d pt=local2cur*objectPoints[i];
            pt=pt/pt.z;
            Point2d error=imagePoints[i]-Point2d(pt.x,pt.y);
            errors.push_back(error.norm());
        }
        std::sort(errors.begin(),errors.end());
        double threshold=errors[errors.size()/2]*3;

        for(int i=0;i<errors.size();i++)
        if(errors[i]<threshold) inliers.push_back(i);
    }
    _logger<<",Inliers"<<inliers.size();
#ifdef HAS_OPENCV
    if(svar.GetInt("DebugTrackLastFrame",0))
    {
        cv::Mat img=_curFrame->getImage();
        for(auto m:matches)
        {
            GSLAM::Point2f pt1,pt2;
            _lastFrame->getKeyPoint(m.first,pt1);
            _curFrame->getKeyPoint(m.second,pt2);
            cv::line(img,cv::Point2f(pt1.x,pt1.y),cv::Point2f(pt2.x,pt2.y),cv::Scalar(255,0,0),2);
        }
        for(int idx:inliers)
        {
            auto m=matches[idx];
            GSLAM::Point2f pt1,pt2;
            _lastFrame->getKeyPoint(m.first,pt1);
            _curFrame->getKeyPoint(m.second,pt2);
            cv::line(img,cv::Point2f(pt1.x,pt1.y),cv::Point2f(pt2.x,pt2.y),cv::Scalar(0,255,0),2);
        }

        //cv::imwrite("trackLastFrame_"+std::to_string(_lastFrame->id())+"-"+std::to_string(_curFrame->id())+".jpg",img);
    }
#endif
    if(!bOK)
    {
        DLOG(ERROR)<<"PnPRansac failed.";
        return false;
    }

    if(inliers.size()<minMatchesNumber)
    {
        DLOG(ERROR)<<"Inliers too few.";
        return false;
    }

    for(int idx:inliers)
    {
        pair<int,GSLAM::PointPtr>& obs=observations[idx];
        if(!obs.second) continue;
        if(_curFrame->getKeyPointObserve(obs.first)) continue;
        _curFrame->addObservation(obs.second,obs.first);
    }

    _curFrame->setPose(local2world*local2cur.inverse());
    return true;
}

bool Tracker::trackRefKeyframe(FramePtr ref)
{
    GSLAM::ScopedTimer mTimer("Tracker::trackRefKeyframe");

    SPtr<MapFrame> refFrame;
    if(ref) refFrame=std::dynamic_pointer_cast<MapFrame>(ref);
    else refFrame=_lastKF;
    if(!refFrame) return false;
    vector<pair<int,int> > matches;
    bool bOK=_matcher->match4initialize(refFrame,_curFrame,matches);
    _logger<<",trackrefFrame"<<refFrame->id()<<",Match"<<matches.size();
    if(!bOK)
        return false;

    vector<pair<int,GSLAM::PointPtr> > observations;
    observations.reserve(matches.size());
    for(pair<int,int>& m : matches)
    {
        GSLAM::PointID pid=refFrame->getKeyPointObserve(m.first);
        if(!pid) continue;
        observations.push_back(std::make_pair(m.second,_map->getPoint(pid)));
    }
    _logger<<",Obs"<<observations.size();

    if( observations.size() < 30 )
    {
        DLOG(ERROR)<<"Observation number "<<observations.size()<<" < "<<30;
        return false;
    }

    GSLAM::SE3      local2world=ref->getPose();
    GSLAM::SE3      world2local=local2world.inverse();
    vector<Point3d> objectPoints;
    vector<Point2d> imagePoints;
    objectPoints.reserve(observations.size());
    imagePoints.reserve(observations.size());

    for(pair<int,GSLAM::PointPtr>& obs : observations)
    {
        if(!obs.second) continue;
        objectPoints.push_back(world2local*obs.second->getPose());
        imagePoints.push_back(*(Point2d*)&_curFrame->keyPoint(obs.first)._ptUn);
    }

    SE3             local2cur;
    vector<int>     inliers;
    if( imagePoints.size() < 30 )
    {
        DLOG(ERROR)<<"Observation number "<<imagePoints.size()<<" < "<<30;
        return false;
    }
    bOK=_estimator->findPnPRansac(local2cur,objectPoints,imagePoints,Camera(),
                                  false,100,0.01,std::max(30,int(imagePoints.size()/3)),&inliers);
    _logger<<",Inliers"<<inliers.size();
    if(!bOK)
    {
        DLOG(ERROR)<<"PnPRansac failed.";
        return false;
    }

    if(inliers.size()<std::max(30,int(imagePoints.size()/3)))
    {
        DLOG(ERROR)<<"Inliers too few.";
        return false;
    }

    for(int idx:inliers)
    {
        pair<int,GSLAM::PointPtr>& obs=observations[idx];
        if(!obs.second) continue;
        if(_curFrame->getKeyPointObserve(obs.first)) continue;
        _curFrame->addObservation(obs.second,obs.first);
    }

    _curFrame->setPose(local2world*local2cur.inverse());
    SPtr<GSLAM::FrameConnection> frCon(new TrackFrameConnection(matches));
    _curFrame->addParent(ref->id(),frCon);
    return true;
}

bool Tracker::trackLocalMap()
{
    if(!svar.GetInt("EnableTrackSubMap",1))
        return true;
    GSLAM::ScopedTimer mtimer("Tracker::trackLocalMap");

    // lollect submappoints
    std::map<GSLAM::PointID,size_t> observations;
    _curFrame->getObservations(observations);

    std::set<GSLAM::FrameID> connects;
    std::map<GSLAM::FrameID,SPtr<GSLAM::FrameConnection> > parents;

    bool trackPParents=svar.GetInt("SLAM.TrackPParents",1);
    int  maxLocalKFNum=svar.GetInt("SLAM.MaxLocalKFNum",40);
    if(_lastKF->getParents(parents))
    {
        for(auto parent:parents)
        {
            GSLAM::FramePtr fr=_map->getFrame(parent.first);
            if(!fr.get()) continue;
            if(connects.size()>maxLocalKFNum) break;
            connects.insert(parent.first);
            if(!trackPParents) continue;

            std::map<GSLAM::FrameID,SPtr<GSLAM::FrameConnection> > pparents;
            if(!fr->getParents(pparents)) continue;

            for(auto pp:pparents)
            {
                if(connects.size()>maxLocalKFNum) break;
                connects.insert(pp.first);
            }
        }
    }
    else
    {
        for(auto& obs:observations)
        {
            PointPtr mpt=_map->getPoint(obs.first);
            if(!mpt) continue;
            std::map<GSLAM::FrameID,size_t> ptObses;
            mpt->getObservations(ptObses);
        }
    }

    if(connects.empty()) return false;
//    timer.enter("Tracker::trackSubMapByProjection::1.CM");
    std::set<GSLAM::PointPtr> mappoints;
    for(auto& con:connects)
    {
        GSLAM::FramePtr fr=_map->getFrame(con);
        if(!fr.get()) continue;
        std::map<GSLAM::PointID,size_t> observations1;
        if(!fr->getObservations(observations1)) continue;
        for(auto& obs:observations1)
        {
            GSLAM::PointPtr pt=((MapFrame*)fr.get())->keyPoint(obs.second)._mapPoint;
            if(!pt.get()) continue;
            if(observations.count(pt->id())) continue;
            mappoints.insert(pt);
        }
    }

    _logger<<",LocalPts:"<<mappoints.size();

    // match submappoints
    GSLAM::SE3 camera2world=_curFrame->getPose();
    GSLAM::SE3 world2camera=camera2world.inverse();
    GSLAM::Point3Type t=camera2world.get_translation();
    GSLAM::Camera cam=_curFrame->getCamera(1);
    vector<std::pair<int,GSLAM::PointPtr> > matches;
    int tryCount=0;
    for(auto& pt:mappoints)
    {
        pi::Point3d pw=pt->getPose();
        pi::Point3d pc=world2camera*pw;
        if(pc.z<=0) continue;
        pi::Point3d nvec=world2camera.get_rotation()*pt->getNormal();
        if(nvec.z>-0.7) continue;
        float searchRadius=nvec.z<-0.9?cam.width()*0.004:cam.width()*0.007;
        pi::Point2d pimg=cam.Project(pc);
        if(pimg.x<=0||pimg.y<=0||pimg.x>=cam.width()||pimg.y>=cam.height()) continue;
        int idx;
        tryCount++;
        if(_matcher->findMatchWindow(pt->getDescriptor(),
                           _curFrame,pimg.x,pimg.y,searchRadius,idx))
            matches.push_back(make_pair(idx,pt));
    }

    int matchNumber=_curFrame->observationNum()+matches.size();
    _logger<<",Try:"<<tryCount<<"-Matched"<<matches.size();
//    if(tryCount<200) svar.GetInt("EmmergencyKF")=1;

//    timer.leave("Tracker::trackSubMapByProjection::1.CM");
    if(observations.size()+matches.size()<50) return false;


    GSLAM::SIM3 l2w=_curFrame->getPoseScale();
    GSLAM::SIM3 w2l=l2w.inv();
    {
        vector<Point3d> mpts;
        vector<Point2d> kpts;
        vector<GSLAM::PointPtr> pts;
        vector<int>         featIds;
        mpts.reserve(matchNumber);
        kpts.reserve(matchNumber);
        pts.reserve(matchNumber);
        featIds.reserve(matchNumber);

        for(auto& obs:observations)
        {
            GSLAM::PointPtr pt=_curFrame->keyPoint(obs.second)._mapPoint;
            assert(pt.get());
            GSLAM::KeyPoint    kp=_curFrame->keyPoint(obs.second)._pt;
            pi::Point3d     p3d=w2l*pt->getPose();
            mpts.push_back(p3d);
            pi::Point3d     pc=cam.UnProject(kp.pt.x,kp.pt.y);
            kpts.push_back(Point2d(pc.x,pc.y));
            pts.push_back(pt);
            featIds.push_back(obs.second);
        }

        for(auto& mt:matches)
        {
            pi::Point3d     p3d=w2l*mt.second->getPose();
            GSLAM::KeyPoint    kp=_curFrame->keyPoint(mt.first)._pt;
            pi::Point3d     pc=cam.UnProject(kp.pt.x,kp.pt.y);
            mpts.push_back(p3d);
            kpts.push_back(Point2d(pc.x,pc.y));
            pts.push_back(mt.second);
            featIds.push_back(mt.first);
        }

        world2camera=GSLAM::SE3();
        vector<int>         inliers;

        int minInlierNum=std::max((int)(mpts.size()*0.5),50);
        if(mpts.size()<minInlierNum) return false;

//        timer.enter("Tracker::trackSubMapByProjection::2.PNP");
        SE3 local2camera;
        _estimator->findPnPRansac(local2camera,mpts,kpts,Camera(),
                                  true,std::max(100,int(mpts.size()/3)),
                                  0.01,std::max(30,int(mpts.size()/2)),&inliers);
//        timer.leave("Tracker::trackSubMapByProjection::2.PNP");

        _logger<<"-"<<inliers.size();

        if(inliers.size()<minInlierNum) return false;

        vector<bool> tracked(mpts.size(),false);
        for(auto idx:inliers) tracked[idx]=true;
        for(int i=0;i<tracked.size();i++)
        {
            SPtr<GSLAM::MapPoint>& mpt=_curFrame->keyPoint(featIds[i])._mapPoint;
            if(tracked[i]&&!mpt.get()) _curFrame->addObservation(pts[i],featIds[i]);
            else if(mpt.get()&&!tracked[i]) _curFrame->eraseObservation(mpt);
        }

        _curFrame->setPose(l2w*local2camera.inverse());

        return true;
    }


    return true;
}

bool Tracker::relocalize()
{
//    if(!_relocalizer)
//        return false;

//    bool bOK=_relocalizer->relocalize(_map,_curFrame);

    GSLAM::LoopCandidates candidates;
    if(!_map->obtainCandidates(_curFrame,candidates)) return false;

    bool bOK=false;
    for(int i=0;i<1&&i<candidates.size();i++)
    {
        GSLAM::LoopCandidate can=candidates[i];
        FramePtr ref=_map->getFrame(can.frameId);
        if(!ref) continue;
        if(!trackRefKeyframe(ref)) continue;
        bOK=true;
        break;
    }

    if(!bOK) return false;
    bOK=trackLocalMap();

//    if(!bOK) {
//        _status=StatusLosted;
//        LOG(ERROR)<<"Losted at frame "<<_curFrame->id();
//        return false;
//    }

    if(addKeyframeIfNeeded()) {_lastKF=_curFrame;}

    {
        GSLAM::WriteMutex mutex(_mutex);
        _lastFrame=_curFrame;
    }
    if(_handle) _handle->handle(new GSLAM::CurrentFrameEvent(_curFrame));

    return bOK;
}

bool Tracker::addKeyframeIfNeeded()
{
    SE3 cur2kf=_lastKF->getPose().inverse()*_curFrame->getPose();
    pi::Array_<double,6> ln=cur2kf.ln();
    double mIDepth=1./_lastKF->getMedianDepth();
    for(int i=0;i<3;i++) ln.data[i]*=mIDepth;

    double weight[]={1.,1.,1.,1,1,0.01};
    double sum=0;
    for(int i=0;i<6;i++) sum+=weight[i]*ln.data[i]*ln.data[i];
    sum=sqrt(sum);

    _logger<<",FOV:"<<sum;
    if(sum>=1-svar.GetDouble("SLAM.MaxOverlap",0.95)||svar.GetInt("EmmergencyKF"))
    {
        svar.GetInt("EmmergencyKF")=0;
        return _mapper->insertKeyFrame(_curFrame);
    }
    return false;
}

void Tracker::draw()
{
    if(!_lastFrame) return;
    FramePtr fr;
    {
        GSLAM::ReadMutex mutex(_mutex);
        fr=_lastFrame;
    }
    double     depth=fr->getMedianDepth()/10;
    GSLAM::SE3 pose=fr->getPose();
    GSLAM::Point3Type t=pose.get_translation();
    double r[9];
    pose.get_rotation().getMatrixUnsafe(r);
    auto camera=fr->getCamera(0);

    // Draw camera rect
    {
        pi::Point3d tl=camera.UnProject(pi::Point2d(0,0));
        pi::Point3d tr=camera.UnProject(pi::Point2d(camera.width(),0));
        pi::Point3d bl=camera.UnProject(pi::Point2d(0,camera.height()));
        pi::Point3d br=camera.UnProject(pi::Point2d(camera.width(),camera.height()));
        //pi::Point2d ct=cam_out->UnProject(pi::Point2d(cam_out->Cx(),cam_out->Cy()));

        GSLAM::Point3Type  W_tl=pose*(pi::Point3d(tl.x,tl.y,1)*depth);
        GSLAM::Point3Type  W_tr=pose*(pi::Point3d(tr.x,tr.y,1)*depth);
        GSLAM::Point3Type  W_bl=pose*(pi::Point3d(bl.x,bl.y,1)*depth);
        GSLAM::Point3Type  W_br=pose*(pi::Point3d(br.x,br.y,1)*depth);

//        Point3Type  W_ct=pose*(pi::Point3d(ct.x,ct.y,1)*depth);
        glBegin(GL_LINES);
        glLineWidth(2.5);
        glColor3f(1, 0, 0);
        glVertex(t);        glVertex(W_tl);
        glVertex(t);        glVertex(W_tr);
        glVertex(t);        glVertex(W_bl);
        glVertex(t);        glVertex(W_br);
        glVertex(W_tl);     glVertex(W_tr);
        glVertex(W_tr);     glVertex(W_br);
        glVertex(W_br);     glVertex(W_bl);
        glVertex(W_bl);     glVertex(W_tl);
        glEnd();
    }

    if(svar.GetInt("FrameTracker.DrawCurrentConnects",1))
    {
        std::map<GSLAM::FrameID,SPtr<GSLAM::FrameConnection> > parents;
        if(!fr->getParents(parents)) return;

        glBegin(GL_LINES);
        glLineWidth(0.5);
        glColor3ub(0,255,255);
        for(auto it:parents)
        {
            FramePtr p=_map->getFrame(it.first);
            if(!p) continue;
            glVertex(t);
            glVertex(p->getPose().get_translation());
        }
        glEnd();
    }
}


}

typedef demo::Tracker TrackerDemo;
REGISTER_TRACKER(TrackerDemo,demo);
