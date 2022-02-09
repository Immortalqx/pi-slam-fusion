
//#define NDEBUG

#ifdef HAS_OPENCV
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
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
#include <Eigen/Core>

#include "../../src/DataTrans.h"

namespace opt
{

using std::vector;
using std::pair;
using std::string;
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

class GPSRectVisualizer : public GSLAM::GObject
{
public:
    GPSRectVisualizer(GSLAM::MapPtr map)
        :_map(map),
         _origin(svar.get_var("Origin",GSLAM::Point3d()))
    {}
    GSLAM::SO3 PYR2Rotation(double pitch,double yaw,double roll)
    {
//        if(fabs(180-fabs(roll))<10) roll+=180;
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

            pyrSigma=Point3d(1,100,1);
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
        double gpsK=svar.GetDouble("GPS.K",1e-2);
        infoM(0,0)=gpsK/(gpsSigma.x*gpsSigma.x);
        infoM(1,1)=gpsK/(gpsSigma.y*gpsSigma.y);
        infoM(2,2)=gpsK/(gpsSigma.z*gpsSigma.z);
        infoM(3,3)=1./(pyrSigma.x*pyrSigma.x);
        infoM(4,4)=1./(pyrSigma.z*pyrSigma.z);
        infoM(5,5)=1./(pyrSigma.y*pyrSigma.y);
        return true;
    }

    virtual void draw(){
        GSLAM::FrameArray frames;
        _map->getFrames(frames);

        for(GSLAM::FramePtr fr:frames){
            double info[36];
            GSLAM::SE3 se3;
            if(getGPSEdge(fr,se3,info))
            {
                se3.get_translation()=se3.get_translation()-_origin;
                if(!_camera.isValid()) _camera=fr->getCamera();
                drawRect(GSLAM::SIM3(se3,fr->getMedianDepth()*0.1),GSLAM::ColorType(255,0,0));
            }
        }
    }

    void drawRect(GSLAM::SIM3 pose,GSLAM::ColorType color)
    {
//        if(!_camera.isValid()) _camera=GSLAM::Camera({640.,480.,500.,500.,320.,240.});
        {
            Point3d t=pose.get_translation();
            pi::Point3d tl=_camera.UnProject(pi::Point2d(0,0));
            pi::Point3d tr=_camera.UnProject(pi::Point2d(_camera.width(),0));
            pi::Point3d bl=_camera.UnProject(pi::Point2d(0,_camera.height()));
            pi::Point3d br=_camera.UnProject(pi::Point2d(_camera.width(),_camera.height()));

            GSLAM::Point3Type  W_tl=pose*(pi::Point3d(tl.x,tl.y,1));
            GSLAM::Point3Type  W_tr=pose*(pi::Point3d(tr.x,tr.y,1));
            GSLAM::Point3Type  W_bl=pose*(pi::Point3d(bl.x,bl.y,1));
            GSLAM::Point3Type  W_br=pose*(pi::Point3d(br.x,br.y,1));

            glBegin(GL_LINES);
            glLineWidth(2.5);
            glColor3ub(color.x,color.y,color.z);
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
    }
    GSLAM::MapPtr _map;
    GSLAM::Camera _camera;
    GSLAM::Point3d _origin;
};

class Tracker : public ::Tracker
{
public:
    Tracker(const GSLAM::MapPtr& map,const SPtr<Mapper>& mapper);
    bool track(const SPtr<MapFrame>& frame);

    void draw();

    virtual void call(const std::string &command, void *arg=NULL);

private:
    bool initialize();
    bool trackLastFrame();
    bool trackRefKeyframe(FramePtr ref);
    bool trackRefKeyframeRansac(FramePtr ref);
    bool trackLocalMap();
    bool relocalize();
    bool addKeyframeIfNeeded();
    bool addKeyframeIfNeeded1();
    bool reOptimizePose(SPtr<MapFrame>& frame, GSLAM::SE3& pose);

    bool fitGPS(vector<SPtr<MapFrame> > frames,GSLAM::SIM3& local2ecef);
    bool optimizePnPEpipolar(const std::vector<std::pair<Point3d,Point3d> >& matches,
                             GSLAM::SE3& pose);
    bool estimateRelative(const std::vector<std::pair<Point3d,Point3d> >& matches,
                          GSLAM::SE3& pose);

    void reset(){
        _map->clear();
        _lastKF.reset();
        _lastFrame.reset();
        _status=StatusInitializing;
    }

    class ScopedLogger
    {
    public:
        ScopedLogger(std::stringstream& sst):_sst(sst),_verbose(svar.GetInt("SLAM.Verbose",2)){
            _sst.str("");
        }
        ~ScopedLogger(){
            if(_sst.str().size()&&(_verbose&0x01))
                LOG(INFO)<<_sst.str();
        }

        std::stringstream& _sst;
        int                _verbose;
    };

    struct Velocity{
        FrameID fid=0;
        SE3     v;
    }_velocity;

    SPtr<GSLAM::Estimator>  _estimator;
    SPtr<Initializer>       _initializer;
    SPtr<Relocalizer>       _relocalizer;
    SPtr<Matcher>           _matcher;

    GSLAM::MutexRW          _mutex;
    SPtr<MapFrame>          _lastKF,_lastFrame,_curFrame;
    int                     _status;
    std::stringstream       _logger;
    double                  _lostedTime;
    bool                    firstTime=true;
};

class TrackFrameConnection : public GSLAM::FrameConnection
{
public:
    TrackFrameConnection(const vector<pair<int,int> >& matches=vector<pair<int,int> >())
        :_matches(matches){}

    virtual std::string type()const{return "TrackFrameConnection";}
    virtual int  matchesNum(){return _matches.size();}

    virtual bool getMatches(std::vector<std::pair<int,int> >& matches){matches=_matches;return true;}

    vector<pair<int,int> > _matches;
};


Tracker::Tracker(const GSLAM::MapPtr& map,const SPtr<Mapper>& mapper)
    : ::Tracker(map,mapper),_status(StatusInitializing)
{
    svar.GetInt("GPS.Fitted",0)=0;
    _estimator=GSLAM::Estimator::create(svar.GetString("Estimator",""));
    _initializer=Initializer::create(svar.GetString("Initializer","svd"));
    if( svar.GetInt("SLAM.isOnline") )
        _matcher=Matcher::create(svar.GetString("Matcher","bow"));
    else
        _matcher=Matcher::create(svar.GetString("Matcher","zy_bfMultiH"));
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
    std::string imgPath;
    frame->call("GetImagePath",&imgPath);
    imgPath=imgPath.substr(imgPath.find_last_of('/')+1);
    if(imgPath.size()) imgPath=","+imgPath;
    _logger.precision(13);
    _logger<<"Frame "<<frame->id()<<imgPath<<","<<frame->timestamp()
          <<","<<frame->keyPointNum()<<" kpts";

    bool bOK=true;
    if(_status==StatusInitializing)
    {
        return initialize();
    }

    if(svar.GetInt("Tracker.TestInit")) return false;

    if(_status==StatusLosted)
    {
        bOK= relocalize();
        if(!bOK&&_map->frameNum()<10)
        {
//            reset();
            return initialize();
        }
        if(!bOK&&frame->timestamp()-_lostedTime>svar.GetDouble("SLAM.LostRestart",10)){
//            reset();
            return initialize();
        }
        return bOK;
    }


    // track current frame
    bOK=trackLastFrame();
    if(!bOK) {
        // track with last KF
        bOK=trackRefKeyframeRansac(_lastKF);
    }
    if(!bOK) {
        bOK=trackRefKeyframe(_lastKF);
    }

    if(bOK)
        trackLocalMap();
    if(!bOK) {
        _status=StatusLosted;
        _lostedTime=frame->timestamp();
        _logger<<",Losted";
        return false;
    }


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

    //TODO 从这里往map2dfusion发送数据？？？
    std::pair<std::string, pi::SE3d> trans_frame(std::to_string(_curFrame->timestamp()), _curFrame->getPose());
    Trans.product(trans_frame);

    return bOK;
}

bool Tracker::fitGPS(vector<SPtr<MapFrame> > frames,GSLAM::SIM3& local2ecef)
{
    if(frames.size()!=2) return false;
    SPtr<MapFrame> fr1=frames.front();
    SPtr<MapFrame> fr2=frames[1];

    while(1)// fit GPS without Optimizer
    {
        _logger<<",SIM3WithPYR:";
        // now if gps information is available, use it to fitting
        GSLAM::SIM3 prioryFirst,priorySecond;
        if(!fr1->getPrioryPose(prioryFirst)) break;
        if(!fr2->getPrioryPose(priorySecond)) break;

        GSLAM::SE3 child2parent=fr1->getPose().inverse()*fr2->getPose();

        // estimate initial status
        double distanceGPS=(priorySecond.get_translation()-prioryFirst.get_translation()).norm();
        Point3d sigma;
        if(fr2->getGPSLLASigma(sigma))
        {
            if(distanceGPS<sigma.norm())
            {
                _logger<<distanceGPS<<"<"<<sigma.norm();
                return false;
            }
        }
        else if(distanceGPS<svar.GetDouble("GPS.MinDistance2Fit",10)){
            _logger<<distanceGPS<<"<"<<svar.GetDouble("GPS.MinDistance2Fit",10);
            return false;
        }
        double distanceEst=child2parent.get_translation().norm();
        double scale=distanceGPS/distanceEst;
        local2ecef.get_se3()=prioryFirst.get_se3()*fr1->getPose().inverse();
        local2ecef.get_scale()=scale;

        auto error=local2ecef*fr2->getPose().get_translation()-priorySecond.get_translation();
        _logger<<error;
        if(error.norm()>1.) break;

        fr1->setPose(local2ecef*fr1->getPose());
        fr2->setPose(local2ecef*fr2->getPose());
        return true;
    }

    // fit with SIM3 Estimator
    _logger<<",SIM3:";

    if(!_estimator)
    {
        LOG(ERROR)<<"No estimator!";
        return false;
    }
    Point3d lla,lla2;
    if(!fr1->getGPSLLA(lla))
    {
        LOG(ERROR)<<"Frame "<<fr1->id()<<" don't have gps information.";
        return false;
    }
    if(!fr2->getGPSLLA(lla2)) {
        LOG(ERROR)<<"Frame "<<fr2->id()<<" don't have gps information.";
        return false;
    }

    // Local to ECEF
    GSLAM::SIM3 local2ECEF;
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

    GSLAM::SE3 child2parent=fr1->getPose().inverse()*fr2->getPose();
    GSLAM::Point3d xyz1=local2ECEF.inv()*GSLAM::GPS<>::GPS2XYZ(lla.y,lla.x,lla.z);
    GSLAM::Point3d xyz2=local2ECEF.inv()*GSLAM::GPS<>::GPS2XYZ(lla2.y,lla2.x,lla2.z);
    // estimate initial status
    double distanceGPS=(xyz2-xyz1).norm();
    double distanceEst=child2parent.get_translation().norm();
    std::vector<Point3d> from={Point3d(0,0,0),
                               child2parent.get_translation(),
                               Point3d(0,0,-1)*distanceEst};
    std::vector<Point3d> to  ={xyz1,xyz2,xyz1+Point3d(0,0,1)*distanceGPS};
    GSLAM::SIM3 fr12ecef;
    if(!_estimator->findSIM3(fr12ecef,from,to))
    {
        _logger<<"Failed to findSIM3.";
        return false;
    }

    double fitError=0;
    for(int i=0;i<from.size();i++)
    {
        auto e=fr12ecef*from[i]-to[i];
        fitError+=e.dot(e);
    }
    fitError=sqrt(fitError/from.size());
    if(fitError>distanceGPS*0.1)
    {
        _logger<<"Error to large.";
        return false;
    }
    else {
        _logger<<fitError;
    }

    local2ecef=local2ECEF*fr12ecef*fr1->getPose().inverse();

    fr1->setPose(local2ecef*fr1->getPose());
    fr2->setPose(local2ecef*fr2->getPose());

    return true;
}

bool Tracker::initialize()
{
    if(!_lastKF) { _lastKF=_curFrame; return true;}

    GSLAM::ScopedTimer mtimer("Tracker::initialize");
    vector<pair<int,int> > matches;
    _logger<<",Init with "<<_lastKF->id() << " - " << _curFrame->id();
    if(!_matcher->match4initialize(_lastKF,_curFrame,matches)||matches.size()<std::max(100,_lastKF->keyPointNum()/10))
    {
        _logger<<", MatchFailed "<<matches.size();
        _lastKF=_curFrame;

        return false;
    }
    _logger<<", Match "<<matches.size();

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
        _logger<<", Not Initialized";
        std::cout << _logger.str() << "\n";

        return false;
    }
    if(mpts.size()<=10) return false;
    // check baseline
    {
        vector<double> depths;
        for(pair<int,Point3d> mpt:mpts)
        {
            depths.push_back(mpt.second.z);
        }
        std::sort(depths.begin(),depths.end());
        double depth=depths[depths.size()/2];
        if(cur2lastkf.get_translation().norm()<depth*0.1)
        {
            _logger<<",SmallBaseline";
            return false;
        }
    }

    // GPS fit
    _lastKF->setPose(SE3());
    _curFrame->setPose(cur2lastkf);
    if(svar.GetInt("GPS.EnableFitGPS",1)&&_curFrame->getGPSNum())
    {
        GSLAM::SIM3 local2ecef;
        if(fitGPS({_lastKF,_curFrame},local2ecef)) {

            for(pair<int,Point3d>& pt:mpts)
            {
                pt.second=local2ecef*pt.second;
            }
            svar.GetInt("GPS.Fitted",0)=1;
        }
        else
        {
            svar.GetInt("GPS.Fitted")=0;
            return false;
        }
    }

    // Create Map
    _mapper.reset();

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

        if(_lastKF->addObservation(mpt,m.first,true)
                &&_curFrame->addObservation(mpt,m.second,true))
            _map->insertMapPoint(mpt);
    }
    LOG(INFO)<<"Created map between frame "<<_lastKF->id()<<" and "<<_curFrame->id()
            <<", with "<<mpts.size()<<" mapoints.";
    _status=StatusTracking;

//    _lastFrame->setImage(GSLAM::GImage());// if not insert to mapper, image should be released to save memory
    _curFrame->addParent(_lastKF->id(),SPtr<GSLAM::FrameConnection>(new TrackFrameConnection(matches)));


    _mapper=Mapper::create(_map,svar.GetString("Mapper","demo"));
    if(!_mapper) LOG(FATAL)<<"No mapper created.";
    if(_handle) _mapper->setObjectHandle(_handle);
    if(_handle) _handle->handle(new GSLAM::DebugImageEvent(_lastKF->getImage(),"FirstKF"));

    if(firstTime){
        firstTime=false;
        if(_handle) _handle->handle(_map);
        SPtr<GPSRectVisualizer> vis(new GPSRectVisualizer(_map));
#if (GSLAM_VERSION_MAJOR<<16|GSLAM_VERSION_MINOR<<8|GSLAM_VERSION_PATCH) >= 0x020402
        _handle->handle(new GSLAM::DrawableEvent(vis,"GPSRect"));
#endif
    }
    _mapper->insertKeyFrame(_lastKF);
    _mapper->insertKeyFrame(_curFrame);

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

    GSLAM::SE3      local2world=_lastFrame->getPose();
    GSLAM::SE3      world2local=local2world.inverse();

    float radius=svar.GetDouble("Tracker.TrackLastFrameWindow",0.05)*_curFrame->getCamera().width();
    vector<pair<int,int> >          matches;
    vector<pair<Point3d,Point3d> >  observations;
    vector<bool>                    tracked(_curFrame->keyPointNum(),false);
    int  pnpCount=0;

    GSLAM::Camera camera=_curFrame->getCamera();
    for(int i=0,iend=_lastFrame->keyPointNum();i<iend;i++){
        // look for match
        PointPtr mpt=_lastFrame->keyPoint(i)._mapPoint;
        if(!mpt) continue;
        Point3d est=world2local*mpt->getPose();
        GSLAM::Point2f pt;
        if(_velocity.fid==_lastFrame->id()){
            pt=camera.Project(_velocity.v*est);
        }
        else if(!_lastFrame->getKeyPoint(i,pt)) continue;
        int idx;
        if(!_matcher->findMatchWindow(_lastFrame->getDescriptor(i),
                                      _curFrame,pt.x,pt.y,radius,idx)) continue;
        if(tracked[idx]) continue;
        matches.push_back(pair<int,int>(i,idx));
        tracked[idx]=true;

        GSLAM::Point3d obs=_curFrame->keyPoint(idx)._ptUn;

        if(mpt)// a map point
        {
            Point3d est=world2local*mpt->getPose();
            if(est.z>0)
            {
                observations.push_back(std::make_pair(est,obs));
                pnpCount++;
                continue;
            }
        }

//        // Epipolar match
//        Point3d est=_lastFrame->keyPoint(i)._ptUn;
//        est.z=-1;
//        observations.push_back(std::make_pair(est,obs));
    }

    _logger<<",trackLastFrame,Match"<<pnpCount<<"/"<<matches.size()<<"/"<<_lastFrame->observationNum();
    int minMatchesNumber=std::max(20,int(_lastFrame->observationNum()*0.1));
    int minPnPNumber    =std::max(10,int(_lastFrame->observationNum()*0.1));

    if(matches.size()<minMatchesNumber||pnpCount<minPnPNumber)
    {
#ifdef HAS_OPENCV
        if(svar.GetInt("DebugTrackLastFrame",0))
        {
            cv::Mat img=_curFrame->getImage().clone();
            for(auto m:matches)
            {
                GSLAM::Point2f pt1,pt2;
                _lastFrame->getKeyPoint(m.first,pt1);
                _curFrame->getKeyPoint(m.second,pt2);
                cv::line(img,cv::Point2f(pt1.x,pt1.y),cv::Point2f(pt2.x,pt2.y),cv::Scalar(255,0,0),2);
            }
            if(_handle)
                _handle->handle(new GSLAM::DebugImageEvent(img,"DebugTrackLastFrame"));

            //cv::imwrite("trackLastFrame_"+std::to_string(_lastFrame->id())+"-"+std::to_string(_curFrame->id())+".jpg",img);
        }
#endif
        return false;
    }

    SE3             local2cur;
    vector<int>     inliers;
    if(_velocity.fid==_lastFrame->id()){
        local2cur=_velocity.v;
    }

    timer.enter("Tracker::trackLastFrame::opt");
    SPtr<GSLAM::Optimizer> optimizer=createOptimizerInstance();
    bool bOK=optimizer->optimizePnP(observations,local2cur);
    timer.leave("Tracker::trackLastFrame::opt");

    if(!bOK)
    {
        LOG(ERROR)<<"PnP failed.";
        return false;
    }

    inliers.reserve(observations.size());

    for(int i=0;i<observations.size();i++)
    {
        if(observations[i].first.z<0) continue;
        Point3d pt=local2cur*observations[i].first;
        pt=pt/pt.z;
        Point3d error=observations[i].second-pt;
        if(error.norm()<0.02) inliers.push_back(i);
    }

    _logger<<",Inliers"<<inliers.size();
#ifdef HAS_OPENCV
    if(svar.GetInt("DebugTrackLastFrame",0))
    {
        cv::Mat img=_curFrame->getImage().clone();
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
        if(_handle)
            _handle->handle(new GSLAM::DebugImageEvent(img,"DebugTrackLastFrame"));

        //cv::imwrite("trackLastFrame_"+std::to_string(_lastFrame->id())+"-"+std::to_string(_curFrame->id())+".jpg",img);
    }
#endif


    if(inliers.size()<std::max(10,int(matches.size()*0.6)))
    {
        LOG(ERROR)<<"Inliers too few.";
        return false;
    }

    for(int idx:inliers)
    {
        int featureId = matches[idx].second;
        if(_curFrame->getKeyPointObserve(featureId)) continue;
        PointPtr  mpt = _lastFrame->keyPoint(matches[idx].first)._mapPoint;
        if(mpt)
            _curFrame->addObservation(mpt,featureId);
    }

    _curFrame->setPose(local2world*local2cur.inverse());

    if(svar.GetInt("Tracker.UseMotionModel",1))
    {
        _velocity.fid=_curFrame->id();
        _velocity.v  =local2cur;
    }
    return true;
}

bool Tracker::trackRefKeyframeRansac(FramePtr ref)
{
    GSLAM::ScopedTimer mTimer("Tracker::trackRefKeyframeRansac");

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
    vector<pair<int,int> > matchesObs;
    observations.reserve(matches.size());
    matchesObs.reserve(matches.size());
    for(pair<int,int>& m : matches)
    {
        GSLAM::PointID pid=refFrame->getKeyPointObserve(m.first);
        if(!pid) continue;
        observations.push_back(std::make_pair(m.second,refFrame->keyPoint(m.first)._mapPoint));
        matchesObs.push_back(m);
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
    _logger<<",PnP"<<imagePoints.size();

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
#ifdef HAS_OPENCV
    if(svar.GetInt("DebugTrackRefFrameRansac",0))
    {
        cv::Mat img=_curFrame->getImage().clone();
        for(auto m:matches)
        {
            GSLAM::Point2f pt1,pt2;
            _lastFrame->getKeyPoint(m.first,pt1);
            _curFrame->getKeyPoint(m.second,pt2);
            cv::line(img,cv::Point2f(pt1.x,pt1.y),cv::Point2f(pt2.x,pt2.y),cv::Scalar(0,0,255),img.cols/500);
        }
        for(int idx:inliers)
        {
            auto m=matchesObs[idx];
            GSLAM::Point2f pt1,pt2;
            _lastFrame->getKeyPoint(m.first,pt1);
            _curFrame->getKeyPoint(m.second,pt2);
            cv::line(img,cv::Point2f(pt1.x,pt1.y),cv::Point2f(pt2.x,pt2.y),cv::Scalar(255,0,0),img.cols/500);
        }

        if(_handle)
            _handle->handle(new GSLAM::DebugImageEvent(img,"DebugTrackRefFrameRansac"));
        //cv::imwrite("trackRefFrame_"+std::to_string(refFrame->id())+"-"+std::to_string(_curFrame->id())+".jpg",img);
    }
#endif
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

bool Tracker::trackRefKeyframe(FramePtr ref)
{
//    if(trackRefKeyframeRansac(ref)) return true;
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

    GSLAM::SE3      local2world=ref->getPose();
    GSLAM::SE3      world2local=local2world.inverse();

    vector<pair<Point3d,Point3d> >     observations;
    observations.reserve(matches.size());
    int pnpCount=0;
    for(pair<int,int>& m : matches)
    {
        PointPtr mpt=refFrame->keyPoint(m.first)._mapPoint;
        if(mpt)
        {
            Point3d localRef=world2local*mpt->getPose();
            if(localRef.z>0)
            {
                observations.push_back(std::make_pair(localRef,
                                              _curFrame->keyPoint(m.second)._ptUn));
                pnpCount++;
                continue;
            }
        }

        Point3d epiRef=refFrame->keyPoint(m.first)._ptUn;epiRef.z=-1;
        observations.push_back(std::make_pair(epiRef,
                                           _curFrame->keyPoint(m.second)._ptUn));
    }
    _logger<<",PnP"<<pnpCount;

    if( pnpCount < 10 )
    {
        DLOG(ERROR)<<"Observation number "<<pnpCount<<" < "<<30;
        return false;
    }

    // Initial guess with svd
    std::vector<std::pair<GSLAM::Point2f,GSLAM::Point2f> > matches2d;
    matches2d.reserve(observations.size());
    for(std::pair<GSLAM::CameraAnchor,GSLAM::CameraAnchor> anchor:observations){
        if(anchor.first.z>0)
            anchor.first=anchor.first/anchor.first.z;
        matches2d.push_back(std::make_pair(Point2f(anchor.first.x,anchor.first.y),
                                           Point2f(anchor.second.x,anchor.second.y)));
    }

    GSLAM::SE3 t12;
    std::vector<std::pair<int,GSLAM::Point3d> > mpts;
    if(!_initializer->initialize(matches2d,GSLAM::Camera(),t12,mpts))
    {
        _logger<<",Failed to find relative pose";
        return false;
    }
    std::vector<double> scales;
    for(std::pair<int,GSLAM::Point3d>& mpt:mpts){
        double depth=observations[mpt.first].first.z;
        if(depth<0) continue;
        scales.push_back(depth/mpt.second.z);
    }
    if(scales.size()<5)
    {
        _logger<<", Scales too few.";
        Point3d curXYZ,refXYZ;
        if(!_curFrame->getGPSECEF(curXYZ)) return false;
        if(!ref->getGPSECEF(refXYZ)) return false;
        double distanceGPS=(curXYZ-refXYZ).norm();
        if(distanceGPS<svar.GetInt("GPS.MinDistance2FitScale",5)) return false;
        t12.get_translation()=t12.get_translation()*(distanceGPS/t12.get_translation().norm());
    }
    else{
        // Estimate best scale with ransac
        sort(scales.begin(),scales.end());
        double scale=scales[scales.size()/2];
        t12.get_translation()=t12.get_translation()*scale;
    }
    GSLAM::SE3 t21=t12.inverse();

    // Optimize with optimizer
    std::vector<std::pair<Point3d,Point3d> > anchors;
    std::vector<GSLAM::IdepthEstimation> idepths;
    anchors.reserve(mpts.size());
    idepths.reserve(mpts.size());

    for(std::pair<int,GSLAM::Point3d>& mpt:mpts){
        pair<Point3d,Point3d>& mt=observations[mpt.first];
        double depth=mt.first.z;
        if(depth<0)
        {
            idepths.push_back(Point2d(1./mpt.second.z,-1));
            anchors.push_back(std::make_pair(Point3d(mt.first.x,mt.first.y,1),mt.second));
        }
        else{
            idepths.push_back(Point2d(1./depth,1000));
            anchors.push_back(std::make_pair(mt.first/mt.first.z,mt.second));
        }
    }
    auto optimizer=createOptimizerInstance();
    if(!optimizer->optimizePose(anchors,idepths,t12))
    {
        _logger<<", Failed optimize";
        return false;
    }

    SE3             local2cur=t12.inverse();

    vector<int>     inliers,allInliers;
    inliers.reserve(observations.size());

    for(int i=0;i<observations.size();i++)
    {
        pair<Point3d,Point3d>& obs=observations[i];
        if(obs.first.z<0)
        {
            const Point3d&  t  = local2cur.get_translation();
            const Point3d&  p2d= obs.second;
            const Point3d&  p =obs.first;
            Point3d tcrosspl(t[1]-t[2]*p2d[1],
                              t[2]*p2d[0]-t[0],
                              t[0]*p2d[1]-t[1]*p2d[0]);
            Point3d line=local2cur.get_rotation().inv()*tcrosspl;
            double _error=(fabs(line[0]*p[0]+line[1]*p[1]+line[2])/sqrt(line[0]*line[0]+line[1]*line[1]+1e-10));
            if(_error<0.02) allInliers.push_back(i);
            continue;
        }
        Point3d pt=local2cur*observations[i].first;
        pt=pt/pt.z;
        Point3d error=observations[i].second-pt;
        if(error.norm()<0.02)
        {
            inliers.push_back(i);
            allInliers.push_back(i);
        }
    }

    _logger<<",Inliers"<<inliers.size()<<",AllInliers"<<allInliers.size();

#ifdef HAS_OPENCV
    if(svar.GetInt("DebugTrackRefFrame",0))
    {
        cv::Mat img=_curFrame->getImage().clone();
        for(auto m:matches)
        {
            GSLAM::Point2f pt1,pt2;
            _lastFrame->getKeyPoint(m.first,pt1);
            _curFrame->getKeyPoint(m.second,pt2);
            cv::line(img,cv::Point2f(pt1.x,pt1.y),cv::Point2f(pt2.x,pt2.y),cv::Scalar(0,0,255),img.cols/500);
        }
        for(int idx:allInliers)
        {
            auto m=matches[idx];
            GSLAM::Point2f pt1,pt2;
            _lastFrame->getKeyPoint(m.first,pt1);
            _curFrame->getKeyPoint(m.second,pt2);
            cv::line(img,cv::Point2f(pt1.x,pt1.y),cv::Point2f(pt2.x,pt2.y),cv::Scalar(0,255,0),img.cols/500);
        }
        for(int idx:inliers)
        {
            auto m=matches[idx];
            GSLAM::Point2f pt1,pt2;
            _lastFrame->getKeyPoint(m.first,pt1);
            _curFrame->getKeyPoint(m.second,pt2);
            cv::line(img,cv::Point2f(pt1.x,pt1.y),cv::Point2f(pt2.x,pt2.y),cv::Scalar(255,0,0),img.cols/500);
        }

        if(_handle)
            _handle->handle(new GSLAM::DebugImageEvent(img,"DebugTrackRefFrame"));
        //cv::imwrite("trackRefFrame_"+std::to_string(refFrame->id())+"-"+std::to_string(_curFrame->id())+".jpg",img);
    }
#endif

    if(inliers.size()<std::max(5,int(pnpCount/3)))
    {
        DLOG(ERROR)<<"Inliers too few.";
        return false;
    }

    for(int idx:inliers)
    {
        pair<int,int>& m=matches[idx];
        PointPtr mpt=refFrame->keyPoint(m.first)._mapPoint;
        if(!mpt) continue;
        if(_curFrame->getKeyPointObserve(m.second)) continue;
        _curFrame->addObservation(mpt,m.second);
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
            // FIXME: need to sort the parent by their common observation?
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
        pi::Point2d pimg=cam.Project(pc);
        if(pimg.x<=0||pimg.y<=0||pimg.x>=cam.width()||pimg.y>=cam.height()) continue;
        pi::Point3d nvec=world2camera.get_rotation()*pt->getNormal();
        if(nvec.z>-0.7) continue;
        float searchRadius=nvec.z<-0.9?cam.width()*0.004:cam.width()*0.007;
        int idx;
        tryCount++;
        if(_matcher->findMatchWindow(pt->getDescriptor(),
                           _curFrame,pimg.x,pimg.y,searchRadius,idx))
            matches.push_back(make_pair(idx,pt));
    }

    int matchNumber=_curFrame->observationNum()+matches.size();
    _logger<<",Try:"<<tryCount
          <<"-Matched"<<matches.size();
//    if(tryCount<200) svar.GetInt("EmmergencyKF")=1;

//    timer.leave("Tracker::trackSubMapByProjection::1.CM");
    if(observations.size()+matches.size()<100)
    {
        svar.GetInt("EmmergencyKF")=1;
        return false;
    }


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
        // FIXME: when running phantom3-npu:
        //      local2camera is empty, therefore, parameter "useExtrinsicGuess" of
        //      findPnPRansac need set to false
        SE3 local2camera;
        _estimator->findPnPRansac(local2camera,mpts,kpts,Camera(),
                                  false,std::max(100,int(mpts.size()/3)),
                                  0.01,std::max(30,int(mpts.size()/2)),&inliers);
//        timer.leave("Tracker::trackSubMapByProjection::2.PNP");

        _logger<<"-"<<inliers.size();

        if( inliers.size()<minInlierNum ) return false;

        vector<bool> tracked(mpts.size(),false);
        for(auto idx:inliers) tracked[idx]=true;
        // check observation
        if(0)
        {
            int mptCount=0;
            std::map<GSLAM::PointID,size_t> observes;
            _curFrame->getObservations(observes);
            for(int i=0;i<_curFrame->keyPointNum();i++){
                auto& mpt=_curFrame->keyPoint(i)._mapPoint;
                if(!mpt) continue;
                if(!observes.count(mpt->id())){
                    LOG(ERROR)<<"Count not match "
                             <<mpt->id()<<", featID:"<<i;
                }
                mptCount++;
            }
            if(mptCount!=observes.size())
                LOG(ERROR)<<"Size not match";
        }
        for(int i=0;i<tracked.size();i++)
        {
            SPtr<GSLAM::MapPoint>& mpt=_curFrame->keyPoint(featIds[i])._mapPoint;
            if(tracked[i]&&!mpt.get()) _curFrame->addObservation(pts[i],featIds[i]);
            else if(mpt.get()&&!tracked[i])
            {
                _curFrame->eraseObservation(mpt);
                mpt.reset();
            }
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

    _logger<<",RL";
    GSLAM::LoopCandidates candidates;
    if(!_map->obtainCandidates(_curFrame,candidates)) return false;

    bool bOK=false;
    for(int i=0;i<1&&i<candidates.size();i++)
    {
        GSLAM::LoopCandidate can=candidates[i];
        FramePtr ref=_map->getFrame(can.frameId);
        if(!ref) continue;
        if(!trackRefKeyframeRansac(ref)) continue;
        bOK=true;
        _status=StatusTracking;
        _lastKF=std::dynamic_pointer_cast<MapFrame>(ref);
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

///
/// \brief Tracker::reOptimizePose re-optimize given frame's POSE using keypoint(2d) - mappoint(3d)
///         through PnP algorithm
///
/// FIXME:
///     On Linux platform, it work fine, but on Windows, it will cause lost?
///
/// \param frame        [in] Given mapframe
/// \param pose         [out] re-optimized mapframe
///
/// \return
///     true            optimized sucessful
///     false           failed
///
bool Tracker::reOptimizePose(SPtr<MapFrame>& frame, GSLAM::SE3& pose)
{
    GSLAM::SE3      local2world=frame->getPose();
    GSLAM::SE3      world2local=local2world.inverse();
    SE3             local2cur;

    vector<pair<Point3d,Point3d> >  observations;

    for(int i=0,iend=frame->keyPointNum();i<iend;i++)
    {
        // look for match
        PointPtr mpt=frame->keyPoint(i)._mapPoint;
        if(!mpt) continue;

        Point3d est = world2local*mpt->getPose();
        GSLAM::Point3d obs = frame->keyPoint(i)._ptUn;

        if(est.z>0)
        {
            observations.push_back(std::make_pair(est,obs));
        }
    }


    timer.enter("Tracker::reOptimizePose::opt");
    SPtr<GSLAM::Optimizer> optimizer=createOptimizerInstance();
    bool bOK=optimizer->optimizePnP(observations,local2cur);
    timer.leave("Tracker::reOptimizePose::opt");
    if( !bOK ) return false;


    pose = local2world*local2cur.inverse();
    SE3 poseDiff = frame->getPose().inverse()*pose;

    //std::cout << "prev pose: " << local2world << "\n";
    //std::cout << "opti pose: " << pose << "\n";
    //std::cout << "pose diff: " << poseDiff << "\n";

    {
        pi::Array_<double,6> ln=poseDiff.ln();
        double mIDepth=1./frame->getMedianDepth();
        for(int i=0;i<3;i++) ln.data[i]*=mIDepth;

        double weight[]={1.,1.,1.,1,1,0.01};
        double sum=0;
        for(int i=0;i<6;i++) sum+=weight[i]*ln.data[i]*ln.data[i];
        sum=sqrt(sum);

        //std::cout << "diff FOV :" << sum << "\n";
    }

    return true;
}

bool Tracker::addKeyframeIfNeeded()
{
//    return addKeyframeIfNeeded1();
    std::map<FrameID,SPtr<TrackFrameConnection> > parents;

    std::map<PointID,size_t> observes;// featureID
    _curFrame->getObservations(observes);
    for(pair<PointID,size_t> obs : observes)
    {
        const PointPtr& pt=_curFrame->keyPoint(obs.second)._mapPoint;
        if(!pt->observationNum()){
            _curFrame->eraseObservation(pt);
            continue;
        }
        std::map<FrameID,size_t> ptObserves;
        pt->getObservations(ptObserves);
        for(pair<FrameID,size_t> ptObs:ptObserves)
        {
            auto it=parents.find(ptObs.first);
            if(it==parents.end())
            {
                TrackFrameConnection* con=new TrackFrameConnection();
                con->_matches.push_back(pair<int,int>(ptObs.second,obs.second));
                parents.insert(make_pair(ptObs.first,SPtr<TrackFrameConnection>(con)));
            }
            else
            {
                SPtr<TrackFrameConnection>& con=it->second;
                con->_matches.push_back(pair<int,int>(ptObs.second,obs.second));
            }
        }
    }

    int mostMatches=0;
    FrameID   refId=0;
    for(pair<FrameID,SPtr<TrackFrameConnection> > it:parents)
    {
        if(it.second->matchesNum()>10&&!_curFrame->getParent(it.first))
        {
            _curFrame->addParent(it.first,it.second);
        }
        if(it.second->matchesNum()>mostMatches){
            mostMatches=it.second->matchesNum();
            refId=it.first;
        }
    }

    GSLAM::FramePtr refFrame=_map->getFrame(refId);
    //    if(refFrame)
    //        _lastKF=std::dynamic_pointer_cast<MapFrame>(refFrame);


    // detech FOV change ratio
    SE3 cur2kf=_lastKF->getPose().inverse()*_curFrame->getPose();
    pi::Array_<double,6> ln=cur2kf.ln();
    double mIDepth=1./_lastKF->getMedianDepth();
    for(int i=0;i<3;i++) ln.data[i]*=mIDepth;

    double weight[]={1.,1.,1.,1,1,0.01};
    double sum=0;
    for(int i=0;i<6;i++) sum+=weight[i]*ln.data[i]*ln.data[i];
    sum=sqrt(sum);

    // add keyframe if FOV change is too large
    _logger<<",FOV:"<<sum;
    if(sum>=1-svar.GetDouble("SLAM.MaxOverlap",0.95)||svar.GetInt("EmmergencyKF"))
    {
        svar.GetInt("EmmergencyKF")=0;
        if( _mapper->insertKeyFrame(_curFrame) )
        {
            // reoptimize last KF
            if( svar.GetInt("SLAM.ReoptimPose", 0) )
            {
                GSLAM::SE3 reoptPose;
                if( reOptimizePose(_lastKF, reoptPose) ) _lastKF->setPose(reoptPose);
            }

            return true;
        }
    }

    return false;
}

bool Tracker::addKeyframeIfNeeded1()
{
    const int nKFs = _map->frameNum();

//    // Do not insert keyframes if not enough frames have passed from last relocalisation
//    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && nKFs>mMaxFrames)
//        return false;

    // Tracked MapPoints in the reference keyframe
    int nMinObs = 3;
    if(nKFs<=2)
        nMinObs=2;
    SPtr<MapFrame> refKF=_lastKF;
    std::map<GSLAM::PointID,size_t> observations;
    _lastKF->getObservations(observations);
    int nRefMatches=0;
    for(std::pair<GSLAM::PointID,size_t> obs:observations){
        PointPtr mpt=refKF->keyPoint(obs.second)._mapPoint;
        if(!mpt) continue;
        if(mpt->observationNum()>=nMinObs) nRefMatches++;
    }
    // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
    const bool c1a = _curFrame->timestamp()-refKF->timestamp()>1.;
    // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
    const bool c1b = svar.GetInt("Mapper.Idle");
    // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
    const bool c2 = (_curFrame->observationNum()<nRefMatches*0.9 && _curFrame->observationNum()>15);


    _logger<<",refM"<<nRefMatches;

    if((c1a||c1b)&&c2) {
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


bool Tracker::optimizePnPEpipolar(const std::vector<std::pair<Point3d,Point3d> >& matches,
                         GSLAM::SE3& pose)
{
    GSLAM::ScopedTimer tm("Tracker::optimizePnPEpipolar");

    std::vector<std::pair<GSLAM::CameraAnchor,GSLAM::CameraAnchor> > anchors=matches;
    for(std::pair<GSLAM::CameraAnchor,GSLAM::CameraAnchor>& anchor:anchors){
        if(anchor.first.z<0){
            anchor.first.z=1;
        }
        else anchor.first=anchor.first/anchor.first.z;
    }


    // filt with fundamental
    vector<u_char>         mask;
    {
        double F[9];
        vector<GSLAM::Point2d> points1,points2;
        for(auto& m:anchors)
        {
            points1.push_back(GSLAM::Point2d(m.first.x,m.first.y));
            points2.push_back(GSLAM::Point2d(m.second.x,m.second.y));
        }

        _estimator->findFundamental(F,points1,points2,GSLAM::RANSAC,0.01,0.99,&mask);
    }

    // reduce
    if(mask.size())
    {
        int curSize=0;
        for(size_t i=0;i<anchors.size();i++){
            if(mask[i])
                anchors[curSize++]=anchors[i];
        }
        anchors.resize(curSize);
    }

    auto optimizer=createOptimizerInstance();
    std::vector<GSLAM::IdepthEstimation> idepths(anchors.size(),GSLAM::Point2d(1,-1));
    GSLAM::SE3 t12;
    if(!optimizer->optimizePose(anchors,idepths,t12))
    {
        LOG(WARNING)<<"Failed optimize.";
        return false;
    }

    if(t12.get_translation().norm()<0.01)
    {
        LOG(WARNING)<<"No enough translation.";
        return false;
    }

    std::vector<int>    inliers;
    std::vector<double> scales;
    GSLAM::SE3 t21=t12.inverse();

    for(int i=0;i<idepths.size();i++)
    {
        if(idepths[i].x<=0.1||idepths[i].x>20) continue;

        GSLAM::Point3d pt=anchors[i].first/idepths[i].x;
        GSLAM::Point3d pt2=t21*pt;
        if(pt2.z<=0.001) continue;
        pt2=pt2/pt2.z;

        pt2=pt2-anchors[i].second;
        if(pt2.dot(pt2)>2e-4) continue;

        inliers.push_back(i);
        if(matches[i].first.z>0){//PnP edge
            scales.push_back(matches[i].first.z*idepths[i].x);
        }
    }
    if(inliers.size()<idepths.size()*0.6)
    {
        LOG(ERROR)<<"Inliers too few. Has "<<inliers.size()<<", need "<<idepths.size()*0.6;
        return false;
    }
    if(scales.size()<5)
    {
        LOG(ERROR)<<"Scales too few.";
        return false;
    }
    // Estimate best scale with ransac, the scale should between 0.1-10
    sort(scales.begin(),scales.end());
    double scale=scales[scales.size()/2];
    t12.get_translation()=t12.get_translation()*scale;
    pose=t12.inverse();
    return true;
}

bool Tracker::estimateRelative(const std::vector<std::pair<Point3d,Point3d> >& matches,
                      GSLAM::SE3& pose)
{
#ifdef HAS_OPENCV
    GSLAM::ScopedTimer tm("Tracker::estimateRelative");


    {
        std::vector<std::pair<GSLAM::Point2f,GSLAM::Point2f> > matches2d;
        matches2d.reserve(matches.size());
        for(std::pair<GSLAM::CameraAnchor,GSLAM::CameraAnchor> anchor:matches){
            if(anchor.first.z>0)
                anchor.first=anchor.first/anchor.first.z;
            matches2d.push_back(std::make_pair(Point2f(anchor.first.x,anchor.first.y),
                                               Point2f(anchor.second.x,anchor.second.y)));
        }
        GSLAM::SE3 t21;
        std::vector<std::pair<int,GSLAM::Point3d> > mpts;
        if(!_initializer->initialize(matches2d,GSLAM::Camera(),t21,mpts))
        {
            _logger<<",Failed to find relative pose";
            return false;
        }
        std::vector<double> scales;
        for(std::pair<int,GSLAM::Point3d>& mpt:mpts){
            double depth=matches[mpt.first].first.z;
            if(depth<0) continue;
            scales.push_back(depth/mpt.second.z);
        }
        if(scales.size()<5)
        {
            _logger<<", Scales too few.";
            return false;
        }
        // Estimate best scale with ransac, the scale should between 0.1-10
        sort(scales.begin(),scales.end());
        double scale=scales[scales.size()/2];
        GSLAM::SE3 t12=t21.inverse();
        t12.get_translation()=t12.get_translation()*scale;
        pose=t12.inverse();
        return true;
    }

    using cv::Mat;
    std::vector<std::pair<GSLAM::CameraAnchor,GSLAM::CameraAnchor> > anchors=matches;
    for(std::pair<GSLAM::CameraAnchor,GSLAM::CameraAnchor>& anchor:anchors){
        if(anchor.first.z<0){
            anchor.first.z=1;
        }
        else anchor.first=anchor.first/anchor.first.z;
    }
    // filt with fundamental
    vector<u_char>         mask;
    cv::Mat E(3,3,CV_64F);
    {
        vector<GSLAM::Point2d> points1,points2;
        for(auto& m:anchors)
        {
            points1.push_back(GSLAM::Point2d(m.first.x,m.first.y));
            points2.push_back(GSLAM::Point2d(m.second.x,m.second.y));
        }

        _estimator->findFundamental(&E.at<double>(0),points1,points2,GSLAM::RANSAC,0.01,0.99,&mask);
    }

    // reduce
    vector<cv::Point2f> refInliers,curInliers;
    refInliers.reserve(mask.size());
    curInliers.reserve(mask.size());
    if(mask.size())
    {
        int curSize=0;
        for(size_t i=0;i<anchors.size();i++){
            if(mask[i])
            {
                std::pair<GSLAM::CameraAnchor,GSLAM::CameraAnchor> a=anchors[i];
                refInliers.push_back(cv::Point2f(a.first.x,a.first.y));
                curInliers.push_back(cv::Point2f(a.second.x,a.second.y));
                anchors[curSize++]=anchors[i];
            }
        }
        anchors.resize(curSize);
    }

    // Decompose E to R,t
    cv::Mat R1, R2, t;
    cv::Mat D, U, Vt;
    cv::SVD::compute(E, D, U, Vt);

    if (cv::determinant(U) < 0) U *= -1.;
    if (cv::determinant(Vt) < 0) Vt *= -1.;

//        cv::Mat W = (cv::Mat_<double>(3, 3) << 0, 1, 0, -1, 0, 0, 0, 0, 1);
    cv::Mat W(3,3,CV_32F,cv::Scalar(0));
    W.at<float>(0,1)=-1;
    W.at<float>(1,0)=1;
    W.at<float>(2,2)=1;
    W.convertTo(W, E.type());
    R1 = U * W * Vt;
    R2 = U * W.t() * Vt;
    t = U.col(2) * 1.0;
    vector<std::pair<cv::Mat,cv::Mat> > Rt={{R1,t},{R2,t},{R1,-t},{R2,-t}};
    int bestCount=0;
    std::pair<cv::Mat,cv::Mat> bestRt;
    cv::Mat bestQ;
    cv::Mat bestMask;
    double dist = 500.0;
    cv::Mat P0 = cv::Mat::eye(3, 4, R1.type());
    for(int i=0;i<Rt.size();i++)
    {
        cv::Mat& R=Rt[i].first;
        cv::Mat& t=Rt[i].second;
        Mat P1(3, 4, R.type());
        P1(cv::Range::all(), cv::Range(0, 3)) = R * 1.0; P1.col(3) = t * 1.0;
        cv::Mat Q;
        cv::triangulatePoints(P0, P1, refInliers, curInliers, Q);
        Mat mask = Q.row(2).mul(Q.row(3)) > 0;
        int count0=cv::countNonZero(mask);
        Q.row(0) /= Q.row(3);
        Q.row(1) /= Q.row(3);
        Q.row(2) /= Q.row(3);
        Q.row(3) /= Q.row(3);
        mask = (Q.row(2) < dist) & mask;
        int count1=cv::countNonZero(mask);
        P1.convertTo(P1,Q.type());
        Q = P1 * Q;
        mask = (Q.row(2) > 0) & mask;
        int count2=cv::countNonZero(mask);
        mask = (Q.row(2) < dist) & mask;
        int count=cv::countNonZero(mask);
        if(count>bestCount)
        {
            bestCount=count;
            bestRt=Rt[i];
            bestQ=Q;
            bestMask=mask;
        }
    }

    std::vector<int>    inliers;
    std::vector<double> scales;
    GSLAM::SE3 t21;

    {
        cv::Mat& r=bestRt.first;
        cv::Mat& t=bestRt.second;
        r.convertTo(r,CV_64F);t.convertTo(t,CV_64F);
        double M[12];
        M[0]=r.at<double>(0,0);M[1]=r.at<double>(0,1);M[2]=r.at<double>(0,2); M[3] =t.at<double>(0);
        M[4]=r.at<double>(1,0);M[5]=r.at<double>(1,1);M[6]=r.at<double>(1,2); M[7] =t.at<double>(1);
        M[8]=r.at<double>(2,0);M[9]=r.at<double>(2,1);M[10]=r.at<double>(2,2);M[11]=t.at<double>(2);
        t21.fromMatrix(M);
    }

    for(int i=0;i<anchors.size();i++)
    {
        if(!bestMask.at<uchar>(i)) continue;

        cv::Mat Q=bestQ.col(i);
        Q.convertTo(Q,CV_64F);

        if(Q.at<double>(2)<=0) continue;

        inliers.push_back(i);
        if(matches[i].first.z>0){//PnP edge
            scales.push_back(matches[i].first.z/Q.at<double>(2));
        }
    }
    if(inliers.size()<anchors.size()*0.1)
    {
        LOG(ERROR)<<"Inliers too few.";
        return false;
    }
    if(scales.size()<5)
    {
        LOG(ERROR)<<"Scales too few.";
        return false;
    }
    // Estimate best scale with ransac, the scale should between 0.1-10
    sort(scales.begin(),scales.end());
    double scale=scales[scales.size()/2];
    GSLAM::SE3 t12=t21.inverse();
    t12.get_translation()=t12.get_translation()*scale;
    pose=t12.inverse();
    return true;
#else
    return false;
#endif
}

void Tracker::call(const std::string &command, void *arg)
{
    if( command == "Tracker.WaitFinished" )
    {
        _mapper->call("Mapper.WaitFinised");
    }
}



}

typedef opt::Tracker TrackerDemo;
REGISTER_TRACKER(TrackerDemo,opt);
