#ifdef HAS_OPENCV
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#endif

#include "Tracker.h"

#include <set>
#include <sstream>
#include <unordered_map>

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
#ifdef HAS_EIGEN3
#include <Eigen/Dense>
#endif

#ifdef HAS_OPENCV
#include <opencv2/highgui/highgui.hpp>
#endif

namespace rtsfmInit
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
class GPSRectVisualizer;
enum TrackerStatus
{
    StatusInitializing,
    StatusTracking,
    StatusLosted
};

class Evaluater
{
public:
    Evaluater():accessCount(0){}
    ~Evaluater(){
        report();
    }
    void access(){accessCount++;}
    void success(int matchCount,int inlierCount){
        successes.push_back(std::make_pair(matchCount,inlierCount));
    }
    void report(){
        int matchSum=0,inlierSum=0;
        for(auto m:successes) {
            matchSum+=m.first;
            inlierSum+=m.second;
        }
        matchSum/=successes.size();
        inlierSum/=successes.size();
        LOG(INFO)<<"Success: "<<successes.size()<<"/"<<accessCount<<"\nMeanMatches:"<<matchSum<<",MeanInliers:"<<inlierSum;
    }
    int accessCount;
    std::vector<std::pair<int,int> > successes;
};

class Tracker : public ::Tracker
{
public:
    Tracker(const GSLAM::MapPtr& map,const SPtr<Mapper>& mapper);
    bool track(const SPtr<MapFrame>& frame);
    static bool getGPSEdge(const FramePtr& fr,GSLAM::SE3& se3,double* info)
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
private:
    bool initialize();
    bool trackExistMap();
    bool createMapPoints(FramePtr ref);
    void localOptimize();
    bool eraseMapPoint(const GSLAM::PointPtr& mpt);
    bool fitGPS(vector<SPtr<MapFrame> > frames,GSLAM::SIM3& local2ecef);
    void reset(){
        _map->clear();
        _lastKF.reset();
        _lastFrame.reset();
        _status=StatusInitializing;
    }

    static GSLAM::SO3 PYR2Rotation(double pitch,double yaw,double roll)
    {
        if(fabs(180-fabs(roll))<10) roll+=180;
        GSLAM::SO3 camera2IMU(-0.5,0.5,-0.5,0.5);
        GSLAM::SO3 imu2world;
        imu2world.FromEulerAngle(-pitch,90.-yaw,roll);
        return imu2world*camera2IMU;
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
    Evaluater               _evaluation;
    GSLAM::Point3d          _origin;
    SPtr<GPSRectVisualizer> _vis;
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

class GPSRectVisualizer : public GSLAM::GObject
{
public:
    GPSRectVisualizer(GSLAM::MapPtr map)
        :_map(map),
        _origin(svar.get_var("Origin",GSLAM::Point3d(-1710905.8403251697,4993661.439450523,3569159.2821966647)))
    {}

    virtual void draw(){
        GSLAM::FrameArray frames;
        _map->getFrames(frames);

        for(GSLAM::FramePtr fr:frames){
            double info[36];
            GSLAM::SE3 se3;
            if(Tracker::getGPSEdge(fr,se3,info))
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

class StitchVisualizer : public GSLAM::GObject
{
public:
    void update(GSLAM::MapPtr map,FramePtr fr1,FramePtr fr2){

    }

    virtual void draw(){

    }


};

class Connection: public GSLAM::FrameConnection
{
public:
    Connection(const vector<pair<int,int> >& matches=vector<pair<int,int> >())
        :_matches(matches){}

    virtual std::string type()const{return "TrackFrameConnection";}
    virtual int  matchesNum(){return _matches.size();}

    virtual bool getMatches(std::vector<std::pair<int,int> >& matches){matches=_matches;return true;}

    vector<pair<int,int> > _matches;
};

Tracker::Tracker(const GSLAM::MapPtr& map,const SPtr<Mapper>& mapper)
    : ::Tracker(map,mapper),_status(StatusInitializing),
      _origin(svar.get_var("Origin",GSLAM::Point3d(-1710905.8403251697,4993661.439450523,3569159.2821966647)))
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
    bool bOK=true;
    if(_status==StatusInitializing)
    {
        bOK= initialize();
        _curFrame->setImage(GSLAM::GImage());
        if(bOK){
            _status=StatusTracking;
            return bOK;
        }
    }

    if(_status==StatusTracking){
        bOK=trackExistMap();
        _curFrame->setImage(GSLAM::GImage());
        if(!bOK) _status=StatusInitializing;
    }
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

        fr1->setPose(local2ecef*fr1->getPose());
        fr2->setPose(local2ecef*fr2->getPose());
        return true;
    }

    // fit with SIM3 Estimator
    LOG(WARNING)<<"Using SIM3 Estimator.";

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
        LOG(ERROR)<<"Failed to findSIM3.";
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
        LOG(ERROR)<<"GPS Error too large.";
        return false;
    }

    local2ecef=local2ECEF*fr12ecef*fr1->getPose().inverse();

    fr1->setPose(local2ecef*fr1->getPose());
    fr2->setPose(local2ecef*fr2->getPose());

    return true;
}

bool Tracker::initialize()
{
    if(!_lastKF) { _lastKF=_curFrame; return true;}
    if(_curFrame->timestamp()-_lastKF->timestamp()<1.) return false;

    _evaluation.access();
    GSLAM::ScopedTimer mtimer("Tracker::initialize");
    vector<pair<int,int> > matches;
    _logger<<",Init with "<<_lastKF->id();

    // 初始化失败原因：
    // 1.match4initialize函数失败
    // 2.找到的特征点匹配数量太少
    if(!_matcher->match4initialize(_lastKF,_curFrame,matches)||matches.size()<std::max(100,_lastKF->keyPointNum()/10))
    {
        _logger<<",Match"<<matches.size();
        _lastKF=_curFrame;
#ifdef HAS_OPENCV
        if(svar.GetInt("DebugInitialize",1))
        {
            cv::Mat img=_curFrame->getImage().clone();
            for(auto m:matches)
            {
                GSLAM::Point2f pt1,pt2;
                _lastKF->getKeyPoint(m.first,pt1);
                _curFrame->getKeyPoint(m.second,pt2);
                // initialize失败，所有匹配点画蓝色的线
                cv::line(img,cv::Point2f(pt1.x,pt1.y),cv::Point2f(pt2.x,pt2.y),cv::Scalar(255,0,0),img.cols/500);
            }
            if(_handle)
                _handle->handle(new GSLAM::DebugImageEvent(img,"DebugInitialize"));

            //cv::imwrite("trackLastFrame_"+std::to_string(_lastFrame->id())+"-"+std::to_string(_curFrame->id())+".jpg",img);
        }
#endif
        return false;
    }
    _logger<<",Match"<<matches.size();


    GSLAM::FrameConnectionPtr con(new Connection(matches));
    _lastKF->addChildren(_curFrame->id(),con);
    _curFrame->addParent(_lastKF->id(),con);

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
        _lastKF = _curFrame;
        _logger<<", Initialize Failed";
#ifdef HAS_OPENCV
        if(svar.GetInt("DebugInitialize",1))
        {
            cv::Mat img=_curFrame->getImage().clone();
            for(auto m:matches)
            {
                GSLAM::Point2f pt1,pt2;
                _lastKF->getKeyPoint(m.first,pt1);
                _curFrame->getKeyPoint(m.second,pt2);
                cv::line(img,cv::Point2f(pt1.x,pt1.y),cv::Point2f(pt2.x,pt2.y),cv::Scalar(255,0,0),img.cols/500);
            }
            for(pair<int,Point3d>& pt:mpts)
            {
                auto m=matches[c2midx[pt.first]];
                GSLAM::Point2f pt1,pt2;
                _lastKF->getKeyPoint(m.first,pt1);
                _curFrame->getKeyPoint(m.second,pt2);
                cv::line(img,cv::Point2f(pt1.x,pt1.y),cv::Point2f(pt2.x,pt2.y),cv::Scalar(0,0,255),img.cols/500);
            }
            if(_handle)
                _handle->handle(new GSLAM::DebugImageEvent(img,"DebugInitialize"));

            //cv::imwrite("trackLastFrame_"+std::to_string(_lastFrame->id())+"-"+std::to_string(_curFrame->id())+".jpg",img);
        }
#endif
        return false;
    }

    SE3 lastKFPose=_lastKF->getPose();
    _lastKF->setPose(SE3());
    _curFrame->setPose(cur2lastkf);

    if(svar.GetInt("GPS.EnableFitGPS",1))
    {
        GSLAM::SIM3 local2ecef;
        if(fitGPS({_lastKF,_curFrame},local2ecef)) {

            if(!_vis)
            {
                SPtr<GPSRectVisualizer> vis(new GPSRectVisualizer(_map));

#if (GSLAM_VERSION_MAJOR<<16|GSLAM_VERSION_MINOR<<8|GSLAM_VERSION_PATCH) >= 0x020402
                _handle->handle(new GSLAM::DrawableEvent(vis,"GPSRect"));
#endif
                _vis=vis;
            }
            for(pair<int,Point3d>& pt:mpts)
            {
                pt.second=local2ecef*pt.second;
            }

            auto optimizer=createOptimizerInstance();

            GSLAM::BundleGraph graph;
//            graph.camera=_curFrame->getCamera().estimatePinHoleCamera();
            graph.keyframes.push_back({(SE3)_lastKF->getPose(),GSLAM::UPDATE_KF_SE3});
            graph.keyframes.push_back({(SE3)_curFrame->getPose(),GSLAM::UPDATE_KF_SE3});
            GSLAM::SE3 lastPrior,curPrior;
            double infoLast[36],infoCur[36];
            getGPSEdge(_lastKF,lastPrior,infoLast);
            getGPSEdge(_curFrame,curPrior,infoCur);
            graph.gpsGraph.push_back({0,lastPrior,infoLast});
            graph.gpsGraph.push_back({1,curPrior,infoCur});

            std::vector<Point3d> errors;
            for(pair<int,Point3d>& pt:mpts)
            {
                int mi=c2midx[pt.first];
                pair<int,int> m=matches[mi];
                auto p1=(_lastKF->keyPoint(m.first)._ptUn);
                auto p2=(_curFrame->keyPoint(m.second)._ptUn);
                graph.mappointObserves.push_back({graph.mappoints.size(),0,_lastKF->keyPoint(m.first)._ptUn});
                graph.mappointObserves.push_back({graph.mappoints.size(),1,_curFrame->keyPoint(m.second)._ptUn});
                graph.mappoints.push_back({pt.second,true});
                auto e1=_lastKF->getPose().inverse()*pt.second;
                auto e2=_curFrame->getPose().inverse()*pt.second;
                errors.push_back(e1/e1.z-p1);
                errors.push_back(e2/e2.z-p2);

            }
            {
            std::vector<double> projErrors;
            for(GSLAM::BundleEdge obs:graph.mappointObserves){
                Point3d camP=graph.keyframes[obs.frameId].estimation.get_se3().inverse()*graph.mappoints[obs.pointId].first;
                auto e=camP/camP.z-obs.measurement;
                projErrors.push_back(e.dot(e));
            }
            std::sort(projErrors.begin(),projErrors.end());
            std::cerr<<"Before:"<<projErrors[projErrors.size()/2]<<std::endl;
            }

            svar.GetPointer("BundleStopFlagPointer")=nullptr;
            if(optimizer->optimize(graph))
            {
                _lastKF->setPose(graph.keyframes[0].estimation);
                _curFrame->setPose(graph.keyframes[1].estimation);

                std::vector<double> projErrors;
                for(GSLAM::BundleEdge obs:graph.mappointObserves){
                    Point3d camP=graph.keyframes[obs.frameId].estimation.get_se3().inverse()*graph.mappoints[obs.pointId].first;
                    auto e=camP/camP.z-obs.measurement;
                    projErrors.push_back(e.dot(e));
                }
                std::sort(projErrors.begin(),projErrors.end());
                std::cerr<<"After:"<<projErrors[projErrors.size()/2]<<std::endl;

                for(int i=0;i<mpts.size();i++)
                {
                    mpts[i].second=graph.mappoints[i].first;
                }
            }

            svar.GetInt("GPS.Fitted",0)=1;
        }
        else
        {
            svar.GetInt("GPS.Fitted")=0;
            _map->clear();
        }
    }
    else
        _map->clear();

//    _map->clear();
    {
        auto p1=_lastKF->getPose();p1.get_translation()=p1.get_translation()-_origin;_lastKF->setPose(p1);
        auto p2=_curFrame->getPose();p2.get_translation()=p2.get_translation()-_origin;_curFrame->setPose(p2);
    }

    _map->insertMapFrame(_lastKF);
    _map->insertMapFrame(_curFrame);

    Point3d t=_curFrame->getPose().get_translation();
    for(pair<int,Point3d>& pt:mpts)
    {
        pt.second=pt.second-_origin;
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



    _logger<<", Created map between frame "<<_lastKF->id()<<" and "<<_curFrame->id()
            <<", with "<<mpts.size()<<" mapoints.";

    if(_handle) _handle->handle(_map);

#ifdef HAS_OPENCV
    if(svar.GetInt("DebugInitialize",1))
    {
        cv::Mat img=_curFrame->getImage().clone();
        for(auto m:matches)
        {
            GSLAM::Point2f pt1,pt2;
            _lastKF->getKeyPoint(m.first,pt1);
            _curFrame->getKeyPoint(m.second,pt2);
            cv::line(img,cv::Point2f(pt1.x,pt1.y),cv::Point2f(pt2.x,pt2.y),cv::Scalar(255,0,0),img.cols/500);
        }
        for(pair<int,Point3d>& pt:mpts)
        {
            auto m=matches[c2midx[pt.first]];
            GSLAM::Point2f pt1,pt2;
            _lastKF->getKeyPoint(m.first,pt1);
            _curFrame->getKeyPoint(m.second,pt2);
            cv::line(img,cv::Point2f(pt1.x,pt1.y),cv::Point2f(pt2.x,pt2.y),cv::Scalar(0,0,255),img.cols/500);
        }
        if(_handle)
            _handle->handle(new GSLAM::DebugImageEvent(img,"DebugInitialize"));

        //cv::imwrite("trackLastFrame_"+std::to_string(_lastFrame->id())+"-"+std::to_string(_curFrame->id())+".jpg",img);
    }
#endif
    LOG(INFO)<<"LastPoseDiff:"<<(lastKFPose.inverse()*_lastKF->getPose()).ln();
    _lastKF=_curFrame;
    _evaluation.success(matches.size(),mpts.size());
    return true;
}

bool triangulate(double* t1,double* t2,const GSLAM::Point3d& xn1,
                 const GSLAM::Point3d& xn2,GSLAM::Point3d& pt)
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


        pt=GSLAM::Point3d(x3D[0]/x3D[3],x3D[1]/x3D[3],x3D[2]/x3D[3]);
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
        pt=GSLAM::Point3d(x3D.at<double>(0),x3D.at<double>(1),x3D.at<double>(2));
        return true;
    }
#endif

    return false;
}


bool Tracker::createMapPoints(FramePtr refKF){
    GSLAM::SE3  ref2cur  =_curFrame->getPose().inverse()*refKF->getPose();
    GSLAM::SE3  cur2ref  =ref2cur.inverse();
    pi::Point3d vBaseline=ref2cur.get_translation();
    GSLAM::Camera refCam =refKF->getCamera();
    double iden[12],cur2refM[12];
    GSLAM::SE3 Iden;
    Iden.getMatrix(iden);
    cur2ref.getMatrix(cur2refM);
    GSLAM::Camera curCam=_curFrame->getCamera();

    auto mDepth=_curFrame->getMedianDepth();
    if(vBaseline.norm()/mDepth<0.01) return false;

    auto connection=_curFrame->getParent(refKF->id());
    std::vector<std::pair<int,int> > matches;
    if(connection) connection->getMatches(matches);
    int successCount=0,count1=0,count2=0;
    for(auto& m:matches)
    {
        auto oldMpt=_curFrame->keyPoint(m.second)._mapPoint;
        if(oldMpt)
        {
            oldMpt->addObservation(refKF->id(),m.first);
            continue;
        }
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
        p3d=_curFrame->getPose()*p3d;
        PointPtr pMP(new MapPoint(_map->getPid(),p3d,(_curFrame->getPose().get_translation()-p3d).normalize(),
                                  _curFrame->keyPoint(m.second)._color,_curFrame->id()));
        pMP->setDescriptor(_curFrame->getDescriptor(m.second).clone());
        _curFrame->addObservation(pMP,m.second,true);
        refKF->addObservation(pMP,m.first,true);
        _map->insertMapPoint(pMP);
        successCount++;
    }
    _logger<<",ref:"<<refKF->id()<<"-"<<successCount
          <<"/"<<count2<<"/"<<count1<<"/"<<matches.size();
    return true;
}

bool Tracker::eraseMapPoint(const GSLAM::PointPtr& mpt)
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
void Tracker::localOptimize()
{
    using namespace GSLAM;
    using namespace std;
    auto _optimizer=createOptimizerInstance();
    if(!_optimizer)
    {
        LOG(ERROR)<<"No valid optimizer!";
        return;
    }
    GSLAM::ScopedTimer mTimer("Tracker::localOptimization");

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
//    std::map<GSLAM::FrameID,SPtr<GSLAM::FrameConnection> >    parents;
//    if(!((::MapFrame*)_curFrame.get())->getParents(parents))
//    {
//        LOG(ERROR)<<"Failed to obtain parents!";
//        return;
//    }
//    for(auto& con:parents)
//    {
//        GSLAM::FramePtr kf=_map->getFrame(con.first);
//        if(!kf.get()) continue;
//        frameId_map.insert(make_pair(con.first,graph.keyframes.size()));
//        graph.keyframes.push_back({w2l*kf->getPose(),UPDATE_KF_SE3});
//        KeyFrameEles.push_back(kf);
//    }
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
            }
        }
        _logger<<",EO:"<<eraseObsCount
              <<",EP:"<<points2remove.size();
        for(auto& pt:points2remove)
            eraseMapPoint(pt);
    }
}

bool Tracker::trackExistMap()
{
    GSLAM::LoopCandidates candidates;
    if(!_map->obtainCandidates(_curFrame,candidates))
    {
        _logger<<"No candidates.";
        return false;
    }

    FramePtr init_ref;
    for(int i=0;i<candidates.size()&&i<8;i+=1){
        FramePtr ref=_map->getFrame(candidates[i].frameId);
        if(!ref) continue;

        vector<pair<int,int> > matches;
//        if((!init_ref))
        {
            _lastKF=std::dynamic_pointer_cast<MapFrame>(ref);
            if(initialize()) init_ref=ref;continue;
        }

        if(_matcher->match4initialize(ref,_curFrame,matches)){
            GSLAM::FrameConnectionPtr con(new Connection(matches));
            ref->addChildren(_curFrame->id(),con);
            _curFrame->addParent(ref->id(),con);
        }
    }

    if(!init_ref) return false;

    _logger<<",Parents:"<<FramePtr(_curFrame)->getParents().size();

    GSLAM::FrameConnectionMap parents=FramePtr(_curFrame)->getParents();
    for(auto it:parents){
        if(it.first==init_ref->id()) continue;
        createMapPoints(_map->getFrame(it.first));
    }

    localOptimize();
    return true;
}

REGISTER_TRACKER(Tracker,rtsfmInit);
}

