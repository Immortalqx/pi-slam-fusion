#ifdef HAS_OPENCV
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
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
#include "../zhaoyong/optimizerG2O/Optimizer.h"
#include <Eigen/Core>


namespace liu_testInitializer
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

            Point3d pyr(-90,0,0),pyrSigma(1e100,1e100,1e100);
            if(!fr->getPitchYawRoll(pyr)){
                pyr=Point3d(-90,0,0);
                pyrSigma=Point3d(10000,1e100,10000);
            }
            else if(!fr->getPYRSigma(pyrSigma))
            {
                pyrSigma=Point3d(1,10,10);
            }

            // Camera to local : east up north
            GSLAM::SE3 camera2local(PYR2Rotation(pyr.x,pyr.y,pyr.z),
                                    GSLAM::Point3d(0,0,0));

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

            // Camera to ECEF
            se3= local2ECEF*camera2local;

            typedef Eigen::Matrix<double,6,6> Matrix6d;
            typedef Eigen::Vector3d           Vector3d;
            Eigen::Map<Matrix6d> infoM(info);
            infoM=Matrix6d::Identity();
            infoM(0,0)=1./(gpsSigma.x*gpsSigma.x);
            infoM(1,1)=1./(gpsSigma.y*gpsSigma.y);
            infoM(2,2)=1./(gpsSigma.z*gpsSigma.z);
            infoM(3,3)=1./(pyrSigma.x*pyrSigma.x);
            infoM(4,4)=1./(pyrSigma.y*pyrSigma.y);
            infoM(5,5)=1./(pyrSigma.z*pyrSigma.z);
            return true;
        }
    private:
        bool initialize();
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

//        fr1->setPose(local2ecef.get_se3()*fr1->getPose());
//        fr2->setPose(local2ecef.get_se3()*fr2->getPose());
            fr1->setPose(prioryFirst.get_se3());
            fr2->setPose(priorySecond.get_se3());
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

        fr1->setPose(local2ecef.get_se3()*fr1->getPose());
        fr2->setPose(local2ecef.get_se3()*fr2->getPose());

        return true;
    }

    bool Tracker::initialize()
    {
        static cv::Mat lastImg;
        static uint matchFailedNumber = 0;
        static uint InitializeFailedNumber = 0;

        if(!_lastKF) { _lastKF=_curFrame; lastImg = _curFrame->getImage().clone();return true;}

        _evaluation.access();
        GSLAM::ScopedTimer mtimer("Tracker::initialize");
        vector<pair<int,int> > matches;
        _logger<<",Init with "<<_lastKF->id();

        // 初始化失败原因：
        // 1.match4initialize函数失败
        // 2.找到的特征点匹配数量太少
        if(!_matcher->match4initialize(_lastKF,_curFrame,matches)||matches.size()<std::max(100,_lastKF->keyPointNum()/10))
        {
            matchFailedNumber++;
            LOG(WARNING) << "Match4Initialize Failed: " << matchFailedNumber;
            _logger<<",Match"<<matches.size();
#ifdef HAS_OPENCV
            if(svar.GetInt("DebugInitialize",1))
            {
                cv::Mat curImg = _curFrame->getImage().clone();
                cv::Mat showImg = cv::Mat(lastImg.rows, lastImg.cols + curImg.cols + 1, lastImg.type());
                lastImg.colRange(0, lastImg.cols).copyTo(showImg.colRange(0, lastImg.cols));
                curImg.colRange(0, curImg.cols).copyTo(showImg.colRange(lastImg.cols + 1, showImg.cols));

                for(auto m:matches)
                {
                    GSLAM::Point2f pt1,pt2;
                    if(!_lastKF->getKeyPoint(m.first,pt1)) continue;
                    if(!_curFrame->getKeyPoint(m.second,pt2)) continue;
                    // initialize失败，所有匹配点画蓝色的线
                    cv::line(showImg,cv::Point2f(pt1.x,pt1.y),cv::Point2f(pt2.x + lastImg.cols,pt2.y),cv::Scalar(255,0,0),showImg.cols/500);

                    if( pt1.x < 20 ) LOG(WARNING) << m.first << " : " << pt1.x << ", " << pt1.y;
                }
                if(_handle)
                    _handle->handle(new GSLAM::DebugImageEvent(showImg,"DebugInitialize"));

                //cv::imwrite("trackLastFrame_"+std::to_string(_lastFrame->id())+"-"+std::to_string(_curFrame->id())+".jpg",img);
            }
#endif
            _lastKF=_curFrame;
            lastImg = _curFrame->getImage().clone();
            return false;
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
            InitializeFailedNumber++;
            LOG(WARNING) << "Initialize Failed: " << InitializeFailedNumber;
            _logger<<", Initialize Failed";
#ifdef HAS_OPENCV
            if(svar.GetInt("DebugInitialize",1))
            {
                cv::Mat curImg = _curFrame->getImage().clone();
                cv::Mat showImg = cv::Mat(lastImg.rows, lastImg.cols + curImg.cols + 1, lastImg.type());
                lastImg.colRange(0, lastImg.cols).copyTo(showImg.colRange(0, lastImg.cols));
                curImg.colRange(0, curImg.cols).copyTo(showImg.colRange(lastImg.cols + 1, showImg.cols));

                for(auto m:matches)
                {
                    GSLAM::Point2f pt1,pt2;
                    if(!_lastKF->getKeyPoint(m.first,pt1)) continue;
                    if(!_curFrame->getKeyPoint(m.second,pt2)) continue;
                    cv::line(showImg,cv::Point2f(pt1.x,pt1.y),cv::Point2f(pt2.x + lastImg.cols,pt2.y),cv::Scalar(255,0,0),showImg.cols/500);
                }
                for(pair<int,Point3d>& pt:mpts)
                {
                    auto m=matches[c2midx[pt.first]];
                    GSLAM::Point2f pt1,pt2;
                    if(!_lastKF->getKeyPoint(m.first,pt1)) continue;
                    if(!_curFrame->getKeyPoint(m.second,pt2)) continue;
                    cv::line(showImg,cv::Point2f(pt1.x,pt1.y),cv::Point2f(pt2.x + lastImg.cols,pt2.y),cv::Scalar(0,0,255),showImg.cols/500);
                }

                if(_handle)
                    _handle->handle(new GSLAM::DebugImageEvent(showImg,"DebugInitialize"));

                //cv::imwrite("trackLastFrame_"+std::to_string(_lastFrame->id())+"-"+std::to_string(_curFrame->id())+".jpg",img);
            }
#endif
            _lastKF = _curFrame;
            lastImg = _curFrame->getImage().clone();
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
                graph.keyframes.push_back({_lastKF->getPose(),GSLAM::UPDATE_KF_SE3});
                graph.keyframes.push_back({_curFrame->getPose(),GSLAM::UPDATE_KF_SE3});
                GSLAM::SE3 lastPrior,curPrior;
                double infoLast[36],infoCur[36];
                getGPSEdge(_lastKF,lastPrior,infoLast);
                getGPSEdge(_curFrame,curPrior,infoCur);
                graph.gpsGraph.push_back({0,lastPrior,infoLast});
                graph.gpsGraph.push_back({1,curPrior,infoCur});

                for(pair<int,Point3d>& pt:mpts)
                {
                    int mi=c2midx[pt.first];
                    pair<int,int> m=matches[mi];
                    graph.mappointObserves.push_back({graph.mappoints.size(),0,_lastKF->keyPoint(m.first)._ptUn});
                    graph.mappointObserves.push_back({graph.mappoints.size(),1,_curFrame->keyPoint(m.second)._ptUn});
                    graph.mappoints.push_back({pt.second,true});
                }
                {
                    std::vector<double> projErrors;
                    for(GSLAM::BundleEdge obs:graph.mappointObserves){
                        Point3d camP=graph.keyframes[obs.frameId].estimation.get_se3().inverse()*graph.mappoints[obs.pointId].first;
                        Point3d e=camP/camP.z-obs.measurement;
                        projErrors.push_back(e.dot(e));
                    }
                    std::sort(projErrors.begin(),projErrors.end());
                    std::cerr<<"Before:"<<projErrors[projErrors.size()/2]<<std::endl;
                }

                if(optimizer->optimize(graph))
                {
                    _lastKF->setPose(graph.keyframes[0].estimation);
                    _curFrame->setPose(graph.keyframes[1].estimation);

                    std::vector<double> projErrors;
                    for(GSLAM::BundleEdge obs:graph.mappointObserves){
                        Point3d camP=graph.keyframes[obs.frameId].estimation.get_se3().inverse()*graph.mappoints[obs.pointId].first;
                        Point3d e=camP/camP.z-obs.measurement;
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
            cv::Mat curImg = _curFrame->getImage().clone();
            cv::Mat showImg = cv::Mat(lastImg.rows, lastImg.cols + curImg.cols + 1, lastImg.type());
            lastImg.colRange(0, lastImg.cols).copyTo(showImg.colRange(0, lastImg.cols));
            curImg.colRange(0, curImg.cols).copyTo(showImg.colRange(lastImg.cols + 1, showImg.cols));

            for(auto m:matches)
            {
                GSLAM::Point2f pt1,pt2;
                if(!_lastKF->getKeyPoint(m.first,pt1)) continue;
                if(!_curFrame->getKeyPoint(m.second,pt2)) continue;
                cv::line(showImg,cv::Point2f(pt1.x,pt1.y),cv::Point2f(pt2.x + lastImg.cols,pt2.y),cv::Scalar(255,0,0),showImg.cols/500);
            }
            for(pair<int,Point3d>& pt:mpts)
            {
                auto m=matches[c2midx[pt.first]];
                GSLAM::Point2f pt1,pt2;
                if(!_lastKF->getKeyPoint(m.first,pt1)) continue;
                if(!_curFrame->getKeyPoint(m.second,pt2)) continue;
                cv::line(showImg,cv::Point2f(pt1.x,pt1.y),cv::Point2f(pt2.x + lastImg.cols,pt2.y),cv::Scalar(0,0,255),showImg.cols/500);
            }
            if(_handle)
                _handle->handle(new GSLAM::DebugImageEvent(showImg,"DebugInitialize"));

            //cv::imwrite("trackLastFrame_"+std::to_string(_lastFrame->id())+"-"+std::to_string(_curFrame->id())+".jpg",img);
        }
#endif
        LOG(INFO)<<"LastPoseDiff:"<<(lastKFPose.inverse()*_lastKF->getPose()).ln();
        _lastKF=_curFrame;
        _evaluation.success(matches.size(),mpts.size());
        lastImg = _curFrame->getImage().clone();
        return true;
    }
}

typedef liu_testInitializer::Tracker TrackerDemo;
REGISTER_TRACKER(TrackerDemo,liu_testInit);
