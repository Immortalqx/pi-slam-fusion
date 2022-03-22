/**
#include "GSLAM/core/Dataset.h"
#include "GSLAM/core/Svar.h"
#include "GSLAM/core/Timer.h"
#include "GSLAM/core/MapFusion.h"
#include "GSLAM/core/TileManager.h"
#include "GSLAM/core/GPS.h"
#include "GSLAM/core/VecParament.h"

#include <ros/ros.h>
#include "std_msgs/String.h"
#include <QImage>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#ifdef HAS_QT

using namespace std;
using namespace GSLAM;



class LiveImageFrame : public GSLAM::MapFrame
{
public:
    LiveImageFrame(const GSLAM::FrameID& id=0,const double& timestamp=0)
        :GSLAM::MapFrame(id,timestamp) {}

    virtual std::string type()const{return "LiveImageFrame";}

    virtual int     cameraNum()const{return 1;}                                      // Camera number
    virtual GSLAM::SE3     getCameraPose(int idx=0) const{return GSLAM::SE3();}                    // The transform from camera to local
    virtual int     imageChannels(int idx=0) const{return GSLAM::IMAGE_BGRA;}               // Default is a colorful camera
    virtual GSLAM::Camera  getCamera(int idx=0){return _camera;}                            // The camera model
    virtual GSLAM::GImage  getImage(int idx,int channalMask){
        if(idx==0)
        {
            if(_image.empty())
            {
                using namespace GSLAM;
                QImage qimage(_imagePath.c_str());
                if(qimage.format()==QImage::Format_RGB32)
                {
                    return GImage(qimage.height(),qimage.width(),
                                  GImageType<uchar,4>::Type,qimage.bits(),true);
                }
                else if(qimage.format()==QImage::Format_RGB888){
                    return GImage(qimage.height(),qimage.width(),
                                  GImageType<uchar,3>::Type,qimage.bits(),true);
                }
                else if(qimage.format()==QImage::Format_Indexed8)
                {
                    return GImage(qimage.height(),qimage.width(),
                                  GImageType<uchar,1>::Type,qimage.bits(),true);
                }
            }
            else
                return _image;
        }
        else return _thumbnail;
    }   // Just return the image if only one channel is available

    // When the frame contains IMUs or GPSs
    virtual int     getIMUNum()const{return (_gpshpyr.size()==11||_gpshpyr.size()==12||_gpshpyr.size()==14)?1:0;}
    virtual bool    getAcceleration(GSLAM::Point3d& acc,int idx=0)const{return false;}        // m/s^2
    virtual bool    getAngularVelocity(GSLAM::Point3d& angularV,int idx=0)const{return false;}// rad/s
    virtual bool    getMagnetic(GSLAM::Point3d& mag,int idx=0)const{return false;}            // gauss
    virtual bool    getAccelerationNoise(GSLAM::Point3d& accN,int idx=0)const{return false;}
    virtual bool    getAngularVNoise(GSLAM::Point3d& angularVN,int idx=0)const{return false;}
    virtual bool    getPitchYawRoll(GSLAM::Point3d& pyr,int idx=0)const{
        if(_gpshpyr.size()==11&&_gpshpyr[8]<20) {pyr=GSLAM::Point3d(_gpshpyr[5],_gpshpyr[6],_gpshpyr[7]);return true;}
        else if(_gpshpyr.size()==14&&_gpshpyr[11]) {pyr=GSLAM::Point3d(_gpshpyr[8],_gpshpyr[9],_gpshpyr[10]);return true;}
        else if(_gpshpyr.size()==12&&_gpshpyr[9]<20) {pyr=GSLAM::Point3d(_gpshpyr[6],_gpshpyr[7],_gpshpyr[8]);return true;}
        return false;
    }     // in rad
    virtual bool    getPYRSigma(GSLAM::Point3d& pyrSigma,int idx=0)const{
        if(_gpshpyr.size()==11&&_gpshpyr[8]<20) {pyrSigma=GSLAM::Point3d(_gpshpyr[8],_gpshpyr[9],_gpshpyr[10]);return true;}
        else if(_gpshpyr.size()==14&&_gpshpyr[11]) {pyrSigma=GSLAM::Point3d(_gpshpyr[11],_gpshpyr[12],_gpshpyr[13]);return true;}
        else if(_gpshpyr.size()==12&&_gpshpyr[9]<20) {pyrSigma=GSLAM::Point3d(_gpshpyr[9],_gpshpyr[10],_gpshpyr[11]);return true;}
        return false;
    }    // in rad

    virtual int     getGPSNum()const{return (_gpshpyr.size()>=6&&_gpshpyr[3]<10)?1:0;}
    virtual bool    getGPSLLA(GSLAM::Point3d& LonLatAlt,int idx=0)const{
        if(getGPSNum()==0) return false;
        LonLatAlt=GSLAM::Point3d(_gpshpyr[0],_gpshpyr[1],_gpshpyr[2]);
        return _gpshpyr[3]<10;
    }        // WGS84 [longtitude latitude altitude]
    virtual bool    getGPSLLASigma(GSLAM::Point3d& llaSigma,int idx=0)const{
        if(_gpshpyr.size()>=6||_gpshpyr.size()==8||_gpshpyr.size()==12||_gpshpyr.size()==14)
        {llaSigma=GSLAM::Point3d(_gpshpyr[3],_gpshpyr[4],_gpshpyr[5]);return true;}
        else if(_gpshpyr.size()==7) {llaSigma=GSLAM::Point3d(_gpshpyr[3],_gpshpyr[3],_gpshpyr[4]);return true;}
        return false;
    }    // meter
    virtual bool    getGPSECEF(GSLAM::Point3d& xyz,int idx=0)const{
        GSLAM::Point3d lla;
        if(!getGPSLLA(lla)) return false;
        xyz=GSLAM::GPS<>::GPS2XYZ(lla.y,lla.x,lla.z);
        return true;
    }             // meter
    virtual bool    getHeight2Ground(GSLAM::Point2d& height,int idx=0)const{
        if(_gpshpyr.size()==14||_gpshpyr.size()==8){height=GSLAM::Point2d(_gpshpyr[6],_gpshpyr[7]);return _gpshpyr[7]<100;}
        return false;
    }    // height against ground

    virtual void call(const std::string &command, void *arg)
    {
        if("GetImagePath"==command)
        {
            if(arg) *(std::string*)arg=_imagePath;
        }
        else if("ReleaseImage"==command)
        {
            _image=GSLAM::GImage();
        }
        else if("GetGPS"==command)
        {
            if((!arg)||_gpshpyr.empty()) return;
            std::vector<double>* dest=(std::vector<double>*)arg;
            *dest=_gpshpyr;
        }
    }

    virtual LiveImageFrame& operator =(const LiveImageFrame& frame)
    {
        this->_camera = frame._camera;
        this->_cameraName = frame._cameraName;
        this->_gpshpyr = frame._gpshpyr;
        this->_imagePath = frame._imagePath;

        return *this;
    }

    std::string imagePath(){return _imagePath;}
    GSLAM::GImage& thumbnail(){return _thumbnail;}

    std::string         _imagePath;       // where image come from or where to cache
    std::string         _cameraName;
    GSLAM::GImage       _image,_thumbnail;// should be RGBA
    GSLAM::Camera       _camera;
    GSLAM::FramePtr     _slamFrame;
    std::vector<double> _gpshpyr;
    // 6 : long,lat,alt,sigmaX,sigmaY,sigmaZ
    // 8 : long,lat,alt,sigmaX,sigmaY,sigmaZ,Height,sigmaH
    // 11: long,lat,alt,sigmaH,sigmaV,pitch,yaw,roll,sigmaP,sigmaY,sigmaR
    // 12: long,lat,alt,sigmaX,sigmaY,sigmaZ,pitch,yaw,roll,sigmaP,sigmaY,sigmaR
    // 14: long,lat,alt,sigmaX,sigmaY,sigmaZ,Height,sigmaH,pitch,yaw,roll,sigmaP,sigmaR,sigmaP
};




class DatasetROS : public Dataset
{
public:
    DatasetROS(const std::string& name="")
        :_frameId(0),
         _name(name)
    {

    }

    ~DatasetROS(){
        _shouldStop=true;
        while(!_prepareThread.joinable()) GSLAM::Rate::sleep(0.01);
        _prepareThread.join();
    }

    void planSignalCallback(const std_msgs::String::ConstPtr &msg){
        std::cout<<"get msg => "<<msg->data<<std::endl;
    }

    std::string type() const {return "DatasetRTMapper";}

    virtual bool open(const string &dataset)
    {

        bool ret = false;
        Svar cfg;

        ret = cfg.ParseFile(dataset);
        if( !ret ) return false;
//        if(!ret)

        ret = openLIV_Svar(cfg,"AirsimROS");

        // set local variables & reading thread
        _frameId = 0;
        _shouldStop = false;
        _prepareThread = std::thread(&DatasetROS::run, this);
        _rosThread = std::thread(&DatasetROS::rosThreadWork,this);
//        ret = true;
        return ret;
    }


    bool isOpened()
    {
        return _camera.isValid() && _frames.size();
    }

    bool openLIV_Svar(Svar& var, const std::string& name)
    {
        _frames.clear();
        _name = name;
        string prjFile = var.GetString("Svar.ParsingFile", "");

        // load camera info
        string cameraName;
        if(var.exist("AirsimROS.Camera"))
        {
            cameraName=var.GetString("AirsimROS.Camera","");
            std::cout<<"------"<<std::endl;
        }
        else if(var.exist("Dataset.Camera"))
            cameraName=var.GetString("Dataset.Camera","");
        std::cout<<"---------------"<<std::endl;
        if(cameraName.empty()) return false;

        VecParament<double> camParas=var.get_var(cameraName+".Paraments",VecParament<double>());//内参
        GSLAM::Camera camera(camParas.data);
        if(!camera.isValid()) return false;
        _cameraName = cameraName;
        _camera = camera;
        SPtr<LiveImageFrame> frame(new LiveImageFrame(_frames.size(),0));

        _frames.push_back(frame);
        return true;
    }

    void rosThreadWork(){
        int argc = 0;
        char** argv = nullptr;
        ros::init(argc,argv,"ROS_node");
        ros::NodeHandle nh("~");
        airsim_gps_sub  =nh.subscribe<sensor_msgs::NavSatFix>("/mavros/global_position/global", 10,&DatasetROS::rosAirsimGPSCallBack,this);
        airsim_image_sub = nh.subscribe< sensor_msgs::Image>("/airsim_node/PX4/front_bottom_custom/Scene", 10,&DatasetROS::rosAirsimImageCallBack, this);
        while (ros::ok()) {
            ros::spin();
        }
    }

    void rosAirsimGPSCallBack(const sensor_msgs::NavSatFix::ConstPtr& msg){
        sensor_gps_data = *msg;
//        std::cout<<"get GPS"<<std::endl;
    }

    void rosAirsimImageCallBack(const sensor_msgs::ImageConstPtr& msg){
        static unsigned int cnt_ = 0,cnt_last_ = 0 , frame_id = 0;
        cnt_++;

        if(cnt_++ - cnt_last_ > 30||cnt_ == 1){
            cv_bridge::CvImagePtr cv_ptr;

            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cv_ptr->image.copyTo(cv_image);

            ros_time_stamp = msg->header.stamp.toSec();
            ros_image_flag = true;
            _imagePrepared.set();
//            std::cout<<"get image"<<std::endl;
            cnt_last_ = cnt_;
        }

    }



    void run()
    {
//        int argc = 0;
//        char **argv = nullptr;
//        ros::init(argc, argv, "svar_node");
//        ros::NodeHandle nh;

//        test_sub = nh.subscribe<std_msgs::String>("/SLAM/test", 10, &DatasetROS::planSignalCallback,this);

        while (!_shouldStop)
        {
            if(_preparedFrames.size()>=2)
            {
                Rate::sleep(0.001);
                continue;
            }

            auto fr = prepareFrame();
            while(!fr){
                fr = prepareFrame();
            }

            _preparedFrames.push_back(fr);
            _eventPrepared.set();
        }

        _shouldStop = true;
    }

    GSLAM::FramePtr grabFrame()
    {
        if(_preparedFrames.size())
        {
            auto ret = _preparedFrames.front();
            _preparedFrames.pop_front();
            return ret;
        }

        if( _shouldStop ) return GSLAM::FramePtr();

        _eventPrepared.wait();
        auto ret = _preparedFrames.front();
        _preparedFrames.pop_front();

        return ret;
    }

    GSLAM::FramePtr prepareFrame()
    {
        _imagePrepared.wait();
        if( ros_image_flag)
        {
            static int frame_id = 0;
            ros_image_flag = false;
            SPtr<LiveImageFrame> nf(new LiveImageFrame(frame_id++, ros_time_stamp));

            nf->_image = cv_image.clone();
            nf->_gpshpyr.clear();
//            nf->_gpshpyr.push_back(sensor_gps_data.longitude);
//            nf->_gpshpyr.push_back(sensor_gps_data.latitude);
//            nf->_gpshpyr.push_back(sensor_gps_data.altitude);
//            nf->_gpshpyr.push_back(0);
//            nf->_gpshpyr.push_back(0);
//            nf->_gpshpyr.push_back(0);
            //bug???至少11位？？
            nf->_gpshpyr = std::vector<double>({sensor_gps_data.longitude,sensor_gps_data.latitude,sensor_gps_data.altitude,
                                                10., 5.,
                                                0.0, 3.09706372944124, 0.0,
                                                0., 0.,    0.});
            nf->_cameraName = "AirsimROS";
            nf->_camera = Camera({1500,1000,750,750,750,500,0,0,0,0,0});
//            std::cout<<sensor_gps_data.longitude<<" "<<sensor_gps_data.latitude<<" "<<sensor_gps_data.altitude<<std::endl;
            return nf;
        }
        return GSLAM::FramePtr(NULL);
    }

    string           _seqTop;
    GSLAM::FrameID   _frameId;
    GSLAM::Camera    _camera;
    string           _name, _cameraName;

    std::vector<SPtr<LiveImageFrame> >   _frames;

    std::thread      _prepareThread;
    std::thread      _rosThread;
    bool             _shouldStop;
    GSLAM::Event     _eventPrepared;
    GSLAM::Event     _imagePrepared;
    std::list<GSLAM::FramePtr> _preparedFrames;

    ros::Subscriber airsim_image_sub;
    ros::Subscriber airsim_gps_sub;

    sensor_msgs::NavSatFix sensor_gps_data;
    GSLAM::GImage mimage_;
    double ros_time_stamp = 0.;
    bool ros_image_flag = false;
    cv::Mat cv_image;

};

REGISTER_DATASET(DatasetROS,ros)

#endif // end of HAS_QT

**/