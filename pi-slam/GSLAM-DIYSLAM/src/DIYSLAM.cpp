#include <string>
#include <list>

#include <GSLAM/core/GSLAM.h>
#include <GSLAM/core/Timer.h>
#include <GSLAM/core/Messenger.h>

#include "Tracker.h"
#include "Mapper.h"
#include "FeatureDetector.h"
#include "MapFrame.h"
#include "Map.h"
#include "LoopDetector.h"
#include "Estimator.h"


class Tracker;
class Mapper;
class FeatureDetector;
class MapFrame;
class DIYSLAM : public GSLAM::SLAM
{
public:
    DIYSLAM();
    virtual ~DIYSLAM();
    virtual std::string type()const{return "DIYSLAM";}
    virtual bool valid()const{return true;_feature2d.get()&&_tracker.get();}

    virtual bool track(GSLAM::FramePtr& frame);

    virtual void call(const std::string& command,void* arg=NULL);

    virtual void draw();

    virtual bool isDrawable()const{return false;}

    virtual bool setCallback(GSLAM::GObjectHandle *cbk);

    void trackingThread();

    void init();
    void release();

    void tryFitGPS();

    void waitFinished(void);

    SPtr<FeatureDetector>   _feature2d;
    SPtr<Tracker>           _tracker;

    std::thread             _trackingThread;

    GSLAM::GObjectHandle*        _handle;

    std::list<SPtr<MapFrame> >   _frames;
    GSLAM::Event                 _frameReadyEvent,_frameNotFullEvent;
    GSLAM::Mutex                 _slamMutex;

    bool                        _shouldStop,_firstFrame;
    GSLAM::FrameID              _frameIdStart;

    GSLAM::Publisher            _pubMapFusionMap;

    struct PlyObject
    {
        PlyObject(std::string file2save="out.ply"):_file2save(file2save){}
        ~PlyObject(){save(_file2save);}
        typedef pi::Point3f Vertex3f;
        typedef pi::Point3ub Color3b;
        typedef pi::Point3d Point3d;
        typedef pi::Point3f Point3f;

        std::string _file2save;
        std::vector<pi::Point3f>  vertices;
        std::vector<unsigned int> faces;
        std::vector<pi::Point3f>  normals;
        std::vector<pi::Point3ub> colors;
        std::vector<unsigned int> edges;

        void addPoint(Point3d pt,Color3b color=Color3b(255,255,255),pi::Point3f normal=Point3f(0,0,1))
        {
            vertices.push_back(pt);
            colors.push_back(color);
            normals.push_back(normal);
        }

        void addLine(Point3d first,Point3d second,Color3b color=Color3b(255,255,255),pi::Point3f normal=Point3f(0,0,1))
        {
            edges.push_back((uint32_t)vertices.size());
            edges.push_back((uint32_t)vertices.size()+1);
            addPoint(first,color,normal);
            addPoint(second,color,normal);
        }

        bool save(std::string filename)
        {
            if(filename.substr(filename.find_last_of('.')+1)!="ply") return false;
            std::fstream file;
            file.open(filename.c_str(),std::ios::out|std::ios::binary);
            if(!file.is_open()){
                fprintf(stderr,"\nERROR: Could not open File %s for writing!",(filename).c_str());
                return false;
            }

            uint32_t _verticesPerFace=3;
            bool binary=false;

            file << "ply";
            if(binary)file << "\nformat binary_little_endian 1.0";
            else file << "\nformat ascii 1.0";
            file << "\nelement vertex " << vertices.size();
            file << "\nproperty float32 x\nproperty float32 y\nproperty float32 z";
            if(normals.size())
                file << "\nproperty float32 nx\nproperty float32 ny\nproperty float32 nz";
            if(colors.size())
                file << "\nproperty uchar red\nproperty uchar green\nproperty uchar blue";
            if(faces.size()){
                file << "\nelement face " << faces.size()/_verticesPerFace;
                file << "\nproperty list uint8 int32 vertex_indices";
            }
            if(edges.size()){
                file << "\nelement edge " << edges.size()/2;
                file << "\nproperty int vertex1\nproperty int vertex2";
            }
            file << "\nend_header";
            if(binary) file << "\n";

            for(unsigned int i=0;i<vertices.size();i++){
                if(binary){
                    file.write((char*)(&(vertices[i])),sizeof(Vertex3f));
                }
                else file << "\n" << vertices[i].x << " " << vertices[i].y << " " << vertices[i].z;

                if(normals.size())
                {
                    if(binary){
                        file.write((char*)(&(normals[i])),sizeof(Vertex3f));
                    }
                    else file << " " << normals[i].x << " " << normals[i].y << " " << normals[i].z;
                }
                if(colors.size()){
                    if(binary){
                        file.write((char*)(&(colors[i])),sizeof(Color3b));
                    }
                    else file << " " << (int)(colors[i].x) << " " << (int)(colors[i].y) << " " << (int)(colors[i].z);
                }
            }
            for(unsigned int i=0;i<faces.size();i+=_verticesPerFace){
                if(binary){
                    file.write((char*)(&_verticesPerFace),sizeof(uchar));
                }
                else file << "\n" << (int)_verticesPerFace;
                for(unsigned int j=0;j<_verticesPerFace;j++)
                    if(binary){
                        unsigned int idx = faces[i+j];
                        file.write((char*)(&idx),sizeof(unsigned int));
                    }
                    else file << " " << (faces[i+j]);
            }
            for(unsigned int i=0;i<edges.size();i+=2){
                if(binary){
                    unsigned int idx = edges[i];
                    file.write((char*)(&idx),sizeof(unsigned int));
                    idx = edges[i+1]; file.write((char*)(&idx),sizeof(unsigned int));
                }
                else file << "\n " << edges[i] << " " << edges[i+1];
            }

            file.close();
            return true;
        }
    };

};

using std::string;
using std::vector;

DIYSLAM::DIYSLAM()
    :_firstFrame(true),_handle(NULL)
{
    init();
}

DIYSLAM::~DIYSLAM()
{
    release();
}


void DIYSLAM::init()
{
    _firstFrame = true;

#if (GSLAM_VERSION_MAJOR<<16|GSLAM_VERSION_MINOR<<8|GSLAM_VERSION_PATCH) >= 0x020402
    string logFile=svar.GetString("LogFile","");
    if(!logFile.empty()){
        using namespace GSLAM;
        AddLogSink(new LogFileSink(ERROR,logFile.c_str()));
    }
#endif

    _trackingThread=std::thread(&DIYSLAM::trackingThread,this);
    _pubMapFusionMap=GSLAM::Messenger::instance().advertise<GSLAM::Map>("fitted_map",1);
}

void DIYSLAM::release()
{
    _shouldStop=true;
    _frameReadyEvent.set();
    if(_trackingThread.joinable())
        _trackingThread.join();

    if(_tracker)
    {
        _tracker.reset();
    }

    if( _curMap )
    {
        _curMap->save(svar.GetString("MapFile2Save",""));
        _curMap->save(svar.GetString("Map2DFusionFolder",""));
    }

    //printf("DIYSLAM::release \n");
}


bool DIYSLAM::track(GSLAM::FramePtr& frame)
{
    // if frame is empty, then wait SLAM finished processed all frames
    if( !frame )
    {
        waitFinished();
        return false;
    }

    // first frame to construct feature extractor, tracker, ...
    if(_firstFrame)
    {
        setMap(createMap(svar.GetString("Map","Hash")));
        _feature2d=FeatureDetector::create(svar.GetString("SLAM.Feature","Sift"));
        _tracker  =Tracker::create(getMap(),SPtr<Mapper>(),svar.GetString("Tracker","opt"));

        if(!_feature2d) LOG(FATAL)<<"Failed to create FeatureDetector.";
        if(!_tracker)   LOG(FATAL)<<"Failed to create Tracker.";

        if(_handle) _tracker->setObjectHandle(_handle);


        if(frame->getGPSNum())
            getMap()->setLoopDetector(createLoopDetector(svar.GetString("LoopDetector","GPS")));
        else
            getMap()->setLoopDetector(createLoopDetector(svar.GetString("LoopDetector","BoW")));

        string map2load=svar.GetString("MapFile2Load","");
        _curMap->load(map2load);
        _frameIdStart=getMap()->getFid();
        _firstFrame=false;
    }

    GSLAM::GImage image=frame->getImage();
    if(image.empty()) return false;
    if(!_feature2d) return false;

    string imgFile;
    frame->call("GetImagePath",&imgFile);
    vector<double> gpsData;
    frame->call("GetGPS",&gpsData);
    SPtr<MapFrame> mpFrame(new MapFrame(frame->id()+_frameIdStart,frame->_timestamp,
                                        image,imgFile,
                                        frame->getCamera(),gpsData,frame->imageChannels()));
    {
        GSLAM::ScopedTimer tm("Tracker::FeatureExtract");
        // extract feature
        vector<GSLAM::KeyPoint> kps;
        GSLAM::GImage           features;

        (*_feature2d)(image,GSLAM::GImage(),kps,features);
        if(kps.empty() || kps.size() < 100 )
        {
            LOG(ERROR)<<"Frame "<<mpFrame->id()<<": No keypoints detected!";
            return false;
        }

        if(svar.GetInt("SLAM.RootSift",1)){

            // Remove duplicated keypoints which have same x,y
            vector<float> descriptorSum;
            vector<GSLAM::KeyPoint> kpsModified;
            vector<GSLAM::GImage> featureVector;

            std::map<int, int> sameKeyPoint;

            for (int i = 0; i < kps.size(); ++i)
            {
                int curX = kps[i].pt.x * 100;
                int curY = kps[i].pt.y * 100;

                if (sameKeyPoint.find(curX) == sameKeyPoint.end())
                    sameKeyPoint[curX] = curY;
                else
                {
                    if (sameKeyPoint[curX] == curY)
                        continue;
                }

                kpsModified.push_back(kps[i]);
                featureVector.push_back(features.row(i).clone());
            }

            GSLAM::GImage featuresModified(kpsModified.size(),128,GSLAM::GImageType<float>::Type);

            // Normalize descriptors
            for (int i = 0; i < kpsModified.size(); ++i)
            {
                float sum = 0;
                for (int j = 0; j < 128; ++j)
                {
                    sum += featureVector[i].at<float>(j);
                    featuresModified.at<float>(j, i) = featureVector[i].at<float>(j);
                }

                descriptorSum.push_back(sum);
            }

            kps.swap(kpsModified);
            features = featuresModified.clone();

            // RootSift
            for (int i = 0; i < features.rows; ++i)
            {
                for (int j = 0; j < features.cols; ++j)
                {
                    features.at<float>(j, i) = sqrt(features.at<float>(j, i)/descriptorSum[i]);
                }
            }
        }

        mpFrame->setKeyPoints(kps,features);
        GSLAM::SIM3 pose=frame->getPoseScale();
//        pose.get_rotation()=pose.get_rotation().inv();
        mpFrame->setPose(pose);
    }

    if(svar.GetInt("SLAM.TrackThread",1))
    {
        if(_frames.size()>1)
            _frameNotFullEvent.wait();
        GSLAM::WriteMutex lock(_slamMutex);
        _frames.push_back(SPtr<MapFrame>(mpFrame));
        _frameReadyEvent.set();
    }
    else
    {
        if(_tracker)
        {
            svar.GetInt("Tracker.Idle")=0;
            _tracker->track(SPtr<MapFrame>(mpFrame));
            svar.GetInt("Tracker.Idle")=1;
        }
    }
    return true;
}

void DIYSLAM::call(const std::string& command,void* arg)
{
    if("SetSvar" == command)
    {
        if(!arg) return;

        GSLAM::Svar* var=(GSLAM::Svar*)arg;
        //var->update();                                                    // FIXME: use svar.insert to pass variables
        for(auto it:var->get_data()) svar.insert(it.first,it.second);

        if( svar.GetInt("Debug.DumpSVars", 0) ) svar.dumpAllVars();
    }
    else if("Finish" == command)
    {
        release();
    }
    else if( "Release" == command )
    {
        release();
    }
    else if( "Init" == command )
    {
        init();
    }
    else if( "FitGPSAll" == command )
    {
        tryFitGPS();
    }
}

void DIYSLAM::draw()
{
    if(_curMap) _curMap->draw();
    if(_tracker) _tracker->draw();
}

bool DIYSLAM::setCallback(GSLAM::GObjectHandle *cbk)
{
    _handle=cbk;
    if(_tracker) _tracker->setObjectHandle(cbk);
//    if(_mapper)  _mapper->setObjectHandle(cbk);
    svar.GetPointer("CallBackHandle")=cbk;
    return true;
}

void DIYSLAM::trackingThread()
{
    _shouldStop=false;
    SPtr<MapFrame> frame;
    while(true)
    {
        if(_frames.empty())
        {
            if(_shouldStop) break;
            else
                _frameReadyEvent.wait();
        }

        {
            GSLAM::WriteMutex lock(_slamMutex);
            if(_frames.empty()) continue;
            frame=_frames.front();
            _frames.pop_front();
        }

        if(_tracker)
        {
            svar.GetInt("Tracker.Idle")=0;
            _tracker->track(frame);
            svar.GetInt("Tracker.Idle")=1;
        }

        _frameNotFullEvent.set();
    }
}

void DIYSLAM::tryFitGPS()
{
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

    // evaluate error
    PlyObject ply("gps.ply");
    double sumerror=0;
    for(int i=0;i<from.size();i++){
        ply.addPoint(to[i]-to[0],GSLAM::ColorType(0,255,0));
        ply.addPoint(sim3*from[i]-to[0],GSLAM::ColorType(0,0,255));
        from[i]=to[i]-sim3*from[i];
        sumerror+=from[i].dot(from[i]);
    }
    sumerror=sqrt(sumerror/from.size());
    LOG(INFO)<<"Average gps fitting error:"<<sumerror<<"m";


    for(GSLAM::FramePtr fr:frames){
        fr->setPose(sim3*fr->getPoseScale());
    }

    GSLAM::PointArray points=_curMap->getPoints();
    for(GSLAM::PointPtr pt:points){
        pt->setPose(sim3*pt->getPose());
    }

    if(_handle) _handle->handle(_curMap);
    _pubMapFusionMap.publish(_curMap);
    LOG(INFO)<<"Published map";
}

void DIYSLAM::waitFinished(void)
{
    GSLAM::Rate r(30);

    // wait tracker finished
    while( !_frames.empty() )
    {
        r.sleep();
    }

    while( 0 == svar.GetInt("Tracker.Idle", 1) )
    {
        r.sleep();
    }

    _tracker->call("Tracker.WaitFinished");
}



USE_GSLAM_PLUGIN(DIYSLAM);
