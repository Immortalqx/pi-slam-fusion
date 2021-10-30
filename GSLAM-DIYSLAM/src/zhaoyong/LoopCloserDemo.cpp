#include "LoopCloser.h"
#include "Matcher.h"
#include "MapFrame.h"

#include <unordered_map>

#include <GSLAM/core/Optimizer.h>
#include <GSLAM/core/Estimator.h>
#include <GSLAM/core/Timer.h>

#include "optimizerG2O/Optimizer.h"

namespace zhaoyong{
using namespace std;

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


struct PlyObject
{
    PlyObject(string file2save="out.ply"):_file2save(file2save){}
    ~PlyObject(){save(_file2save);}
    typedef pi::Point3f Vertex3f;
    typedef pi::Point3ub Color3b;

    std::string _file2save;
    std::vector<pi::Point3f>  vertices;
    std::vector<unsigned int> faces;
    std::vector<pi::Point3f>  normals;
    std::vector<pi::Point3ub> colors;
    std::vector<unsigned int> edges;

    void addPoint(Point3d pt,Color3b color=Color3b(255,255,255),pi::Point3f normal=GSLAM::Point3f(0,0,1))
    {
        vertices.push_back(pt);
        colors.push_back(color);
        normals.push_back(normal);
    }

    void addLine(Point3d first,Point3d second,Color3b color=Color3b(255,255,255),pi::Point3f normal=GSLAM::Point3f(0,0,1))
    {
        edges.push_back(vertices.size());
        edges.push_back(vertices.size()+1);
        addPoint(first,color,normal);
        addPoint(second,color,normal);
    }

    bool save(string filename)
    {
        if(filename.substr(filename.find_last_of('.')+1)!="ply") return false;
        std::fstream file;
        file.open(filename.c_str(),std::ios::out|std::ios::binary);
        if(!file.is_open()){
            fprintf(stderr,"\nERROR: Could not open File %s for writing!",(filename).c_str());
            return false;
        }

        int _verticesPerFace=3;
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

class LoopCloserSE3Graph : public LoopCloser
{
public:
    LoopCloserSE3Graph(const GSLAM::MapPtr &map)
        : LoopCloser(map){
        _matcher=Matcher::create(svar.GetString("LoopCloser.Matcher","flann"));

//        _optimizer=GSLAM::Optimizer::create();
        if(!_optimizer)
            _optimizer=createOptimizerInstance();

        if(!_optimizer)
        {
            LOG(FATAL)<<"No valid Optimizer!";
        }
        if(!_matcher){
            LOG(FATAL)<<"No valid Matcher!";
        }
//        _thread=std::thread(&LoopCloserSE3Graph::mappingThread,this);
    }
    ~LoopCloserSE3Graph(){
        stop();
//        _thread.join();
    }
    virtual bool insertKeyFrame(const SPtr<MapFrame>& frame);
    virtual void stop(){_shouldStop=true;}
    virtual void mappingThread();

private:
    bool closeLoop();

    class ScopedLogger
    {
    public:
        ScopedLogger(std::stringstream& sst):_sst(sst),_verbose(svar.GetInt("SLAM.Verbose",2)){
            _sst.str("");
        }
        ~ScopedLogger(){
            if(_sst.str().size()&&(_verbose&2))
                LOG(INFO)<<_sst.str();
        }

        std::stringstream& _sst;
        int                _verbose;
    };


    SPtr<GSLAM::Optimizer>     _optimizer;
    SPtr<MapFrame>             _curFrame;
    SPtr<Matcher>              _matcher;
    GSLAM::Event               _insertionEnabled,_newFrameInserted;

    bool                       _shouldStop;
    std::stringstream          _logger;
    std::thread                _thread;
};

bool LoopCloserSE3Graph::insertKeyFrame(const SPtr<MapFrame>& frame){
//    int onlineMode=svar.GetInt("SLAM.isOnline");
//    if(onlineMode)
//    {
//        if(_curFrame)
//        {
//            _abordBundle=true;
//            if(onlineMode>1)
//            {
//                return false;
//            }
//            // full bundle and insert kf
//            GSLAM::ScopedTimer st("LoopCloser::insertWaiting");
//            _insertionEnabled.wait();
//        }

//        _abordBundle=false;
//        _curFrame=frame;
//        _newFrameInserted.set();
//        return true;
//    }
//    else
    {
        // offline dataset, wait keyframe handled
        _curFrame=frame;
        closeLoop();
//        _newFrameInserted.set();
//        _insertionEnabled.wait();
    }
    return true;
}


void LoopCloserSE3Graph::mappingThread()
{
    _shouldStop=false;
    _insertionEnabled.set();
    int& loopCloserIdle=svar.GetInt("LoopCloser.Idle");
    while(true)
    {
        if(!_curFrame)
        {
            if(_shouldStop) break;
            _insertionEnabled.set();
            loopCloserIdle=1;
            GSLAM::Rate::sleep(0.001);
            continue;
        }
        loopCloserIdle=0;
        closeLoop();
        _insertionEnabled.set();
        loopCloserIdle=1;
    }
}

bool LoopCloserSE3Graph::closeLoop(){
    ScopedLogger lg(_logger);
    GSLAM::LoopCandidates candidates;
    if(!_map->obtainCandidates(_curFrame,candidates)) return false;
    if(candidates.empty()) return false;
    _logger<<"LoopCloserSE3Graph:"<<_curFrame->id()<<",Candidates:"<<candidates.size();

    auto frame=_map->getFrame(candidates.front().frameId);

    SPtr<MapFrame> refFrame=std::dynamic_pointer_cast<MapFrame>(frame);
    if(!refFrame) return  false;
    _logger<<",Ref:"<<refFrame->id();
    vector<pair<int,int> > matches;
    bool bOK=_matcher->match4initialize(refFrame,_curFrame,matches);
    if(!bOK)
        return false;
    _logger<<",Matches:"<<matches.size();

    vector<pair<int,GSLAM::PointPtr> > observations;
    observations.reserve(matches.size());
    for(pair<int,int>& m : matches)
    {
        GSLAM::PointID pid=refFrame->getKeyPointObserve(m.first);
        if(!pid) continue;
        observations.push_back(std::make_pair(m.second,refFrame->keyPoint(m.first)._mapPoint));
    }
    _logger<<",Obs"<<observations.size();

    if( observations.size() < 30 )
    {
        DLOG(ERROR)<<"Observation number "<<observations.size()<<" < "<<30;
        return false;
    }

    GSLAM::FramePtr ref=refFrame;
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
    auto _estimator=GSLAM::Estimator::create();
    bOK=_estimator->findPnPRansac(local2cur,objectPoints,imagePoints,GSLAM::Camera(),
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


    GSLAM::ScopedTimer tm("Mapper::closeLoop");
    // Graph optimization
    GSLAM::BundleGraph graph;

    graph.keyframes.reserve(_map->frameNum());
    GSLAM::FrameArray frames;
    std::unordered_map<FrameID,size_t> kfIdxs;
    _map->getFrames(frames);

    for(GSLAM::FramePtr fr:frames)
    {
        kfIdxs.insert(std::make_pair(fr->id(),graph.keyframes.size()));
        graph.keyframes.push_back({fr->getPoseScale(),GSLAM::UPDATE_KF_SE3});
    }

    {
        graph.keyframes[kfIdxs[refFrame->id()]].dof=GSLAM::UPDATE_KF_NONE;
        std::map<GSLAM::FrameID,SPtr<GSLAM::FrameConnection> > parents;
        refFrame->getParents(parents);
        for(auto parent:parents){
            graph.keyframes[kfIdxs[parent.first]].dof=GSLAM::UPDATE_KF_NONE;
        }
    }

    for(FrameID i=0;i<frames.size();i++)
    {
        GSLAM::FramePtr fr=frames[i];
        std::map<GSLAM::FrameID,SPtr<GSLAM::FrameConnection> > parents;
        fr->getParents(parents);
        for(auto parent:parents){
            GSLAM::FrameID parentId=kfIdxs[parent.first];
            GSLAM::FramePtr ref=frames[parentId];
            if(i==parentId) continue;
            graph.se3Graph.push_back(GSLAM::SE3Edge({i,parentId,fr->getPose().inverse()*ref->getPose(),NULL}));
        }
    }

    _logger<<",Graph:"<<graph.keyframes.size()<<"-"<<graph.se3Graph.size();
    graph.se3Graph.push_back(GSLAM::SE3Edge({kfIdxs[refFrame->id()],
                                               kfIdxs[_curFrame->id()],
                                               local2cur.inverse(),NULL}));

    {
        PlyObject ply;
        for(auto kf:graph.keyframes){
            ply.addPoint(kf.estimation.get_translation(),GSLAM::ColorType(255,0,0));
        }
        for(GSLAM::SE3Edge edge:graph.se3Graph){
            if(edge.firstId==edge.secondId)
                continue;
            auto pt1=frames[edge.firstId]->getPose().get_translation();
            auto pt2=frames[edge.secondId]->getPose().get_translation();
            ply.addLine(pt1,pt2);
        }
        _optimizer->optimize(graph);

        for(auto kf:graph.keyframes){
            ply.addPoint(kf.estimation.get_translation(),GSLAM::ColorType(0,255,0));
        }
    }
    for(int i=0;i<graph.keyframes.size();i++){
        auto pose=graph.keyframes[i].estimation;
        graph.keyframes[i].estimation=pose*frames[i]->getPose().inverse();
        frames[i]->setPose(pose);
    }

    GSLAM::PointArray points;
    _map->getPoints(points);
    for(GSLAM::PointPtr pt:points){
        size_t idx=kfIdxs[pt->refKeyframeID()];
        pt->setPose(graph.keyframes[idx].estimation*pt->getPose());
    }

    for(int idx:inliers)
    {
        pair<int,GSLAM::PointPtr>& obs=observations[idx];
        if(!obs.second) continue;
        if(_curFrame->getKeyPointObserve(obs.first)) continue;
        _curFrame->addObservation(obs.second,obs.first);
    }

//    _curFrame->setPose(local2world*local2cur.inverse());
    SPtr<GSLAM::FrameConnection> frCon;
    _curFrame->addParent(ref->id(),frCon);
    std::map<GSLAM::FrameID,SPtr<GSLAM::FrameConnection> > parents;
    refFrame->getParents(parents);
    for(auto parent:parents){
        _curFrame->addParent(parent.first,frCon);
    }
    refFrame->getChildren(parents);
    for(auto parent:parents){
        _curFrame->addParent(parent.first,frCon);
    }
    return true;
}

REGISTER_LOOPCLOSER(LoopCloserSE3Graph,se3graph);

}


