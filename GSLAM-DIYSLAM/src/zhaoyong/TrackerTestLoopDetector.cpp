#include "Tracker.h"

#include <sstream>
#include <list>

#include <GSLAM/core/Timer.h>
#include <GSLAM/core/Svar.h>
#include <GSLAM/core/Event.h>
#include <GSLAM/core/Estimator.h>
#include <GSLAM/core/VecParament.h>

#include "MapFrame.h"
#include "Matcher.h"
#include "Estimator.h"

namespace test
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


class Tracker : public ::Tracker
{
public:
    Tracker(const GSLAM::MapPtr& map,const SPtr<Mapper>& mapper);
    bool track(const SPtr<MapFrame>& frame);

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
    SPtr<Matcher>           _matcher;

    GSLAM::MutexRW          _mutex;
    std::list<FramePtr>     _localKFS;
    int                     _status;
    std::stringstream       _logger;
    double                  _lostedTime;
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
    : ::Tracker(map,mapper)
{
    _estimator=GSLAM::Estimator::create(svar.GetString("Estimator",""));
    _matcher=Matcher::create(svar.GetString("Matcher","flann"));

    if(!_estimator)
    {
        _estimator=createEstimatorInstance();
        if(!_estimator)
            LOG(FATAL)<<"No valid Estimator!";
    }
    if(!_matcher){
        LOG(FATAL)<<"No valid Matcher!";
    }
}

bool Tracker::track(const SPtr<MapFrame>& frame)
{
    ScopedLogger logger(_logger);
    _logger<<"Frame "<<frame->id()
          <<","<<frame->keyPointNum()<<" kpts";

    if(frame->keyPointNum()<300) return false;

    if(_localKFS.empty()){// first keyframe
        _map->insertMapFrame(frame);
        _localKFS.push_back(frame);
        _handle->handle(_map);
        return true;
    }

    std::vector<std::pair<int,int> > matches;
    FramePtr lastKF=_localKFS.back();
    _matcher->match4initialize(lastKF,frame,matches);

    if(matches.size()<200&&frame->timestamp()-lastKF->timestamp()>0.5){
        // new keyframe
        for(FramePtr ref:_localKFS)
        {
            SPtr<GSLAM::FrameConnection> connect(new TrackFrameConnection(matches));
            frame->addParent(ref->id(),connect);
//            ref->addChildren(frame->id(),connect);
        }

        if(_localKFS.size()>5) _localKFS.pop_front();
        _localKFS.push_back(frame);

        // do loop detection

        GSLAM::LoopCandidates candidates;

        bool ret=_map->obtainCandidates(frame,candidates);

        _map->insertMapFrame(frame);
        _handle->handle(frame);
        frame->clearParents();

        if(!ret) return true;

        float minScore=svar.GetInt("Loop.MinScore",60);
        float maxScore=-1;
        for(int i=candidates.size()-1;i>=0;i--)
        {
            GSLAM::LoopCandidate can=candidates[i];
            if(maxScore<0) maxScore=can.score;
            if(can.score<maxScore*0.5) break;
            if(can.score<minScore) break;
            FramePtr ref=_map->getFrame(can.frameId);
            std::vector<std::pair<int,int> > matches;
            _matcher->match4initialize(ref,frame,matches);
            if(matches.size()<50) continue;
            SPtr<GSLAM::FrameConnection> connect(new TrackFrameConnection(matches));
            ref->addChildren(frame->id(),connect);
            frame->addParent(ref->id(),connect);
            LOG(INFO)<<"LoopFound between "<<ref->id()<<" "<<frame->id();
        }
    }

    _handle->handle(new GSLAM::CurrentFrameEvent(frame));
    _logger<<",Pose"<<frame->getPose();

    return true;
}


}

typedef test::Tracker TrackerTestLoopDetector;
REGISTER_TRACKER(TrackerTestLoopDetector,testLoopDetector);
