#include <LoopDetector.h>


class LoopDetectorGPS : public GSLAM::LoopDetector
{
public:
    typedef std::vector< std::pair<GSLAM::FrameID, GSLAM::Point3d> > Table;
    LoopDetectorGPS(){}

    virtual std::string type()const{return "LoopDetectorGPS";}
    virtual bool insertMapFrame(const GSLAM::FramePtr& fr)
    {
        GSLAM::WriteMutex lock(_mutex);
        GSLAM::Point3d gps;
        if(!fr->getGPSECEF(gps)) return false;
        _locations.push_back(std::make_pair(fr->id(),gps));
        return true;
    }
    virtual bool eraseMapFrame(const GSLAM::FrameID& frameId){
        GSLAM::WriteMutex lock(_mutex);
        for(Table::iterator it=_locations.begin();it!=_locations.end();it++)
        {
            if(it->first==frameId) {_locations.erase(it);return true;}
        }
        return false;
    }

    virtual bool obtainCandidates(const GSLAM::FramePtr& cur,
                                  GSLAM::LoopCandidates& candidates)
    {
        GSLAM::ReadMutex lock(_mutex);
        GSLAM::Point3d gps;
        if(!cur->getGPSECEF(gps)) return false;
        int     candiNum     =svar.GetInt("LoopDetectorGPS.CandidatesNum",-1);
        double  candiDistance=svar.GetDouble("SLAM.MaxLoopDistance",100);
        double  minLoopOverlap=1.-svar.GetDouble("SLAM.MinLoopOverlap",0.4);
        if(cur->getMedianDepth()>0&&svar.GetInt("GPS.Fitted"))
            candiDistance=cur->getCamera().UnProject(0,0).norm()
                    *2*cur->getMedianDepth()*minLoopOverlap;
        if(candiNum<=0)
        {
            candidates.reserve(_locations.size());
            for(const std::pair<GSLAM::FrameID, GSLAM::Point3d>& it:_locations)
            {
                if(cur->id()==it.first) continue;
                if(cur->getParent(it.first)) continue;
                double distance=(gps-it.second).norm();
                if(distance>candiDistance) continue;
                candidates.push_back(GSLAM::LoopCandidate(it.first,distance));
            }
            std::sort(candidates.begin(),candidates.end());
            return true;
        }
        LOG(INFO)<<candiNum<<" "<<candiDistance<<" "<<minLoopOverlap<<" "<<_locations.size();
        return false;
    }

    GSLAM::Mutex                                             _mutex;
    std::vector< std::pair<GSLAM::FrameID, GSLAM::Point3d> > _locations;
};

REGISTER_LOOPDETECTOR(LoopDetectorGPS,GPS);
