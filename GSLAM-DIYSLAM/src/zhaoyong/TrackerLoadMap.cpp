#include "Tracker.h"

namespace zhaoyong
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

class Tracker : public ::Tracker
{
public:
    Tracker(const GSLAM::MapPtr& map,const SPtr<Mapper>& mapper): ::Tracker(map,mapper)
    {
    }

    bool track(const SPtr<MapFrame>& frame){

        return false;
    }

    virtual void setObjectHandle(GSLAM::GObjectHandle* handle){
        _handle=handle;
        if(handle&&_map){
            if(!_map->load(svar.GetString("MapFile2Load","map.gmap"))) return ;
            handle->handle(_map);
        }
    }
};

REGISTER_TRACKER(Tracker,loadmap);

}

