/**
  A Tracker implementation tracks the map and insert keyframes to the mapper.
  **/
#ifndef TRACKER_H
#define TRACKER_H
#include <GSLAM/core/GSLAM.h>
#include <Mapper.h>

class MapFrame;

class Tracker
{
public:
    Tracker(const GSLAM::MapPtr& map,const SPtr<Mapper>& mapper)
        :_map(map),_mapper(mapper),_handle(NULL){
        if(!_mapper)
            _mapper=Mapper::create(_map,svar.GetString("Mapper","demo"));
    }

    virtual ~Tracker(){}

    virtual bool track(const SPtr<MapFrame>& frame){return false;}

    virtual void setObjectHandle(GSLAM::GObjectHandle* handle){_handle=handle;}

    virtual void draw(){}

    virtual void call(const std::string &command, void *arg=NULL) {}

    static SPtr<Tracker> create(const GSLAM::MapPtr& map,const SPtr<Mapper>& mapper,const std::string& desireType="");
protected:
    GSLAM::MapPtr           _map;
    SPtr<Mapper>            _mapper;
    GSLAM::GObjectHandle*   _handle;
};

typedef SPtr<Tracker> TrackerPtr;
typedef TrackerPtr (*funcCreateTracker)(const GSLAM::MapPtr&,const SPtr<Mapper>&);

inline TrackerPtr Tracker::create(const GSLAM::MapPtr& map,const SPtr<Mapper>& mapper,const std::string& desireType){
    auto& inst=GSLAM::SvarWithType<funcCreateTracker>::instance();
    std::cout<<inst.getStatsAsText();
    funcCreateTracker createFunc=inst.get_var(desireType,NULL);
    if(!createFunc) return TrackerPtr();
    return createFunc(map,mapper);
}

#define REGISTER_TRACKER(D,E) \
    extern "C" TrackerPtr create##D##E(const GSLAM::MapPtr& map,const SPtr<Mapper>& mapper){ return TrackerPtr(new D(map,mapper));}\
    class D##E##_Register{ \
    public: D##E##_Register(){\
    GSLAM::SvarWithType<funcCreateTracker>::instance().insert(#E,create##D##E);\
}}D##E##_instance;



#endif // TRACKER_H
