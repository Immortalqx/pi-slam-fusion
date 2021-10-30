#ifndef MAPPER_H
#define MAPPER_H
#include <list>
#include <GSLAM/core/GSLAM.h>

class MapFrame;

class Mapper
{
public:
    Mapper(const GSLAM::MapPtr &map):_map(map),_handle(NULL){}
    virtual ~Mapper(){}

    void setObjectHandle(GSLAM::GObjectHandle* handle) {_handle=handle;}
    void run(){mappingThread();}

    virtual bool insertKeyFrame(const SPtr<MapFrame>& frame){return false;}
    virtual void stop(){}
    virtual void mappingThread(){}

    virtual void        call(const std::string &command, void *arg=NULL) {}

    static SPtr<Mapper> create(const GSLAM::MapPtr& map,const std::string& desireType);

    GSLAM::MapPtr           _map;
    GSLAM::GObjectHandle*   _handle;
};

typedef SPtr<Mapper> MapperPtr;
typedef MapperPtr (*funcCreateMapper)(const GSLAM::MapPtr&);

inline MapperPtr Mapper::create(const GSLAM::MapPtr& map,const std::string& desireType){
    auto& inst=GSLAM::SvarWithType<funcCreateMapper>::instance();
    funcCreateMapper createFunc=inst.get_var(desireType,NULL);
    if(!createFunc) return MapperPtr();
    return createFunc(map);
}

#define REGISTER_MAPPER(D,E) \
    extern "C" MapperPtr create##E(const GSLAM::MapPtr& map){ return MapperPtr(new D(map));}\
    class D##E##_Register{ \
    public: D##E##_Register(){\
    GSLAM::SvarWithType<funcCreateMapper>::instance().insert(#E,create##E);\
}}D##E##_instance;



#endif // MAPPER_H
