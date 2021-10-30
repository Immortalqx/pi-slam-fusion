#ifndef MAP_H
#define MAP_H
#include <GSLAM/core/GSLAM.h>

typedef GSLAM::Map       Map;
typedef SPtr<GSLAM::Map> MapPtr;
typedef MapPtr (*funcCreateMap)();

inline MapPtr createMap(const std::string& desireType){
    auto& inst=GSLAM::SvarWithType<funcCreateMap>::instance();
    funcCreateMap createFunc=inst.get_var(desireType,NULL);
    if(!createFunc) return MapPtr();
    return createFunc();
}

#define REGISTER_MAP(D,E) \
    extern "C" MapPtr create##E(){ return MapPtr(new D());}\
    class D##E##_Register{ \
    public: D##E##_Register(){\
    GSLAM::SvarWithType<funcCreateMap>::instance().insert(#E,create##E);\
}}D##E##_instance;


#endif // MAP_H
