#include <GSLAM/core/GSLAM.h>

typedef GSLAM::LoopDetector       LoopDetector;
typedef SPtr<GSLAM::LoopDetector> LoopDetectorPtr;
typedef LoopDetectorPtr (*funcCreateLoopDetector)();

inline LoopDetectorPtr createLoopDetector(const std::string& desireType){
    auto& inst=GSLAM::SvarWithType<funcCreateLoopDetector>::instance();
    funcCreateLoopDetector createFunc=inst.get_var(desireType,NULL);
    if(!createFunc) return LoopDetectorPtr();
    return createFunc();
}

#define REGISTER_LOOPDETECTOR(D,E) \
    extern "C" LoopDetectorPtr create##E(){ return LoopDetectorPtr(new D());}\
    class D##E##_Register{ \
    public: D##E##_Register(){\
    GSLAM::SvarWithType<funcCreateLoopDetector>::instance().insert(#E,create##E);\
}}D##E##_instance;

