#include <GSLAM/core/GSLAM.h>

class Relocalizer
{
public:
    virtual ~Relocalizer(){}

    virtual bool relocalize(const GSLAM::MapPtr& map,const GSLAM::FramePtr& frame){return false;}

    static SPtr<Relocalizer> create(const std::string& implName);
};

typedef SPtr<Relocalizer> RelocalizerPtr;
typedef RelocalizerPtr (*funcCreateRelocalizer)();

inline RelocalizerPtr Relocalizer::create(const std::string& desireType){
    auto& inst=GSLAM::SvarWithType<funcCreateRelocalizer>::instance();
    funcCreateRelocalizer createFunc=inst.get_var(desireType,NULL);
    if(!createFunc) return RelocalizerPtr();
    return createFunc();
}

#define REGISTER_RELOCALIZER(D,E) \
    extern "C" RelocalizerPtr create##E(){ return RelocalizerPtr(new D());}\
    class D##E##_Register{ \
    public: D##E##_Register(){\
    GSLAM::SvarWithType<funcCreateRelocalizer>::instance().insert(#E,create##E);\
}}D##E##_instance;
