#include "Initializer.h"
#include "optimizerG2O/Optimizer.h"

class InitializerOpt : public Initializer
{
public:
    virtual bool initialize(const std::vector<std::pair<GSLAM::Point2f,GSLAM::Point2f> >& matches,
                            const GSLAM::Camera& camera,
                            GSLAM::SE3& t12,
                            std::vector<std::pair<int,GSLAM::Point3d> >& mpts);

    SPtr<GSLAM::Optimizer> _optimizer;
};



bool InitializerOpt::initialize(const std::vector<std::pair<GSLAM::Point2f,GSLAM::Point2f> >& matches,
                        const GSLAM::Camera& camera,
                        GSLAM::SE3& t12,
                        std::vector<std::pair<int,GSLAM::Point3d> >& mpts)
{
    if(!_optimizer){
        _optimizer=createOptimizerInstance();//GSLAM::Optimizer::create();
        if(!_optimizer)
        {
            LOG(FATAL)<<"No valid Optimizer!";
            return false;
        }
    }

    std::vector<std::pair<GSLAM::CameraAnchor,GSLAM::CameraAnchor> > anchors;
    std::vector<GSLAM::IdepthEstimation> idepths(matches.size(),GSLAM::Point2d(1,-1));

    anchors.reserve(matches.size());

    if(camera.isValid())
    {
        for(auto m:matches)
        {
            anchors.push_back(std::make_pair(camera.UnProject(m.first),
                                             camera.UnProject(m.second)));
        }
    }
    else
    {
        for(auto m:matches)
            anchors.push_back(std::make_pair(GSLAM::Point3d(m.first.x,m.first.y,1),
                                             GSLAM::Point3d(m.second.x,m.second.y,1)));
    }

    int enoughBaselineCount=0;
    for(auto m:anchors)
    {
        if((m.second-m.first).norm()>0.05) enoughBaselineCount++;
    }
    if(enoughBaselineCount*5<anchors.size())
    {
        LOG(WARNING)<<"No enough baseline count.";
        return false;
    }

    if(!_optimizer->optimizePose(anchors,idepths,t12))
    {
        LOG(WARNING)<<"Failed optimize.";
        return false;
    }

    if(t12.get_translation().norm()<0.03)
    {
        LOG(WARNING)<<"No enough translation.";
        return false;
    }

    GSLAM::SE3 t21=t12.inverse();
    mpts.reserve(anchors.size());
    for(int i=0;i<idepths.size();i++)
    {
        if(idepths[i].x<=0.1||idepths[i].x>20) continue;

        GSLAM::Point3d pt=anchors[i].first/idepths[i].x;
        GSLAM::Point3d pt2=t21*pt;
        if(pt2.z<=0.001) continue;
        pt2=pt2/pt2.z;

        pt2=pt2-anchors[i].second;
        if(pt2.dot(pt2)>1e-5) continue;

        mpts.push_back(std::make_pair(i,pt));
    }

    if(mpts.size()>50&&mpts.size()*2>idepths.size()) return true;
    else
    {
        LOG(WARNING) << "No enough mappoints.";
        return false;
    }
}

REGISTER_INITIALIZER(InitializerOpt,opt);
