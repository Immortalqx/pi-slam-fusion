#if defined(HAS_EIGEN3)&&!defined(HAS_OPENCV)&&0

#include <vector>
#include <thread>

#include <GSLAM/core/Random.h>
#include <GSLAM/core/Estimator.h>
#include <GSLAM/core/VecParament.h>

#include "Initializer.h"
#include "../Estimator.h"

namespace zhaoyong{

class InitializerSVDEigen : public Initializer
{
public:
    typedef std::pair<int,int> Match;
    typedef std::pair<GSLAM::Point2f, GSLAM::Point2f> GMatch;

public:
    InitializerSVDEigen(){
        estimator=createEstimatorInstance();
    }

    // Computes in parallel a fundamental matrix and a homography
    // Selects a model and tries to recover the motion and the structure from motion
    virtual bool initialize(const std::vector<GMatch>& matches,
                            const GSLAM::Camera& camera,
                            GSLAM::SE3& t12,
                            std::vector<std::pair<int,GSLAM::Point3d> >& mpts)
    {
        GSLAM::SE3  Ft12;
        std::vector<std::pair<int,GSLAM::Point3d> > Fmpts;
        std::thread Fthread(InitializerSVDEigen::initializeFundamental,matches,camera,Ft12,Fmpts);

        GSLAM::SE3  Ht12;
        std::vector<std::pair<int,GSLAM::Point3d> > Hmpts;
        std::thread Hthread();

        Fthread.join();
        Hthread.join();

        if(Fmpts.size()<30&&Hmpts.size()<30)
            return false;

        if(Fmpts.size()*0.7>Hmpts.size()){
            t12=Ft12;
            mpts=Fmpts;
        }
        else{
            t12=Ht12;
            mpts=Hmpts;
        }
        return true;
    }

    void initializeFundamental(const std::vector<GMatch>& matches,
                               const GSLAM::Camera& camera,
                               GSLAM::SE3& t12,
                               std::vector<std::pair<int,GSLAM::Point3d> >& mpts)
    {
        assert(camera.valid()==false||camera.CameraType()=="Ideal");

        double F[9];
        std::vector<Point2d> points1(matches.size()),points2(matches.size());
        for(int i=0;i<matches.size();++i)
        {
            points1[i]=matches[i].first;
            points2[i]=matches[i].second;
        }
        std::vector<uchar> mask;
        if(!estimator->findFundamental(F,points1,points2,GSLAM::RANSAC,0.01,0.99,mask)) return;

        // Decompose R,T

    }

    void initializeHomography(const std::vector<GMatch>& matches,
                               const GSLAM::Camera& camera,
                               GSLAM::SE3& t12,
                               std::vector<std::pair<int,GSLAM::Point3d> >& mpts)
    {

    }

    SPtr<GSLAM::Estimator> estimator;
};

typedef zhaoyong::InitializerSVDEigen InitializerSVD;
REGISTER_INITIALIZER(InitializerSVD,eigen);
}

#endif
