#include "Matcher.h"
#include "GSLAM/core/Timer.h"
#include "GSLAM/core/Vocabulary.h"
#include "GSLAM/core/Estimator.h"
#include "GSLAM/core/Event.h"
#include "Estimator.h"

#include <opencv2/imgproc/imgproc.hpp>

using namespace std;

class MatcherMultiH : public Matcher
{
public:
    MatcherMultiH(){
        _estimator=GSLAM::Estimator::create();
        if(!_estimator)
        {
            _estimator=createEstimatorInstance();
            if(!_estimator)
                LOG(ERROR)<<"MatcherBoW failed to create Estimator.";
        }
    }

    virtual bool match4initialize(const GSLAM::FramePtr& lastKF,const GSLAM::FramePtr& curFrame,
                                  std::vector<std::pair<int,int> >& matches)const;

    virtual bool match4triangulation(const GSLAM::FramePtr& ref,const GSLAM::FramePtr& cur,
                                     std::vector<std::pair<int,int> >& matches)const
    {
        GSLAM::ScopedTimer mTimer("MatcherMultiH::match4triangulation");

        // compute fundamental matrix
        GSLAM::SE3 cur2ref=ref->getPose().inverse()*cur->getPose();
        GSLAM::SO3 so3    =cur2ref.get_rotation().inv();
        GSLAM::Point3d   t=cur2ref.get_translation();

        GSLAM::FeatureVector featvec1,featvec2;
        if(!ref->getFeatureVector(featvec1)) return false;
        if(!cur->getFeatureVector(featvec2)) return false;

        GSLAM::FeatureVector::iterator it1  = featvec1.begin();
        GSLAM::FeatureVector::iterator it2  = featvec2.begin();
        GSLAM::FeatureVector::iterator end1 = featvec1.end();
        GSLAM::FeatureVector::iterator end2 = featvec2.end();

        float maxDistance=-1;
        vector<bool> matched(cur->keyPointNum(),false);
        while(it1 != end1 && it2 != end2)
        {
            if(it1->first == it2->first)
            {
                vector<unsigned int> ids1 = it1->second;
                vector<unsigned int> ids2 = it2->second;

                for(unsigned int i1:ids1)
                {
                    if(ref->getKeyPointObserve(i1)) continue;
                    vector<std::pair<float,int> > disIdxs;
                    disIdxs.reserve(ids2.size());
                    GSLAM::GImage des=ref->getDescriptor(i1);

                    if(maxDistance<0)
                    {
                        if(des.type()==GSLAM::GImageType<float>::Type&&des.cols==128) maxDistance=0.2;//SIFT
                        else if(des.type()==GSLAM::GImageType<uchar>::Type&&des.cols==32) maxDistance=50;//ORB

                        assert(maxDistance>0);
                    }

                    for(unsigned int i2:ids2)
                    {
                        if(matched[i2]) continue;
                        if(cur->getKeyPointObserve(i2)) continue;
                        auto distance=GSLAM::Vocabulary::distance(des,cur->getDescriptor(i2));
                        if(distance>maxDistance)  continue;
                        disIdxs.push_back(std::pair<float,int>(distance,i2));
                    }
                    if(disIdxs.empty()) continue;

                    std::sort(disIdxs.begin(),disIdxs.end());

                    float bestDistance=disIdxs.front().first;
                    float distanceThreshold=bestDistance*2;

                    GSLAM::Point2f refkp;
                    if(!ref->getKeyPoint(i1,refkp)) continue;
                    GSLAM::Point3d refpt=ref->getCamera().UnProject(refkp.x,refkp.y);
                    GSLAM::Point3d tcrosspl(t.y-t.z*refpt.y,
                                            t.z*refpt.x-t.x,
                                            t.x*refpt.y-t.y*refpt.x);
                    GSLAM::Point3d line=so3*tcrosspl;
                    double         invabsqrt=1./sqrt(line.x*line.x+line.y*line.y);
                    for(auto &m:disIdxs)
                    {
                        if(m.first>distanceThreshold)
                            break;
                        GSLAM::Point2f curkp;
                        cur->getKeyPoint(m.second,curkp);
                        GSLAM::Point3d curpt=cur->getCamera().UnProject(curkp.x,curkp.y);
                        double distance=fabs(line.x*curpt.x+line.y*curpt.y+line.z)*invabsqrt;
                        if(distance>0.02)
                        {
                            continue;
                        }
                        matches.push_back(std::pair<int,int>(i1,m.second));
                        matched[m.second]=true;
                        break;
                    }

                }

                it1++;
                it2++;
            }
            else if(it1->first < it2->first)
            {
                it1 = featvec1.lower_bound(it2->first);
            }
            else
            {
                it2 = featvec2.lower_bound(it1->first);
            }
        }

        return true;
    }

    virtual bool findMatchWindow(const GSLAM::GImage& des,const GSLAM::FramePtr& fr,
                                 const float& x,const float& y,const float& r,
                                 int& idx,bool discardMapPoints=true)const
    {
        GSLAM::ScopedTimer mTimer("MatcherMultiH::findMatchWindow");

        std::vector<size_t> candidates=fr->getFeaturesInArea(x,y,r);
        if(candidates.empty()) return false;

        float minDistance=1e8,minDistance2=1e8;
        float maxDistance=-1;
        if(maxDistance<0)
        {
            if(des.type()==GSLAM::GImageType<float>::Type&&des.cols==128) maxDistance=0.2;//SIFT
            else if(des.type()==GSLAM::GImageType<uchar>::Type&&des.cols==32) maxDistance=80;//ORB

            assert(maxDistance>0);
        }
        for(size_t& i:candidates)
        {
            if(discardMapPoints&&fr->getKeyPointObserve(i)) continue;
            GSLAM::GImage des1=fr->getDescriptor(i);

            float dis=GSLAM::Vocabulary::distance(des1,des);
            if(dis<minDistance)
            {
                minDistance2=minDistance;
                minDistance=dis;
                idx=i;
            }
            else if(dis<minDistance2){
                minDistance2=minDistance;
            }
        }

//        if(minDistance>minDistance2*0.6) return false;

        if(minDistance<maxDistance) return true;
        else return false;
    }

    virtual bool findMatchEpipolarLine(const GSLAM::GImage&  des , const GSLAM::FramePtr& fr ,
                                       const GSLAM::Point3d& line, const float& r ,
                                       int& idx, bool discardMapPoints=true){



        return false;
    }

    virtual void transform(const vector<double> H, const GSLAM::Point2f& src,GSLAM::Point2f& dst)const{
        double x=H[0]*src.x+H[1]*src.y+H[2];
        double y=H[3]*src.x+H[4]*src.y+H[5];
        double z=H[6]*src.x+H[7]*src.y+H[8];
        dst=GSLAM::Point2f(x/z,y/z);
    }

    virtual double distance(double* F, const GSLAM::Point2d& src,const GSLAM::Point2d& dst)const{
        // src^T*F*dst=0
        double x=F[0]*src.x+F[1]*src.y+F[2];
        double y=F[3]*src.x+F[4]*src.y+F[5];
        double z=F[6]*src.x+F[7]*src.y+F[8];
        return fabs(dst.x*x+dst.y*y+z);
    }

    SPtr<GSLAM::Estimator> _estimator;
};

bool MatcherMultiH::match4initialize(const GSLAM::FramePtr& lastKF,const GSLAM::FramePtr& curFrame,
                                  std::vector<std::pair<int,int> >& matches)const
{
    GSLAM::ScopedTimer mTimer("MatcherMultiH::match4initialize");

    // 1. Obtain initial matches and inliers
    GSLAM::FeatureVector featvec1,featvec2;
    if(!lastKF->getFeatureVector(featvec1)) return false;
    if(!curFrame->getFeatureVector(featvec2)) return false;

    GSLAM::FeatureVector::iterator it1  = featvec1.begin();
    GSLAM::FeatureVector::iterator it2  = featvec2.begin();
    GSLAM::FeatureVector::iterator end1 = featvec1.end();
    GSLAM::FeatureVector::iterator end2 = featvec2.end();

    float maxDistance=-1;
    bool crossCheck=svar.GetInt("MatcherBoW.CrossCheck",1);
    while(it1 != end1 && it2 != end2)
    {
        if(it1->first == it2->first)
        {
            vector<unsigned int> ids1 = it1->second;
            vector<unsigned int> ids2  = it2->second;

            for(unsigned int i1:ids1)
            {
                float bestDistance=std::numeric_limits<float>::max();
                unsigned int  bestIdx=0;
                GSLAM::GImage des=lastKF->getDescriptor(i1);
                if(maxDistance<0)
                {
                    if(des.type()==GSLAM::GImageType<float>::Type&&des.cols==128) maxDistance=0.2;//SIFT
                    else if(des.type()==GSLAM::GImageType<uchar>::Type&&des.cols==32) maxDistance=50;//ORB

                    assert(maxDistance>0);
                }
                for(unsigned int i2:ids2)
                {
//                    if(curFrame->getKeyPointObserve(i2)) continue;
                    auto distance=GSLAM::Vocabulary::distance(des,curFrame->getDescriptor(i2));
                    if(distance<bestDistance){
                        bestDistance=distance;
                        bestIdx=i2;
                    }
                }
                bool isMin=true;
                if(crossCheck)
                {
                    des=curFrame->getDescriptor(bestIdx);
                    for(unsigned int j:ids1)
                    {
                        if(j==i1) continue;
                        auto distance=GSLAM::Vocabulary::distance(lastKF->getDescriptor(j),des);
                        if(distance<bestDistance) {isMin=false;break;}
                    }
                }
                if(isMin&&bestDistance<maxDistance) matches.push_back(std::make_pair(i1,bestIdx));
            }

            it1++;
            it2++;
        }
        else if(it1->first < it2->first)
        {
            it1 = featvec1.lower_bound(it2->first);
        }
        else
        {
            it2 = featvec2.lower_bound(it1->first);
        }
    }

    // filter with keypoint angle
    if(svar.GetInt("MatcherBoW.FilterAngle",1))
    {
        std::vector<int> bins[30];

        for(int i=0;i<30;i++)
            bins[i].reserve(matches.size());

        for(int i=0;i<matches.size();i++){
            GSLAM::KeyPoint kp1,kp2;
            if(!lastKF->getKeyPoint(matches[i].first,kp1)) continue;
            if(!curFrame->getKeyPoint(matches[i].second,kp2)) continue;
            float angle=kp1.angle-kp2.angle;
            if(angle<0) angle+=360;
            bins[int(angle/30)].push_back(i);
        }

        int max1=0;
        int max2=0;
        int max3=0;
        int ind1=-1;
        int ind2=-1;
        int ind3=-1;

        for(int i=0; i<30; i++)
        {
            const int s = bins[i].size();
            if(s>max1)
            {
                max3=max2;
                max2=max1;
                max1=s;
                ind3=ind2;
                ind2=ind1;
                ind1=i;
            }
            else if(s>max2)
            {
                max3=max2;
                max2=s;
                ind3=ind2;
                ind2=i;
            }
            else if(s>max3)
            {
                max3=s;
                ind3=i;
            }
        }

        if(max2<0.1f*(float)max1)
        {
            ind2=-1;
            ind3=-1;
        }
        else if(max3<0.1f*(float)max1)
        {
            ind3=-1;
        }
        int curSize=0;
        for(int i=0; i<30; i++)
        {
            if(i==ind1 || i==ind2 || i==ind3)
            for(int j:bins[i])
            {
                matches[curSize++]=matches[j];
            }
        }
        DLOG(INFO)<<"FilterAngle:"<<matches.size()<<"->"<<curSize;
        matches.resize(curSize);
    }

    int numThreshold=std::max(50,int(curFrame->keyPointNum()*0.03));

    if(matches.size()<numThreshold)
    {
        DLOG(ERROR)<<"matches number "<<matches.size()<<"<"<<numThreshold;
#ifdef HAS_OPENCV
        GSLAM::GObjectHandle* handle=static_cast<GSLAM::GObjectHandle*>(svar.GetPointer("CallBackHandle"));
        if(svar.GetInt("DebugMatcher")&&handle)
        {
            cv::Mat img=curFrame->getImage().clone();
            for(size_t i=0;i<matches.size();i++)
            {
                auto m=matches[i];
                GSLAM::Point2f pt1,pt2;
                lastKF->getKeyPoint(m.first,pt1);
                curFrame->getKeyPoint(m.second,pt2);
                cv::line(img,cv::Point2f(pt1.x,pt1.y),cv::Point2f(pt2.x,pt2.y),cv::Scalar(0,255,0),2);
            }
            handle->handle(new GSLAM::DebugImageEvent(img,"DebugMatcher"));
        }
#endif
        return false;
    }

    // filt with fundamental
    vector<u_char>         mask;
    GSLAM::Camera  cam1=lastKF->getCamera();
    GSLAM::Camera  cam2=curFrame->getCamera();
    double F[9];
    vector<GSLAM::Point2d> points1,points2;
    for(auto& m:matches)
    {
        GSLAM::Point2f pt;
        if(!lastKF->getKeyPoint(m.first,pt)) LOG(FATAL)<<"MapFrame::getKeyPoint() not implemented!";
        GSLAM::Point3d p3d=cam1.UnProject(pt.x,pt.y);
        points1.push_back(GSLAM::Point2d(p3d.x,p3d.y));

        if(!curFrame->getKeyPoint(m.second,pt)) LOG(FATAL)<<"MapFrame::getKeyPoint() not implemented!";
        p3d=cam2.UnProject(pt.x,pt.y);
        points2.push_back(GSLAM::Point2d(p3d.x,p3d.y));
    }

    if(!_estimator->findFundamental(F,points1,points2,GSLAM::RANSAC,0.01,0.99,&mask)) return false;

#ifdef HAS_OPENCV
    GSLAM::GObjectHandle* handle=static_cast<GSLAM::GObjectHandle*>(svar.GetPointer("CallBackHandle"));
    if(svar.GetInt("DebugMatcher")&&handle)
    {
        cv::Mat img=curFrame->getImage().clone();
        for(size_t i=0;i<matches.size();i++)
        {
            auto m=matches[i];
            GSLAM::Point2f pt1,pt2;
            lastKF->getKeyPoint(m.first,pt1);
            curFrame->getKeyPoint(m.second,pt2);
            cv::line(img,cv::Point2f(pt1.x,pt1.y),cv::Point2f(pt2.x,pt2.y),mask[i]?cv::Scalar(255,0,0):cv::Scalar(0,255,0),2);
        }
        handle->handle(new GSLAM::DebugImageEvent(img,"DebugMatcher"));
    }
#endif

    int curSize=0;
    for(size_t i=0;i<matches.size();i++){
        if(mask[i])
            matches[curSize++]=matches[i];
    }
    matches.resize(curSize);

    numThreshold=std::max(50,int(mask.size()*0.1));
    if(matches.size()<numThreshold)
    {
        DLOG(ERROR)<<"inlier number "<<matches.size()<<"<"<<numThreshold;
        return false;
    }

    // find multi H
    std::vector<std::vector<double> > Hs;
    std::vector<std::pair<int,int> >  matchesLeft=matches;

    while(matchesLeft.size()>8){
        std::vector<GSLAM::Point2d> srcPoints,dstPoints;
        srcPoints.reserve(matchesLeft.size());
        dstPoints.reserve(matchesLeft.size());
        std::vector<double> H(9);
        std::vector<uchar>  mask;
        for(auto& m:matchesLeft)
        {
            GSLAM::Point2f pt;
            if(!lastKF->getKeyPoint(m.first,pt)) LOG(FATAL)<<"MapFrame::getKeyPoint() not implemented!";
            GSLAM::Point3d p3d=cam1.UnProject(pt.x,pt.y);
            srcPoints.push_back(GSLAM::Point2d(p3d.x,p3d.y));

            if(!curFrame->getKeyPoint(m.second,pt)) LOG(FATAL)<<"MapFrame::getKeyPoint() not implemented!";
            p3d=cam2.UnProject(pt.x,pt.y);
            dstPoints.push_back(GSLAM::Point2d(p3d.x,p3d.y));
        }
        if(!_estimator->findHomography(H.data(),srcPoints,dstPoints,GSLAM::RANSAC,0.01,&mask)) break;

        int curSize=0;
        for(size_t i=0;i<matchesLeft.size();i++){
            if(!mask[i])
                matchesLeft[curSize++]=matchesLeft[i];
        }
        matchesLeft.resize(curSize);
        Hs.push_back(H);
    }

    if(Hs.empty()) return matches.size()>=numThreshold;

    std::vector<uchar> matchedMask(lastKF->keyPointNum(),0);
    for(auto& m:matches) matchedMask[m.first]=255;

    std::vector<std::pair<int,int> > matchesH;
    for(int i=0;i<lastKF->keyPointNum();i++){
        if(matchedMask[i]) continue;
        GSLAM::Point2f kp;
        if(!lastKF->getKeyPoint(i,kp)) continue;
        GSLAM::Point3d up=cam1.UnProject(kp);
        GSLAM::Point2f srcPt(up.x,up.y);
        GSLAM::Point2f dstPt,bestDstPt;
        double  minDistance=1e10;
        for(int j=0;j<Hs.size();j++)
        {
            transform(Hs[j],srcPt,dstPt);
            double dis=distance(F,srcPt,dstPt);
            if(dis<minDistance) {
                minDistance=dis;
                bestDstPt=dstPt;
            }
        }
        if(minDistance>0.01) continue;

        GSLAM::Point2d pt=cam2.Project(bestDstPt.x,bestDstPt.y,1);
        int idx;
        if(!findMatchWindow(lastKF->getDescriptor(i),curFrame,
                            pt.x,pt.y,cam2.width()/64,idx)) continue;

        matchesH.push_back(std::make_pair(i,idx));
    }


#ifdef HAS_OPENCV
    if(svar.GetInt("DebugMatcher")&&handle)
    {
        cv::Mat img=curFrame->getImage().clone();
        for(size_t i=0;i<matches.size();i++)
        {
            auto m=matches[i];
            GSLAM::Point2f pt1,pt2;
            lastKF->getKeyPoint(m.first,pt1);
            curFrame->getKeyPoint(m.second,pt2);
            cv::line(img,cv::Point2f(pt1.x,pt1.y),cv::Point2f(pt2.x,pt2.y),cv::Scalar(0,0,255),cam2.width()/500);
        }
        for(size_t i=0;i<matchesH.size();i++)
        {
            auto m=matchesH[i];
            GSLAM::Point2f pt1,pt2;
            lastKF->getKeyPoint(m.first,pt1);
            curFrame->getKeyPoint(m.second,pt2);
            cv::line(img,cv::Point2f(pt1.x,pt1.y),cv::Point2f(pt2.x,pt2.y),cv::Scalar(255,0,0),cam2.width()/500);
        }
        handle->handle(new GSLAM::DebugImageEvent(img,"DebugMatcherH"));
    }
#endif

    matches.insert(matches.end(),matchesH.begin(),matchesH.end());

    return matches.size()>=numThreshold;
}



REGISTER_MATCHER(MatcherMultiH,multiH);
