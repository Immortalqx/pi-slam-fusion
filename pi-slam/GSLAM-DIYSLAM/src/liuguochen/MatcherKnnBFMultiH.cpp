#ifdef HAS_OPENCV
#include <opencv2/features2d/features2d.hpp>
#include "opencv2/video/tracking.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "Matcher.h"
#include "GSLAM/core/Timer.h"
#include "GSLAM/core/Vocabulary.h"
#include "GSLAM/core/Estimator.h"
#include "GSLAM/core/Event.h"
#include "Estimator.h"

using namespace std;

class MatcherKnnBFMultiH : public Matcher
{
public:
    MatcherKnnBFMultiH()
    {
        _estimator=GSLAM::Estimator::create();
        if(!_estimator)
        {
            _estimator=createEstimatorInstance();
            if(!_estimator)
                LOG(ERROR)<<"MatcherKnnBFMultiH failed to create Estimator.";
        }
    }

    virtual bool match4initialize(const GSLAM::FramePtr& lastKF,const GSLAM::FramePtr& curFrame,
                                  std::vector<std::pair<int,int> >& matches)const;

    virtual bool match4triangulation(const GSLAM::FramePtr& ref,const GSLAM::FramePtr& cur,
                                     std::vector<std::pair<int,int> >& matches)const {
//        GSLAM::ScopedTimer mTimer("MatcherBoW::match4triangulation");

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

//        if(minDistance>minDistance2*0.8) return false;

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

    double getMatchScore(const GSLAM::Point2f srcPoint, const cv::Mat srcImage,
                         const GSLAM::Point2f dstPoint, const cv::Mat dstImage, const int windowSize) const
    {
        if (srcPoint.x < windowSize || srcPoint.y < windowSize || dstPoint.x < windowSize || dstPoint.y < windowSize)
            return -1;
        else if(srcPoint.x + windowSize > srcImage.cols || srcPoint.y + windowSize > srcImage.rows)
            return -1;
        else if(dstPoint.x + windowSize > dstImage.cols || dstPoint.y + windowSize > dstImage.rows)
            return -1;

        int radius = windowSize / 2;
        double distance = 0;

        for (int i = -radius; i <= radius; ++i)
        {
            for (int j = -radius; j <= radius; ++j)
            {
                int srcPointX = srcPoint.x + j;
                int srcPointY = srcPoint.y + i;
                int dstPointX = dstPoint.x + j;
                int dstPointY = dstPoint.y + i;

                int srcB = srcImage.at<cv::Vec3b>(srcPointY, srcPointX)[0];
                int srcG = srcImage.at<cv::Vec3b>(srcPointY, srcPointX)[1];
                int srcR = srcImage.at<cv::Vec3b>(srcPointY, srcPointX)[2];

                int dstB = dstImage.at<cv::Vec3b>(dstPointY, dstPointX)[0];
                int dstG = dstImage.at<cv::Vec3b>(dstPointY, dstPointX)[1];
                int dstR = dstImage.at<cv::Vec3b>(dstPointY, dstPointX)[2];

                int srcGray = (srcB * 15 + srcG * 75 + srcR * 38) >> 7;
                int dstGray = (dstB * 15 + dstG * 75 + dstR * 38) >> 7;

                distance += pow(srcGray - dstGray, 2);
            }
        }

        return sqrt(distance);
    }

    std::vector<size_t> getFeaturesAlongPolarLine(double* F, const GSLAM::Point2f src,
                                                  const GSLAM::FramePtr& curKF, const double distance) const
    {
        if (distance <= 0)
            return std::vector<size_t> (0);

        std::vector<size_t > features;

        double x=F[0]*src.x+F[1]*src.y+F[2];
        double y=F[3]*src.x+F[4]*src.y+F[5];
        double z=F[6]*src.x+F[7]*src.y+F[8];

        for (int i = 0; i < curKF->keyPointNum(); ++i)
        {
            GSLAM::Point2f pt;
            if (!curKF->getKeyPoint(i, pt))
                continue;

            double featureDistance = fabs(pt.x + pt.y + z);
            if (featureDistance < distance)
                features.push_back(i);
        }

        return features;
    }

    SPtr<GSLAM::Estimator> _estimator;
};

bool MatcherKnnBFMultiH::match4initialize(const GSLAM::FramePtr& lastKF,const GSLAM::FramePtr& curFrame,
                                       std::vector<std::pair<int,int> >& matches)const
{
    SCOPE_TIMER

    std::vector<std::vector<cv::DMatch> > matches12, matches21;

    const cv::Mat& des1 = lastKF->getDescriptor();
    const cv::Mat& des2 = curFrame->getDescriptor();

    cv::PCA pca1(des1, cv::Mat(), CV_PCA_DATA_AS_ROW, 20);

    cv::Mat des11 = pca1.project(des1);
    cv::Mat des22 = pca1.project(des2);

    {
        cv::Ptr<cv::DescriptorMatcher> matcher(new cv::FlannBasedMatcher());
//        cv::Ptr<cv::DescriptorMatcher> matcher = cv::makePtr<cv::FlannBasedMatcher>(cv::makePtr<cv::flann::KDTreeIndexParams>());
        matcher->knnMatch(des11, des22, matches12, 1);
        matcher->knnMatch(des22, des11, matches21, 1);
    }

    {
        matches.clear();
        matches.reserve(matches12.size());

        for(auto m:matches12)
        {
//            if(matches21[m[0].trainIdx][0].trainIdx == m[0].queryIdx || matches21[m[0].trainIdx][1].trainIdx == m[0].queryIdx)
//                matches.push_back(std::pair<int, int>(m[0].queryIdx, m[0].trainIdx));
//            else if(matches21[m[1].trainIdx][0].trainIdx == m[1].queryIdx || matches21[m[1].trainIdx][1].trainIdx == m[1].queryIdx)
//                matches.push_back(std::pair<int, int>(m[1].queryIdx, m[1].trainIdx));

            if (matches21[m[0].trainIdx][0].trainIdx == m[0].queryIdx)
                matches.push_back(std::pair<int, int>(m[0].queryIdx, m[0].trainIdx));
        }
    }

//    if (matches.size() < 100) return false;

    GSLAM::Camera cam1 = lastKF->getCamera();
    GSLAM::Camera cam2 = curFrame->getCamera();
    double F[9];
    vector<GSLAM::Point2d> points1, points2;
    vector<uchar > mask;

    {
        for(auto& m:matches)
        {
            GSLAM::Point2f pt;
            if(!lastKF->getKeyPoint(m.first, pt)) LOG(FATAL) << "MapFrame::getKeyPoint() not implemented!";
            GSLAM::Point3d p3d = cam1.UnProject(pt.x, pt.y);
            points1.push_back(GSLAM::Point2d(p3d.x, p3d.y));

            if(!curFrame->getKeyPoint(m.second,pt)) LOG(FATAL)<<"MapFrame::getKeyPoint() not implemented!";
            p3d=cam2.UnProject(pt.x,pt.y);
            points2.push_back(GSLAM::Point2d(p3d.x,p3d.y));
        }

        _estimator->findFundamental(F,points1,points2,GSLAM::RANSAC,0.01,0.99,&mask);
    }

    // reduce
    if(mask.size())
    {
        int curSize=0;
        for(size_t i=0;i<matches.size();i++){
            if(mask[i])
                matches[curSize++]=matches[i];
        }
        matches.resize(curSize);
    }

//    if (matches.size() < 100)
//    {
//        return false;
//    }

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

    int numThreshold = std::max(50,int(mask.size()*0.1));
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
        double minDistance=1e10;
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

    LOG(WARNING) << "matchesHSize: " << matchesH.size();
    matches.insert(matches.end(),matchesH.begin(),matchesH.end());

    if (svar.GetInt("MatcherFlannMultiH.FilterAngle", 1))
    {
        int binNumberSelect = svar.GetInt("MatcherFlannMultiH.BinNumberSelect", 1);
        int binNumber = svar.GetInt("MatcherFlannMultiH.BinNumber", 45);

        std::vector<std::pair<int, int> > filterMatches;
        std::vector<std::vector<int> > bins(binNumber, std::vector<int>());

//        for (int i = 0; i < binNumber; ++i)
//        {
//            bins[i].reserve(matches.size());
//        }

        for (int i = 0; i < matches.size(); ++i)
        {
            GSLAM::KeyPoint kp1, kp2;
            if (!lastKF->getKeyPoint(matches[i].first, kp1)) continue;
            if (!curFrame->getKeyPoint(matches[i].second, kp2)) continue;
            float angle = kp1.angle - kp2.angle;
            if(angle < 0) angle+=360.0;
            int selectBin = angle/(360/binNumber);
            bins[selectBin].push_back(i);
        }

        int bestSelectBinsNumber = 0;
        int allSelectBinsNumber = 0;
        int bestIdx = 0;

        for (int i = 0; i < binNumberSelect; ++i)
        {
            allSelectBinsNumber += bins[i].size();
        }
        bestSelectBinsNumber = allSelectBinsNumber;

        for (int i = binNumberSelect; i < binNumber; ++i)
        {
            allSelectBinsNumber = allSelectBinsNumber - bins[i - binNumberSelect].size() + bins[i].size();
            if (allSelectBinsNumber > bestSelectBinsNumber)
            {
                bestSelectBinsNumber = allSelectBinsNumber;
                bestIdx = i - binNumberSelect + 1;
            }
        }

        for (int i = 0; i < binNumberSelect; ++i)
        {
            assert(binNumber - binNumberSelect + i < binNumber);
            allSelectBinsNumber = allSelectBinsNumber - bins[binNumber - binNumberSelect + i].size() + bins[i].size();
            if (allSelectBinsNumber > bestSelectBinsNumber)
            {
                bestSelectBinsNumber = allSelectBinsNumber;
                bestIdx = binNumber - binNumberSelect + i + 1;
            }
        }

        int curSize=0;
        for(int i = 0; i < binNumberSelect; i++)
        {
            int selectBinNumber = i + bestIdx;
            if (selectBinNumber >= binNumber)
                selectBinNumber = selectBinNumber - binNumber;

            assert(selectBinNumber < binNumber);
            for(int j:bins[selectBinNumber])
            {
                filterMatches.push_back(matches[j]);
                curSize++;
            }
        }

        matches.swap(filterMatches);
        matches.resize(curSize);
    }

    if (svar.GetInt("MatcherFlannMultiH.FilterAngleGaussian", 1))
    {
        int binNumberSelect = svar.GetInt("MatcherFlannMultiH.BinNumberSelectGaussian", 1);
        int binNumber = svar.GetInt("MatcherFlannMultiH.BinNumberGaussian", 45);

        std::vector<std::pair<int, int> > filterMatches;
        std::vector<std::vector<int> > bins(binNumber, std::vector<int>());

        for (int i = 0; i < matches.size(); ++i)
        {
            GSLAM::KeyPoint kp1, kp2;
            if (!lastKF->getKeyPoint(matches[i].first, kp1)) continue;
            if (!curFrame->getKeyPoint(matches[i].second, kp2)) continue;
            float angle = kp1.angle - kp2.angle;
            if(angle < 0) angle+=360.0;
            int selectBin = angle/(360/binNumber);
            bins[selectBin].push_back(i);
        }

        int bestIdx = 0;
        int maxMatches = -1;
        for (int i = 0; i < bins.size(); ++i)
        {
            if(bins[i].size() > maxMatches)
            {
                bestIdx = i;
                maxMatches = bins[i].size();
            }
        }

        int halfNumberSelect = binNumberSelect/2;
        int curSize=0;
        for(int i = -halfNumberSelect; i <= halfNumberSelect; i++)
        {
            int selectBinNumber = i + bestIdx;
            if (selectBinNumber >= binNumber)
                selectBinNumber = selectBinNumber - binNumber;
            else if(selectBinNumber < 0)
                selectBinNumber = binNumber + selectBinNumber;

            assert(selectBinNumber < binNumber);
            for(int j:bins[selectBinNumber])
            {
                filterMatches.push_back(matches[j]);
                curSize++;
            }
        }

        matches.swap(filterMatches);
        matches.resize(curSize);
    }

    if(svar.GetInt("MatcherFlannMultiH.PreFilterAngle",1))
    {
        int binNumber = svar.GetInt("MatcherFlannMultiH.PreBinNumber", 45);
        std::vector<std::pair<int,int> > filterMatches;

        std::vector<int> *bins;
        bins = new std::vector<int>[binNumber];

        for(int i=0;i<binNumber;i++)
            bins[i].reserve(matches.size());

        for(int i=0;i<matches.size();i++){
            GSLAM::KeyPoint kp1,kp2;
            if(!lastKF->getKeyPoint(matches[i].first,kp1)) continue;
            if(!curFrame->getKeyPoint(matches[i].second,kp2)) continue;
            float angle=kp1.angle-kp2.angle;
            if(angle<0) angle+=360;
            bins[int(angle/binNumber)].push_back(i);
        }

        int max1=0;
        int max2=0;
        int max3=0;
        int ind1=-1;
        int ind2=-1;
        int ind3=-1;

        for(int i=0; i<binNumber; i++)
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
        for(int i=0; i<binNumber; i++)
        {
            if(i==ind1 || i==ind2 || i==ind3)
                for(int j:bins[i])
                {
                    filterMatches.push_back(matches[j]);
                    curSize++;
                }
        }
        DLOG(INFO)<<"FilterAngle:"<<matches.size()<<"->"<<curSize;
        matches.swap(filterMatches);
        matches.resize(curSize);

        delete [] bins;
    }

    std::function<int(std::vector<uchar>&) > computeInliers = [](std::vector<uchar> &mask) -> int{
        int curSize = 0;
        for (int i = 0; i < mask.size(); ++i)
        {
            if(mask[i])
                curSize++;
        }
        return curSize;
    };

    std::function<bool(const GSLAM::Point3d &, const GSLAM::Point3d &, double* F, double ) > filterF = [](const GSLAM::Point3d &p1, const GSLAM::Point3d &p2, double* F, double maxError) -> bool{
        double maxResidual = maxError * maxError;

        const double Fp1x = F[0] * p1.x + F[1] * p1.y + F[2] * p1.z;
        const double Fp1y = F[3] * p1.x + F[4] * p1.y + F[5] * p1.z;
        const double Fp1z = F[6] * p1.x + F[7] * p1.y + F[8] * p1.z;
        const double Ftp2x = F[0] * p2.x + F[3] * p2.y + F[6] * p2.z;
        const double Ftp2y = F[1] * p2.x + F[4] * p2.y + F[7] * p2.z;
        const double Ftp2z = F[2] * p2.x + F[5] * p2.y + F[8] * p2.z;

        const double p2tFp1 = p2.x * Fp1x + p2.y * Fp1y + p2.z * Fp1z;

        return p2tFp1 * p2tFp1 /
               (Fp1x * Fp1x + Fp1y * Fp1y + Ftp2x * Ftp2x + Ftp2y * Ftp2y) < maxResidual;
    };

    std::function<bool(const GSLAM::Point3d &, const GSLAM::Point3d &, double* H, double) > filterH = [](const GSLAM::Point3d &p1, const GSLAM::Point3d &p2, double* H, double maxError) -> bool{
        double maxResidual = maxError * maxError;

        const double Hp1x = H[0] * p1.x + H[1] * p1.y + H[2] * p1.z;
        const double Hp1y = H[3] * p1.x + H[4] * p1.y + H[5] * p1.z;
        const double Hp1z = H[6] * p1.x + H[7] * p1.y + H[8] * p1.z;

        const double distanceX = (Hp1x / Hp1z) - p2.x;
        const double distanceY = (Hp1y / Hp1z) - p2.y;

        return sqrt(distanceX * distanceX + distanceY * distanceY) < maxResidual;
    };

    enum ConfigType
    {
        UNDEFINED = 0,
        // Degenerate configuration (e.g., no overlap or not enough inliers).
            DEGENERATE = 1,
        // Essential matrix.
            CALIBRATED = 2,
        // Fundamental matrix.
            UNCALIBRATED = 3,
        // Homography, planar scene with baseline.
            PLANAR = 4,
        // Homography, pure rotation without baseline.
            PANORAMIC = 5,
        // Homography, planar or panoramic.
            PLANAR_OR_PANORAMIC = 6,
        // Watermark, pure 2D translation in image borders.
            WATERMARK = 7,
        // Multi-model configuration, i.e. the inlier matches result from multiple
        // individual, non-degenerate configurations.
            MULTIPLE = 8,
    };

    if (svar.GetInt("MatcherKnnBFMultiH.colmapSolution", 1))
    {
        vector<uchar> outInliersMask(matches.size(), 0);
        double H[9];
        double E[9];

        vector<std::pair<int, int> > matchesOutput;

        {
            vector<std::pair<int, int> > matchesHandled(matches);
            while (true)
            {
                if (matchesHandled.size() < 15)
                    break;

                double maxError = svar.GetDouble("MatcherKnnBFMultiH.colmapMaxError", 0.2);
                vector<uchar> maskH;
                vector<uchar> maskF;
                vector<uchar> maskE;

                vector<GSLAM::Point2d> points1, points2;
                vector<GSLAM::Point2d> points1Normed;
                vector<GSLAM::Point2d> points2Normed;

                for (auto &m:matchesHandled)
                {
                    GSLAM::Point2f pt;
                    if (!lastKF->getKeyPoint(m.first, pt)) LOG(FATAL) << "MapFrame::getKeyPoint() not implemented!";
                    GSLAM::Point3d p3d = cam1.UnProject(pt.x, pt.y);
                    points1.push_back(GSLAM::Point2d(p3d.x, p3d.y));
                    points1Normed.push_back(GSLAM::Point2d(pt.x, pt.y));

                    if (!curFrame->getKeyPoint(m.second, pt)) LOG(FATAL) << "MapFrame::getKeyPoint() not implemented!";
                    p3d = cam2.UnProject(pt.x, pt.y);
                    points2.push_back(GSLAM::Point2d(p3d.x, p3d.y));
                    points2Normed.push_back(GSLAM::Point2d(pt.x, pt.y));
                }

                bool successH = _estimator->findHomography(H, points1, points2, GSLAM::RANSAC, 0.01, &maskH);
                bool successF = _estimator->findFundamental(F, points1, points2, GSLAM::RANSAC, 0.01, 0.98, &maskF);
                bool successE = _estimator->findEssentialMatrix(E, points1Normed, points2Normed, GSLAM::RANSAC, 0.01, 0.98, &maskE);

                vector<uchar> *inlierMask;
                int numInliers = 0;
                int config = 0;
                double inliersH = computeInliers(maskH);
                double inliersF = computeInliers(maskF);
                double inliersE = computeInliers(maskE);

                const double E_F_inliersRatio = inliersE / inliersF;
                const double H_E_inliersRatio = inliersH / inliersE;
                const double H_F_inliersRatio = inliersH / inliersF;

                if (E_F_inliersRatio > 0.95 && inliersE > 15 && successE)
                {
                    if (inliersE > inliersF)
                    {
                        inlierMask = &maskE;
                        numInliers = inliersE;
                    }
                    else
                    {
                        inlierMask = &maskF;
                        numInliers = inliersF;
                    }

                    if (H_E_inliersRatio > 0.8)
                    {
                        config = PLANAR_OR_PANORAMIC;
                        if (inliersH > numInliers)
                        {
                            inlierMask = &maskH;
                            numInliers = inliersH;
                        }
                    }
                    else
                    {
                        config = CALIBRATED;
                    }
                }
                else if (inliersF >= 15 && successF)
                {
                    inlierMask = &maskF;
                    numInliers = inliersF;

                    if (H_F_inliersRatio > 0.8)
                    {
                        config = PLANAR_OR_PANORAMIC;
                        if (inliersH > numInliers)
                        {
                            inlierMask = &maskH;
                            numInliers = inliersH;
                        }
                    }
                    else
                    {
                        config = UNCALIBRATED;
                    }
                }
                else if (inliersH >= 15 && successH)
                {
                    inlierMask = &maskH;
                    numInliers = inliersH;
                    config = PLANAR_OR_PANORAMIC;
                }
                else
                {
                    config = DEGENERATE;
                }

                if (config == DEGENERATE)
                    break;

                if (inlierMask->size())
                {
                    int curSize = 0;
                    for (size_t i = 0; i < matchesHandled.size(); i++)
                    {
                        if (!inlierMask->at(i))
                            matchesHandled[curSize++] = matchesHandled[i];
                        else
                        {
                            GSLAM::Point2f point1, point2;
                            if (!lastKF->getKeyPoint(matchesHandled[i].first, point1))continue;
                            if (!curFrame->getKeyPoint(matchesHandled[i].second, point2))continue;

                            GSLAM::Point3d pt1 = cam1.UnProject(point1.x, point1.y);
                            GSLAM::Point3d pt2 = cam1.UnProject(point1.x, point1.y);

                            if (config == CALIBRATED || config == UNCALIBRATED)
                            {
                                if (filterF(pt1, pt2, F, maxError))
                                    matchesOutput.push_back(matchesHandled[i]);
                            }
                            else if (config == PLANAR_OR_PANORAMIC || config == PLANAR ||
                                     config == PANORAMIC)
                            {
                                if (filterH(pt1, pt2, H, maxError))
                                    matchesOutput.push_back(matchesHandled[i]);
                            }
                        }
                    }
                    LOG(WARNING) << "matchedHandledSize: " << matchesHandled.size();
                    matchesHandled.resize(curSize);
                }
            }
        }

        LOG(WARNING) << "PreMatchesSize: " << matches.size();
        matches.swap(matchesOutput);
        LOG(WARNING) << "NowMatchesSize: " << matches.size();
    }

    return matches.size()>=numThreshold;
}
#endif
REGISTER_MATCHER(MatcherKnnBFMultiH,bf_knn_multiH);
