//
// Created by chengyuqi on 19-3-7.
//

#ifdef HAS_OPENCV
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "GSLAM/core/Estimator.h"
#include <GSLAM/core/Timer.h>
#include <GSLAM/core/Vocabulary.h>

#include "MapFrame.h"
#include "Matcher.h"
using std::vector;


class MatcherFlann : public Matcher
{
public:
    double distance(double* F, const GSLAM::Point2d& src,const GSLAM::Point2d& dst)const{
        // src^T*F*dst=0
        double x=F[0]*src.x+F[1]*src.y+F[2];
        double y=F[3]*src.x+F[4]*src.y+F[5];
        double z=F[6]*src.x+F[7]*src.y+F[8];
        return fabs(dst.x*x+dst.y*y+z);
    }
    MatcherFlann()
    {
        _estimator=GSLAM::Estimator::create();
        if(!_estimator)
        {
            LOG(ERROR)<<"MatcherBoW failed to create Estimator.";
        }
    }

    virtual bool match4triangulation(const GSLAM::FramePtr& ref,const GSLAM::FramePtr& cur,
                                     std::vector<std::pair<int,int> >& matches)const
    {
        GSLAM::ScopedTimer mtimer("MatcherFlann::match4triangulation");

        return match4initialize(ref, cur, matches);
    }

    virtual bool match4initialize(const GSLAM::FramePtr& lastKF,const GSLAM::FramePtr& curFrame,
                                  std::vector<std::pair<int,int> >& matches)const
    {
        GSLAM::ScopedTimer mtimer("MatcherFlann::match4initialize");

        std::vector<cv::DMatch> matches12,matches21;
        int            nmatches=0;

        const cv::Mat& des1=lastKF->getDescriptor();
        const cv::Mat& des2=curFrame->getDescriptor();
        {
            cv::Ptr<cv::DescriptorMatcher> matcher(new cv::FlannBasedMatcher());
            matcher->match(des1,des2,matches12);
            matcher->match(des2,des1,matches21);
        }

        //block filter
        {
            matches.clear();
            matches.reserve(matches12.size());
            for(auto m:matches12)
            {
                assert(m.queryIdx<des1.rows&&m.trainIdx<des2.rows);
                if(matches21[m.trainIdx].trainIdx==m.queryIdx)
                    matches.push_back(std::pair<int,int>(m.queryIdx,m.trainIdx));
            }
        }

        int m1, m2, m3;
        int numThreshold=std::max(50,int(curFrame->keyPointNum()*0.03));
        m1 = matches.size();
        if(matches.size()<numThreshold) return false;
        // filt with fundamental
        vector<u_char>         mask;
        {
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

        m2 = matches.size();
        //自己写的
/*        vector<cv::DMatch> cvmatches;
        cv::Mat img1=lastKF->getImage().clone();
        cv::Mat img2=curFrame->getImage().clone();
        cv::Mat img_match;
        std::vector<cv::KeyPoint> keypoints_1, keypoints_2;
        std::vector<GSLAM::KeyPoint> keypoints1, keypoints2;
        keypoints1 = lastKF->getKeyPoints();
        keypoints2 = curFrame->getKeyPoints();
        cvmatches.resize(matches.size());
       *//* for (int j = 0; j <matches.size() ; ++j)
        {
            cvmatches[j].queryIdx = matches[j].first;
            cvmatches[j].trainIdx = matches[j].second;
            cvmatches[j].imgIdx = 1;
            cvmatches[j].distance =
        }*//*
        LOG(INFO)<<"lastKF_keypoints1.size= "<<lastKF->keyPointNum();
        LOG(INFO)<<"curFrame_keypoints2.size= "<<curFrame->keyPointNum();
        LOG(INFO)<<"keypoints1.size= "<<keypoints1.size();
        LOG(INFO)<<"keypoints2.size= "<<keypoints2.size();
        LOG(INFO)<<"matches12.size= "<<matches12.size();
        LOG(INFO)<<"matches.size= "<<matches.size();
        for (int j = 0; j <keypoints1.size() ; ++j)
        {
            keypoints_1.push_back((cv::KeyPoint) keypoints1[j]);
            //LOG(INFO)<<"keypoints_1["<<j<<"].x= "<< keypoints_1[j].pt.x<<" ,keypoints_1["<<j<<"].y= "<<keypoints_1[j].pt.y;

        }
        for (int k = 0; k <keypoints2.size() ; ++k)
        {
            keypoints_2.push_back((cv::KeyPoint) keypoints2[k]);
           // LOG(INFO)<<"keypoints_2["<<k<<"].x= "<< keypoints_2[k].pt.x<<" ,keypoints_2["<<k<<"].y= "<<keypoints1[k].pt.y;

        }
        cv::drawMatches(img1, keypoints_1, img2, keypoints_2, matches12, img_match);
        cv::imshow ( "优化后匹配点对", img_match );*/
        printf("m1: %d, m2: %d\n", m1, m2);
        if( m2 < m1*0.2 ) return false;

        return true;

        //return matches.size()>100;
    }

    virtual bool findMatchWindow(const GSLAM::GImage& des,const GSLAM::FramePtr& fr,
                                 const float& x,const float& y,const float& r,
                                 int& idx,bool discardMapPoints=true)const
    {
        std::vector<size_t> candidates=fr->getFeaturesInArea(x,y,r);
        if(candidates.empty()) return false;

        float minDistance=1e8;
        float maxDistance=-1;
        if(maxDistance<0)
        {
            if(des.type()==GSLAM::GImageType<float>::Type&&des.cols==128) maxDistance=0.2;//SIFT
            else if(des.type()==GSLAM::GImageType<uchar>::Type&&des.cols==64) maxDistance=50;//ORB

            assert(maxDistance>0);
        }
        for(size_t& i:candidates)
        {
            if(discardMapPoints&&fr->getKeyPointObserve(i)) continue;
            GSLAM::GImage des1=fr->getDescriptor(i);

            float dis=GSLAM::Vocabulary::distance(des1,des);
            if(dis<minDistance)
            {
                minDistance=dis;
                idx=i;
            }
        }

        if(minDistance<maxDistance) return true;
        else return false;
    }

    SPtr<GSLAM::Estimator> _estimator;

};

REGISTER_MATCHER(MatcherFlann,flanntest);

#endif
