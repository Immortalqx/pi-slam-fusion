//
// Created by liu on 19-4-4.
//

#ifdef HAS_OPENCV
#include <opencv2/features2d/features2d.hpp>

#include "GSLAM/core/Estimator.h"
#include <GSLAM/core/Timer.h>
#include <GSLAM/core/Vocabulary.h>
#include "zhaoyong/SiftGPU/SiftGPU.h"
#include "Matcher.h"
using std::vector;
class MatcherLiuSiftGPU : public Matcher {
public:
    MatcherLiuSiftGPU() {
        LOG(INFO) << "MatcherLiuSiftGPU";
        _estimator = GSLAM::Estimator::create();
        if (!_estimator) {
            LOG(ERROR) << "MatcherLiuSiftGPU failed to create Estimator.";
        }
    }

    virtual bool match4triangulation(const GSLAM::FramePtr &ref, const GSLAM::FramePtr &cur,
                                     std::vector<std::pair<int, int> > &matches) const {
        GSLAM::ScopedTimer mtimer("MatcherLiuSiftGPU::match4triangulation");

        return match4initialize(ref, cur, matches);
    }

    virtual bool match4initialize(const GSLAM::FramePtr &lastKF, const GSLAM::FramePtr &curFrame,
                                  std::vector<std::pair<int, int> > &matches) const {
        GSLAM::ScopedTimer mtimer("MatcherLiuSiftGPU::match4initialize");
        SiftGPU sift;
        SiftMatchGPU matcher_SiftGPU(4096);
        GSLAM::GImage descriptor_mat1 = lastKF->getDescriptor();
        GSLAM::GImage descriptor_mat2 = curFrame->getDescriptor();

        std::vector<GSLAM::KeyPoint> ref_keypoints(1), cur_keypoints(1);
        int key_num = 0;
        ref_keypoints.resize(lastKF->keyPointNum());
        cur_keypoints.resize(curFrame->keyPointNum());
        LOG(INFO)<<"lastKey_num: "<<lastKF->keyPointNum();
        LOG(INFO)<<"curKey_num: "<<curFrame->keyPointNum();
        while (key_num < lastKF->keyPointNum()) {
            lastKF->getKeyPoint(key_num, ref_keypoints[key_num]);
            key_num++;
        }
        key_num = 0;
        while (key_num < curFrame->keyPointNum()) {
            curFrame->getKeyPoint(key_num, cur_keypoints[key_num]);
            key_num++;
        }
//        int support = sift.CreateContextGL();
//        if (support != SiftGPU::SIFTGPU_FULL_SUPPORTED) {
//            std::cerr << "SiftGPU is not supported!" << std::endl;
//            return false;
//        }

        int match_buf[4096][2];
        LOG(INFO)<<"getkeypoint finish3";
        LOG(INFO)<<"getkeypoint finish4";
//        LOG(INFO)<<"descriptor1: "<<descriptor1.size();
//        LOG(INFO)<<"descriptor2: "<<descriptor2.size();
        LOG(INFO)<<"ref_keypoints: "<<ref_keypoints.size();
        LOG(INFO)<<"cur_keypoints: "<<cur_keypoints.size();
//        descriptor1.resize(128*ref_keypoints.size());
//        descriptor2.resize(128*cur_keypoints.size());
        matcher_SiftGPU.VerifyContextGL();
//            sift->GetFeatureVector(&ref_keypoints[0], &descriptor1[0]);
//            sift->GetFeatureVector(&cur_keypoints[0], &descriptor2[0]);
        LOG(INFO)<<"ref_keypoints: "<<ref_keypoints.size();
        LOG(INFO)<<"cur_keypoints: "<<cur_keypoints.size();
        matcher_SiftGPU.SetDescriptors(0, ref_keypoints.size(), descriptor_mat1.ptr<float>(0));
        matcher_SiftGPU.SetDescriptors(1, cur_keypoints.size(), descriptor_mat2.ptr<float>(0));
        LOG(INFO)<<"SetDescriptors finish";
        int num_match = matcher_SiftGPU.GetSiftMatch(4096, match_buf);

        matches.reserve(num_match);
        for (int i = 0; i < num_match; ++i) {
            matches.push_back(std::pair<int, int>(match_buf[i][0], match_buf[i][1]));
        }
        int m1, m2, m3;
        int numThreshold = std::max(50, int(curFrame->keyPointNum() * 0.03));
        m1 = matches.size();
        if (matches.size() < numThreshold) return false;
        LOG(INFO)<<"matches finish";
        // filt with fundamental
        vector<u_char> mask;
        {
            GSLAM::Camera cam1 = lastKF->getCamera();
            GSLAM::Camera cam2 = curFrame->getCamera();
            double F[9];
            vector<GSLAM::Point2d> points1, points2;
            for (auto &m:matches) {
                GSLAM::Point2f pt;
                if (!lastKF->getKeyPoint(m.first, pt)) LOG(FATAL) << "MapFrame::getKeyPoint() not implemented!";
                GSLAM::Point3d p3d = cam1.UnProject(pt.x, pt.y);
                points1.push_back(GSLAM::Point2d(p3d.x, p3d.y));

                if (!curFrame->getKeyPoint(m.second, pt)) LOG(FATAL) << "MapFrame::getKeyPoint() not implemented!";
                p3d = cam2.UnProject(pt.x, pt.y);
                points2.push_back(GSLAM::Point2d(p3d.x, p3d.y));
            }

            _estimator->findFundamental(F, points1, points2, GSLAM::RANSAC, 0.05, 0.99, &mask);
        }

        // reduce
        if (mask.size()) {
            int curSize = 0;
            for (size_t i = 0; i < matches.size(); i++) {
                if (mask[i])
                    matches[curSize++] = matches[i];
            }
            matches.resize(curSize);
        }

        m2 = matches.size();

        printf("m1: %d, m2: %d\n", m1, m2);
        if (m2 < m1 * 0.2) return false;

        return true;

        //return matches.size()>100;
    }

    virtual bool findMatchWindow(const GSLAM::GImage &des, const GSLAM::FramePtr &fr,
                                 const float &x, const float &y, const float &r,
                                 int &idx, bool discardMapPoints = true) const {
        std::vector<size_t> candidates = fr->getFeaturesInArea(x, y, r);
        if (candidates.empty()) return false;

        float minDistance = 1e8;
        float maxDistance = -1;
        if (maxDistance < 0) {
            if (des.type() == GSLAM::GImageType<float>::Type && des.cols == 128) maxDistance = 0.2;//SIFT
            else if (des.type() == GSLAM::GImageType<uchar>::Type && des.cols == 64) maxDistance = 50;//ORB

            assert(maxDistance > 0);
        }
        for (size_t &i:candidates) {
            if (discardMapPoints && fr->getKeyPointObserve(i)) continue;
            GSLAM::GImage des1 = fr->getDescriptor(i);

            float dis = GSLAM::Vocabulary::distance(des1, des);
            if (dis < minDistance) {
                minDistance = dis;
                idx = i;
            }
        }

        if (minDistance < maxDistance) return true;
        else return false;
    }

    SPtr<GSLAM::Estimator> _estimator;

};

REGISTER_MATCHER(MatcherLiuSiftGPU, liu_SiftGPU);

#endif
