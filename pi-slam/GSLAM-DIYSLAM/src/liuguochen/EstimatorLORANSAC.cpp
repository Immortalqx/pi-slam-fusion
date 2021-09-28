//#if !defined(HAS_OPENCV)&&defined(HAS_EIGEN3)

#include <GSLAM/core/Estimator.h>
#include <GSLAM/core/Random.h>
#include <opencv2/opencv.hpp>

namespace GSLAM
{

    class EstimatorLORANSAC : public Estimator
    {
    public:
        EstimatorLORANSAC()
        {

        }

        void Normalize(const std::vector<cv::Point2d> &vPts, std::vector<cv::Point2d> &vNormalizedPoints, cv::Mat &T) const
        {
            double meanX = 0;
            double meanY = 0;
            const int N = vPts.size();

            vNormalizedPoints.resize(N);

            for(int i=0; i<N; i++)
            {
                meanX += vPts[i].x;
                meanY += vPts[i].y;
            }

            meanX = meanX/N;
            meanY = meanY/N;

            double meanDevX = 0;
            double meanDevY = 0;

            for(int i=0; i<N; i++)
            {
                vNormalizedPoints[i].x = vPts[i].x - meanX;
                vNormalizedPoints[i].y = vPts[i].y - meanY;

                meanDevX += abs(vNormalizedPoints[i].x);
                meanDevY += abs(vNormalizedPoints[i].y);
            }

            meanDevX = meanDevX/N;
            meanDevY = meanDevY/N;

            double sX = 1.0/meanDevX;
            double sY = 1.0/meanDevY;

            for(int i=0; i<N; i++)
            {
                vNormalizedPoints[i].x = vNormalizedPoints[i].x * sX;
                vNormalizedPoints[i].y = vNormalizedPoints[i].y * sY;
            }

            T = cv::Mat::eye(3,3,CV_32F);
            T.at<float>(0,0) = sX;
            T.at<float>(1,1) = sY;
            T.at<float>(0,2) = -meanX*sX;
            T.at<float>(1,2) = -meanY*sY;
        }

        cv::Mat ComputeF21(const std::vector<cv::Point2d> &vP1,const std::vector<cv::Point2d> &vP2) const
        {
            const int N = vP1.size();

            cv::Mat A(N,9,CV_32F);

            for(int i=0; i<N; i++)
            {
                const float u1 = vP1[i].x;
                const float v1 = vP1[i].y;
                const float u2 = vP2[i].x;
                const float v2 = vP2[i].y;

                A.at<float>(i,0) = u2*u1;
                A.at<float>(i,1) = u2*v1;
                A.at<float>(i,2) = u2;
                A.at<float>(i,3) = v2*u1;
                A.at<float>(i,4) = v2*v1;
                A.at<float>(i,5) = v2;
                A.at<float>(i,6) = u1;
                A.at<float>(i,7) = v1;
                A.at<float>(i,8) = 1;
            }

            cv::Mat u,w,vt;

            cv::SVDecomp(A,w,u,vt,cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

            cv::Mat Fpre = vt.row(8).reshape(0, 3);

            cv::SVDecomp(Fpre,w,u,vt,cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

            w.at<float>(2,2)=0;

            return  u*cv::Mat::diag(w)*vt;
        }

        float CheckFundamental(const cv::Mat &F21, const std::vector<cv::Point2d>& mvPn1, const std::vector<cv::Point2d>& mvPn2,
                               std::vector<bool> &vbMatchesInliers, float sigma) const
        {
//            const int N = vbMatchesInliers.size();
//
//            const float f11 = F21.at<float>(0,0);
//            const float f12 = F21.at<float>(0,1);
//            const float f13 = F21.at<float>(0,2);
//            const float f21 = F21.at<float>(1,0);
//            const float f22 = F21.at<float>(1,1);
//            const float f23 = F21.at<float>(1,2);
//            const float f31 = F21.at<float>(2,0);
//            const float f32 = F21.at<float>(2,1);
//            const float f33 = F21.at<float>(2,2);
//
//            vbMatchesInliers.resize(N);
//
//            float score = 0;
//
//            const float th = 3.841;
//            const float thScore = 5.991;
//
//            const float invSigmaSquare = 1.0/(sigma*sigma);
//
//            for(int i=0; i<N; i++)
//            {
//                bool bIn = true;
//
//                const cv::Point2f &kp1=mvPn1[i];
//                const cv::Point2f &kp2=mvPn2[i];
//
//                const float u1 = kp1.x;
//                const float v1 = kp1.y;
//                const float u2 = kp2.x;
//                const float v2 = kp2.y;
//
//                // Reprojection error in second image
//                // l2=F21x1=(a2,b2,c2)
//
//                const float a2 = f11*u1+f12*v1+f13;
//                const float b2 = f21*u1+f22*v1+f23;
//                const float c2 = f31*u1+f32*v1+f33;
//
//                const float num2 = a2*u2+b2*v2+c2;
//
//                const float squareDist1 = num2*num2/(a2*a2+b2*b2);
//
//                const float chiSquare1 = squareDist1*invSigmaSquare;
//
//                if(chiSquare1>th)
//                    bIn = false;
//                else
//                    score += thScore - chiSquare1;
//
//                // Reprojection error in second image
//                // l1 =x2tF21=(a1,b1,c1)
//
//                const float a1 = f11*u2+f21*v2+f31;
//                const float b1 = f12*u2+f22*v2+f32;
//                const float c1 = f13*u2+f23*v2+f33;
//
//                const float num1 = a1*u1+b1*v1+c1;
//
//                const float squareDist2 = num1*num1/(a1*a1+b1*b1);
//
//                const float chiSquare2 = squareDist2*invSigmaSquare;
//
//                if(chiSquare2>th)
//                    bIn = false;
//                else
//                    score += thScore - chiSquare2;
//
//                if(bIn)
//                    vbMatchesInliers[i]=true;
//                else
//                    vbMatchesInliers[i]=false;
//            }
//
//            return score;

            const int N = vbMatchesInliers.size();

            const double f11 = F21.at<float>(0,0);
            const double f12 = F21.at<float>(0,1);
            const double f13 = F21.at<float>(0,2);
            const double f21 = F21.at<float>(1,0);
            const double f22 = F21.at<float>(1,1);
            const double f23 = F21.at<float>(1,2);
            const double f31 = F21.at<float>(2,0);
            const double f32 = F21.at<float>(2,1);
            const double f33 = F21.at<float>(2,2);

            vbMatchesInliers.resize(N);
            double err = sigma * sigma;
            double score = 0;

            for (int i = 0; i < N; ++i)
            {
                const cv::Point2f &kp1=mvPn1[i];
                const cv::Point2f &kp2=mvPn2[i];

                const double u1 = kp1.x;
                const double v1 = kp1.y;
                const double u2 = kp2.x;
                const double v2 = kp2.y;

                const double a1 = f11*u1+f12*v1+f13;
                const double b1 = f21*u1+f22*v1+f23;
                const double c1 = f31*u1+f32*v1+f33;

                const double s1 = 1./(a1 * a1 + b1 * b1);
                const double d1 = u2 * a1 + v2 * b1 + c1;

                const double a2 = f11*u2+f21*v2+f31;
                const double b2 = f12*u2+f22*v2+f32;
                const double c2 = f13*u2+f23*v2+f33;

                const double s2 = 1./(a2 * a2 + b2 * b2);
                const double d2 = u1 * a2 + v1 * b2 + c2;

                double thisErr = (double )std::max(d1 * d1 * s1, d2 * d2 * s2);
                if (thisErr < err)
                {
                    vbMatchesInliers[i] = true;
                    score += thisErr;
                }
            }

            return score;
        }

        int UpdateNumIters(double p, double ep, int model_points, int max_iters) const
        {
            p = std::max(p, 0.);
            p = std::min(p, 1.);
            ep = std::max(ep, 0.);
            ep = std::min(ep, 1.);

            double num = std::max(1. - p, DBL_MIN);
            double denom = 1. - pow(1. - ep, model_points);
            if (denom < DBL_MIN)
                return 0;

            num = log(num);
            denom = log(denom);

            return denom >= 0 || -num >= max_iters * (-denom) ? max_iters : cvRound(num / denom);
        }

        inline int getInlierNum(std::vector<bool> &matchInlier) const
        {
            int num = 0;
            for(bool isInlier: matchInlier)
                if(isInlier)
                    num++;
            return num;
        }

        inline std::vector<cv::Point2d> toInputArray(const std::vector<Point2d>& input)const
        {
            return *(std::vector<cv::Point2d>*)&input;
        }

        inline std::vector<cv::Point3d> toInputArray(const std::vector<Point3d>& input)const
        {
            return *(std::vector<cv::Point3d>*)&input;
        }

        inline bool toReturn(double* Ret,cv::Mat result)const
        {
            if(result.empty()) return false;
            result.convertTo(result,CV_64F);
            memcpy(Ret,result.data,sizeof(double)*result.total());
            return true;
        }

        std::string type()const{return "EstimatorLORANSAC";}

        // 2D corrospondences
        bool findHomography(double* H,//3x3 dof=8
                                    const std::vector<Point2d>& srcPoints,
                                    const std::vector<Point2d>& dstPoints,
                                    int method=0, double ransacReprojThreshold=3,
                                    std::vector<uchar>* mask=NULL)const
        {
            if(!H) return false;
            return toReturn(H,cv::findHomography(toInputArray(srcPoints),toInputArray(dstPoints),
                                                 method,ransacReprojThreshold,mask?(*mask):cv::noArray()));
        }

        bool findFundamental(double* F,//3x3
                                     const std::vector<Point2d>& points1,
                                     const std::vector<Point2d>& points2,
                                     int method=0, double param1=3., double param2=0.99,
                                     std::vector<uchar>* mask=NULL)const
        {
            int modelNum = 8;
            int maxIters = 2000;
            int N = points1.size();
            mask->resize(N, false);

            std::vector<cv::Point2d> _points1 = toInputArray(points1), _points2 = toInputArray(points2);

            std::vector<cv::Point2d> vPn1i(modelNum);
            std::vector<cv::Point2d> vPn2i(modelNum);
            std::vector<cv::Point2d> inliers1;
            std::vector<cv::Point2d> inliers2;
            cv::Mat F21, F21i;
            std::vector<bool>  vbMatchesInliers;
            double bestScore = 0, currentScore = 0, localScore = 0;
            int bestInlierNum = 0, currentInlierNum = 0, localInlierNum = 0;

            std::vector<size_t> vAllIndices;
            vAllIndices.reserve(N);
            std::vector<size_t> vAvailableIndices;

            for(int i=0; i<N; i++)
            {
                vAllIndices.push_back(i);
            }

            std::vector< std::vector<size_t> > mvSets(maxIters, std::vector<size_t>(modelNum,0));

            for(int it=0; it<maxIters; it++)
            {
                vAvailableIndices = vAllIndices;

                // Select a minimum set
                for(size_t j=0; j<modelNum; j++)
                {
                    int randi = GSLAM::Random::RandomInt(0,vAvailableIndices.size()-1);
                    int idx = vAvailableIndices[randi];

                    mvSets[it][j] = idx;

                    vAvailableIndices[randi] = vAvailableIndices.back();
                    vAvailableIndices.pop_back();
                }
            }

            for(int it=0; it<maxIters; it++)
            {
                // Select a minimum set
                std::vector<bool> vbCurrentInliers(N,false);
                for(int j=0; j<modelNum; j++)
                {
                    int idx = mvSets[it][j];

                    vPn1i[j] = _points1[idx];
                    vPn2i[j] = _points2[idx];
                }

                cv::Mat Fn = ComputeF21(vPn1i,vPn2i);

                F21i = Fn;

                currentScore = CheckFundamental(F21i, _points1, _points2, vbCurrentInliers, param1);
                int inlierNum = getInlierNum(vbCurrentInliers);

                static int localComputeNum = 0;
                if(inlierNum > bestInlierNum || (inlierNum == bestInlierNum && currentScore < bestScore) && inlierNum >= 8)
                {
                    std::vector<bool> vbLocalInliers(N, false);
                    F21 = F21i.clone();
                    vbMatchesInliers = vbCurrentInliers;
                    bestScore = currentScore;
                    bestInlierNum = inlierNum;

                    inliers1.clear();
                    inliers2.clear();
                    inliers1.reserve(inlierNum);
                    inliers2.reserve(inlierNum);

                    for (int i = 0; i < vbMatchesInliers.size(); ++i)
                    {
                        if (vbMatchesInliers[i])
                        {
                            inliers1.emplace_back(_points1[i]);
                            inliers2.emplace_back(_points2[i]);
                        }
                    }
                    cv::Mat F_local = ComputeF21(inliers1, inliers2);
                    cv::Mat F_local_T = F_local;
                    localScore = CheckFundamental(F_local_T, _points1, _points2, vbLocalInliers, param1);
                    localInlierNum = getInlierNum(vbLocalInliers);

                    if(localInlierNum > bestInlierNum || (localInlierNum == bestInlierNum && localScore < bestScore))
                    {
                        localComputeNum++;
                        F21 = F_local_T.clone();
                        vbMatchesInliers = vbLocalInliers;
                        bestScore = localScore;
                    }

                    maxIters = UpdateNumIters(param2, (double)(N - inlierNum) / N, modelNum, maxIters);
                }
            }

            for (int k = 0; k < 9; ++k)
            {
                F[k] = F21.at<double>(k);
            }

            for (int l = 0; l < N; ++l)
            {
                if (vbMatchesInliers[l])
                    mask->at(l) = true;
            }
        }
    };
//    USE_ESTIMATOR_PLUGIN(EstimatorLORANSAC);

}

//#endif
