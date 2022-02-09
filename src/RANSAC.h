#ifndef RANSAC_RANSAC_H
#define RANSAC_RANSAC_H

#include <string>
#include <vector>
#include <cmath>
#include <GSLAM/core/SE3.h>

#define ransac RANSAC::Instance()

class RANSAC
{
public:
    // 获取单实例对象
    static RANSAC &Instance();

    //进行求解
    void solve(pi::Point3d point3D);

    //是否完成计算
    bool is_finished() const;

private:
    RANSAC();

    /**
     * 求解点M到平面的距离
     * @param M 点M
     * @param P 平面上一点P
     * @param N 平面的法向量
     */
    static double solve_distance(pi::Point3d M, pi::Point3d P, pi::Point3d N);

    /**
     * 根据三点求解平面方程
     * @param A 点A
     * @param B 点B
     * @param C 点C
     */
    void solve_plane(pi::Point3d A, pi::Point3d B, pi::Point3d C);

    /**
     * RANSAC核心算法
     */
    void ransac_core();

private:
    //保存的点集
    std::vector<pi::Point3d> points;
    //计算得到的平面上的一点
    pi::Point3d plane_P;
    //计算得到的平面的四元数
    pi::SO3d plane_Q;
    //计算得到的平面的法向量
    pi::Point3d plane_N;
    //是否完成计算
    bool finished;
};

#endif //RANSAC_RANSAC_H
