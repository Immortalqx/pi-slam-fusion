
#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/linear_solver_eigen.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/solvers/linear_solver_dense.h"
#include "sophus/types.hpp"
#include "sophus/sim3.hpp"
#include "Optimizer.h"
#include <GSLAM/core/Timer.h>

using namespace std;
using namespace GSLAM;
using namespace Sophus;

// Update pose with 3D-2D corrospondences

bool OptimizerG2O::optimizePnP(const std::vector<std::pair<GSLAM::Point3d,CameraAnchor> >& matches,
                               GSLAM::SE3& pose,KeyFrameEstimzationDOF dof,double* information){
    // Get Calibration Parameters for later projection
    float fx=1,fy=1,cx=0,cy=0;

    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();

    g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    optimizer.setVerbose(false);

    // SET FRAME VERTEX
    Sophus::VertexSE3 * vSE3 = new Sophus::VertexSE3();
    vSE3->setEstimate(pose);
    vSE3->setId(0);
    vSE3->setFixed(false);
    optimizer.addVertex(vSE3);

    // SET MAP POINT VERTICES
    vector<OptimizableGraph::Edge*> vpEdges;
    vector<Sophus::VertexXYZ*>      vVertices;

    const int N = matches.size();
    vpEdges.reserve(N);
    vVertices.reserve(N);
    vector<bool> mvbOutlier(N,false);

    const float threshold= 1e-5;
    const float delta    = sqrt(threshold);
    Eigen::Matrix<double,2,1> obs;

    for(int i=0; i<N; i++)
    {
        std::pair<GSLAM::Point3d,CameraAnchor> mt=matches[i];
        if(mt.first.z>0)
        {
            Sophus::VertexXYZ* vPoint = new Sophus::VertexXYZ();
            vPoint->setEstimate(*((g2o::Vector3d*)&mt.first));
            vPoint->setId(i+1);
            vPoint->setFixed(true);
            optimizer.addVertex(vPoint);
            vVertices.push_back(vPoint);

            //SET EDGE
            obs(0,0)=mt.second.x;
            obs(1,0)=mt.second.y;

            Sophus::EdgeSE3PinHoleXYZ* e = new Sophus::EdgeSE3PinHoleXYZ();

            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(i+1)));
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
            e->setMeasurement(obs);
            e->setInformation(Eigen::Matrix2d::Identity());

            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            rk->setDelta(delta);

            e->fx = fx;
            e->fy = fy;
            e->cx = cx;
            e->cy = cy;

            optimizer.addEdge(e);

            vpEdges.push_back(e);
        }
        else if(0){
            Sophus::EdgeSE3Epipolar* e=new Sophus::EdgeSE3Epipolar();
            e->setVertex(0,dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
            //SET EDGE
            obs(0,0)=mt.second.x;
            obs(1,0)=mt.second.y;
            e->setMeasurement(obs);
            e->setInformation(Sophus::EdgeSE3Epipolar::InformationType(1.));

            e->p=mt.first;

            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            rk->setDelta(delta);


            optimizer.addEdge(e);
            vpEdges.push_back(e);
        }

    }

    {
        GSLAM::ScopedTimer tm("OptimizerG2O::optimizePnP");
        optimizer.initializeOptimization();
        optimizer.optimize(30);
    }

//    // We perform 4 optimizations, decreasing the inlier region
//    // From second to final optimization we include only inliers in the optimization
//    // At the end of each optimization we check which points are inliers
////    const float chi2[4]={9.210,7.378,5.991,5.991};
//    const float chi2[4]={3e-5,2e-5,1e-5,1e-5};
//    int its[4]={10,10,7,5};

//    int nBad=0;
//    for(size_t it=0; it<4; it++)
//    {
//        optimizer.initializeOptimization();
//        optimizer.optimize(its[it]);

//        nBad=0;
//        for(size_t i=0, iend=vpEdges.size(); i<iend; i++)
//        {
//            Sophus::EdgeSE3PinHoleXYZ* e = dynamic_cast<Sophus::EdgeSE3PinHoleXYZ*>(vpEdges[i]);

//            const size_t idx = i;

//            if(mvbOutlier[idx])
//            {
//                e->setInformation(Eigen::Matrix2d::Identity());
//                e->computeError();
//            }

//            if(e->chi2()>chi2[it])
//            {
//                mvbOutlier[idx]=true;
//                e->setInformation(Eigen::Matrix2d::Identity()*1e-10);
//                nBad++;
//            }
//            else// if(e->chi2()<=chi2[it])
//            {
//                mvbOutlier[idx]=false;
//            }
//        }

//        if(optimizer.edges().size()<10)
//            break;
//    }

    pose=vSE3->estimate();

    return true;
}


bool OptimizerG2O::optimizeSE3Graph(GSLAM::BundleGraph& graph)
{
//    // Setup optimizer
//    g2o::SparseOptimizer optimizer;
//    g2o::BlockSolverX::LinearSolverType * linearSolver;

//    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

//    g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);

//    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
//    optimizer.setAlgorithm(solver);

    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(false);
    g2o::BlockSolver_6_3::LinearSolverType * linearSolver =
           new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();
    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

    optimizer.setAlgorithm(solver);

    solver->setUserLambdaInit(1e-16);

    std::vector<Sophus::VertexSE3*> vertexes;
    vertexes.reserve(graph.keyframes.size());
    for(int i=0;i<graph.keyframes.size();i++){
        Sophus::VertexSE3* se3=new Sophus::VertexSE3();
        se3->setId(i);
        se3->setFixed(graph.keyframes[i].dof==GSLAM::UPDATE_KF_NONE);
        GSLAM::SE3 poseInv=graph.keyframes[i].estimation.get_se3();
        se3->setEstimate(poseInv);
        optimizer.addVertex(se3);
        vertexes.push_back(se3);
    }
    for(GSLAM::SE3Edge e:graph.se3Graph){
        Sophus::EdgeSE3* edge=new Sophus::EdgeSE3();
        edge->setMeasurement(e.measurement);
        edge->setVertex(0,dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(e.firstId)));
        edge->setVertex(1,dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(e.secondId)));
        edge->setInformation(Eigen::Matrix<double,6,6>::Identity());
        optimizer.addEdge(edge);
    }

    optimizer.initializeOptimization();
    optimizer.optimize(20);

    // Recover optimized data

    //Keyframes
    for(size_t i=0,iend=graph.keyframes.size();i<iend;i++)
    {
        KeyFrameEstimzation& kf=graph.keyframes[i];
        VertexSE3* vSE3 = static_cast<VertexSE3*>(optimizer.vertex(i));
        kf.estimation.get_se3()=vSE3->estimate();
    }
    return true;
}

bool OptimizerG2O::optimize(BundleGraph& graph){
//    if(graph.se3Graph.size()) return optimizeSE3Graph(graph);
    if(graph.invDepthObserves.size()||graph.sim3Graph.size())
    {
        LOG(ERROR)<<"OptimizerG2O can only handle mappoint bundle adjust problem!";
    }

    // Get Calibration Parameters for later projection
    float fx=1,fy=1,cx=0,cy=0;

    GSLAM::Camera cam=graph.camera;
    if(cam.isValid())
    {
        pi::Point2d cxcy = cam.Project(pi::Point3d(0, 0, 1));
        pi::Point2d fxfy = cam.Project(pi::Point3d(1, 1, 1)) - cxcy;

        fx = fxfy.x;
        fy = fxfy.y;
        cx = cxcy.x;
        cy = cxcy.y;
    }

    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

    g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    bool* pbStopFlag=(bool*)svar.GetPointer("BundleStopFlagPointer");
    if(pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    long unsigned int maxKFid = 0;

    // SET KEYFRAME VERTICES
    for(KeyFrameEstimzation& it:graph.keyframes)
    {
        VertexSE3* vSE3 = new VertexSE3();
        vSE3->setEstimate(it.estimation.get_se3().inverse());
        vSE3->setFixed(it.dof==UPDATE_KF_NONE);
        vSE3->setId(maxKFid++);
        optimizer.addVertex(vSE3);
    }

    // SET MAP POINT VERTICES
    long unsigned int pointStartId=maxKFid;
    for(MapPointEstimation& it:graph.mappoints)
    {
        VertexXYZ* vPoint = new VertexXYZ();
        vPoint->setEstimate(*((g2o::Vector3d*)&it.first));
        vPoint->setId(maxKFid++);
        optimizer.addVertex(vPoint);
    }
    // SET UP EDGES

    vector<EdgeSE3PinHoleXYZ*> vpEdges;
    vector<size_t> outliers;
    vpEdges.reserve(graph.mappointObserves.size());

    const float threshold=cam.isValid()?5.991:1e-5;
    const float thHuber = sqrt(threshold);

    for(BundleEdge& it:graph.mappointObserves)
    {
        Eigen::Matrix<double,2,1> obs;
        obs(0,0)=it.measurement.x;
        obs(1,0)=it.measurement.y;

        EdgeSE3PinHoleXYZ* e = new EdgeSE3PinHoleXYZ();

        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(it.pointId+pointStartId)));
        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(it.frameId)));
        e->setMeasurement(obs);
        e->setInformation(Eigen::Matrix2d::Identity());
        e->cx=cx;e->cy=cy;e->fx=fx;e->fy=fy;

        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
        e->setRobustKernel(rk);
        rk->setDelta(thHuber);

        optimizer.addEdge(e);
        vpEdges.push_back(e);
    }

    //        optimizer.initializeOptimization();
    //        optimizer.optimize(5);

    //        // Check inlier observations
    //        for(size_t i=0, iend=vpEdges.size(); i<iend;i++)
    //        {
    //            EdgeSE3PinHoleXYZ* e = vpEdges[i];

    //            if(e->chi2()>threshold || !e->isDepthPositive())
    //            {
    //                optimizer.removeEdge(e);
    //                vpEdges[i]=NULL;
    //                outliers.push_back(i);
    //            }
    //        }

    // GPS VERTICES AND EDGES
    for(GPSEdge& it:graph.gpsGraph)
    {
        VertexSE3* fr=dynamic_cast<VertexSE3*>(optimizer.vertex(it.frameId));

        if(fr->fixed()) continue;

        EdgeSE3GPS* e=new EdgeSE3GPS();

        e->setVertex(0, fr);

        Matrix6d infoMat=Matrix6d::Identity()*0.01;
        if(it.information)
        {
            infoMat=Eigen::Map<Matrix6d>(it.information);
        }
        e->setInformation(infoMat);
        Sophus::SE3d measure;//(it.measurement);

        for(int i=0;i<7;i++) measure.data()[i]=((double*)&it.measurement)[i];
        e->setMeasurement(measure);// FIXME: Why segment fault when put it before set information

//        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
//        e->setRobustKernel(rk);
//        rk->setDelta(0.1);
        e->computeError();

        if(svar.GetInt("Optimizer.Verbose"))
            LOG(INFO)<<"infor:"<<infoMat<<",chi2:"<<e->chi2();

        optimizer.addEdge(e);
    }

    for(GSLAM::SE3Edge e:graph.se3Graph){
        Sophus::EdgeSE3* edge=new Sophus::EdgeSE3();
        edge->setMeasurement(e.measurement);
        edge->setVertex(0,dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(e.firstId)));
        edge->setVertex(1,dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(e.secondId)));
        Matrix6d infoMat=Matrix6d::Identity()*0.01;
        if(e.information)
        {
            infoMat=Eigen::Map<Matrix6d>(e.information);
        }
        edge->setInformation(infoMat);
        optimizer.addEdge(edge);
    }

    {
        GSLAM::ScopedTimer tm("OptimizerG2O::optimize");
        optimizer.setVerbose(svar.GetInt("Optimizer.Verbose"));
        optimizer.initializeOptimization();
        optimizer.optimize(50);
    }

    //        for(SIM3Edge& it:graph.sim3Graph){
    //            EdgeSim3* e=new EdgeSim3();
    //            e->setVertex(0,dynamic_cast<VertexSim3*>(optimizer.vertex(it.frameId)));
    //            VertexSE3* fr=;
    //            if(fr->fixed()) continue;
    //            EdgeSE3GPS* e=new EdgeSE3GPS();
    //            e->setVertex(0, fr);
    //            e->setMeasurement(it.measurement);

    ////            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
    ////            e->setRobustKernel(rk);
    ////            rk->setDelta(20);

    //            Matrix6d infoMat=Matrix6d::Identity();
    //            if(it.information)
    //            {
    //                infoMat=Eigen::Map<Matrix6d>(it.information);
    //            }
    //            e->setInformation(infoMat);
    //            if(infoPrint)
    //            {
    //                infoPrint=false;
    //                LOG(INFO)<<infoMat;
    //            }
    //            optimizer.addEdge(e);
    //            gpsEdges.push_back(e);
    //        }

    // Recover optimized data

    //Keyframes
    for(size_t i=0,iend=graph.keyframes.size();i<iend;i++)
    {
        KeyFrameEstimzation& kf=graph.keyframes[i];
        VertexSE3* vSE3 = static_cast<VertexSE3*>(optimizer.vertex(i));
        kf.estimation.get_se3()=vSE3->estimate().inverse();
    }

    //Points
    for(size_t i=0,iend=graph.mappoints.size();i<iend;i++)
    {
        MapPointEstimation& pt=graph.mappoints[i];
        VertexXYZ* vPoint = static_cast<VertexXYZ*>(optimizer.vertex(pointStartId+i));
        g2o::Vector3d vec3=vPoint->estimate();
        pt.first=*((pi::Point3d*)&vec3);
    }
    return true;
}

bool OptimizerG2O::optimizePose(std::vector<std::pair<GSLAM::CameraAnchor,GSLAM::CameraAnchor> >& matches,
                          std::vector<GSLAM::IdepthEstimation>& firstIDepth,GSLAM::SE3&    relativePose,// T_{12}
                          GSLAM::KeyFrameEstimzationDOF dof,double* information)
{
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();

    g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    optimizer.setVerbose(false);

    // SET FRAME VERTEX
    Sophus::VertexSE3 * vSE3 = new Sophus::VertexSE3();
    vSE3->setEstimate(relativePose.inverse());
    vSE3->setId(0);
    vSE3->setFixed(false);
    optimizer.addVertex(vSE3);

    const float threshold= 1e-5;
    const float delta    = sqrt(threshold);

    std::vector<Sophus::VertexIdepth*> vertexIdepths;
    vertexIdepths.reserve(matches.size());
    for(int i=0;i<matches.size();i++){
        Sophus::VertexIdepth* idepth=new Sophus::VertexIdepth();
        idepth->setEstimate(firstIDepth[i].x);
        idepth->setId(i+1);
        if(firstIDepth[i].y>10) idepth->setFixed(true);
        optimizer.addVertex(idepth);
        vertexIdepths.push_back(idepth);

        Sophus::EdgeSE3InvDepth* edge=new Sophus::EdgeSE3InvDepth();
        edge->setMeasurement((double*)&matches[i]);
        edge->setVertex(0,vSE3);
        edge->setVertex(1,idepth);
        edge->setInformation(Eigen::Matrix2d::Identity());

        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
        edge->setRobustKernel(rk);
        rk->setDelta(delta);

        optimizer.addEdge(edge);
    }

    optimizer.initializeOptimization();
    optimizer.optimize(50);

    relativePose=vSE3->estimate().inverse();

    for(int i=0;i<matches.size();i++)
        firstIDepth[i].x=vertexIdepths[i]->estimate();
    return true;
}



USE_OPTIMIZER_PLUGIN(OptimizerG2O);

