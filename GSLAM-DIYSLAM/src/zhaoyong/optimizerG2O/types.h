#pragma once

#include "g2o/core/base_vertex.h"
#include "g2o/core/base_binary_edge.h"
#include "g2o/core/base_unary_edge.h"
#include "g2o/types/sim3.h"
#include "g2o/types/se3quat.h"

using namespace Eigen;
using namespace g2o;

typedef Matrix<double, 6, 6> Matrix6d;
typedef Matrix<double, 7, 7> Matrix7d;
typedef Matrix<double, 6, 1> Vector6d;
typedef Matrix<double, 7, 1> Vector7d;

class VertexSim3 : public BaseVertex<7, Sim3>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  VertexSim3(): BaseVertex<7, Sim3>()
{
  _marginalized=false;
  _fix_scale = false;
}

  virtual bool read(std::istream& is)
{
  Vector7d cam2world;
  for (int i=0; i<6; i++){
    is >> cam2world[i];
  }
  is >> cam2world[6];
  for (int i=0; i<2; i++)
  {
    is >> _focal_length1[i];
  }
  for (int i=0; i<2; i++)
  {
    is >> _principle_point1[i];
  }

  setEstimate(Sim3(cam2world).inverse());
  return true;
}

  virtual bool write(std::ostream& os) const
{
  Sim3 cam2world(estimate().inverse());
  Vector7d lv=cam2world.log();
  for (int i=0; i<7; i++){
    os << lv[i] << " ";
  }
  for (int i=0; i<2; i++)
  {
    os << _focal_length1[i] << " ";
  }
  for (int i=0; i<2; i++)
  {
    os << _principle_point1[i] << " ";
  }
  return os.good();
}


  virtual void setToOriginImpl() {
    _estimate = Sim3();
  }

  virtual void oplusImpl(const double* update_)
  {
    Eigen::Map<Vector7d> update(const_cast<double*>(update_));

    if (_fix_scale)
      update[6] = 0;

    Sim3 s(update);
    setEstimate(s*estimate());
  }

  Vector2d _principle_point1, _principle_point2;
  Vector2d _focal_length1, _focal_length2;

  Vector2d cam_map1(const Vector2d & v) const
  {
    Vector2d res;
    res[0] = v[0]*_focal_length1[0] + _principle_point1[0];
    res[1] = v[1]*_focal_length1[1] + _principle_point1[1];
    return res;
  }

  Vector2d cam_map2(const Vector2d & v) const
  {
    Vector2d res;
    res[0] = v[0]*_focal_length2[0] + _principle_point2[0];
    res[1] = v[1]*_focal_length2[1] + _principle_point2[1];
    return res;
  }

  bool _fix_scale;
};

class  VertexSE3 : public g2o::BaseVertex<6, g2o::SE3Quat>{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  VertexSE3(): BaseVertex<6, SE3Quat>() {
  }


  bool read(std::istream& is){
      Vector7d est;
      for (int i=0; i<7; i++)
        is  >> est[i];
      SE3Quat cam2world;
      cam2world.fromVector(est);
      setEstimate(cam2world.inverse());
      return true;
    }

  bool write(std::ostream& os) const{
      SE3Quat cam2world(estimate().inverse());
      for (int i=0; i<7; i++)
        os << cam2world[i] << " ";
      return os.good();
    }

  virtual void setToOriginImpl() {
    _estimate = SE3Quat();
  }

  virtual void oplusImpl(const double* update_)  {
    Eigen::Map<const Vector6d> update(update_);
    setEstimate(SE3Quat::exp(update)*estimate());
  }
};

class VertexXYZ : public BaseVertex<3, Vector3d>
{
 public:
   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
   VertexXYZ(): BaseVertex<3, Vector3d>()
   {
   }

   virtual bool read(std::istream& is)
   {
     Vector3d lv;
     for (int i=0; i<3; i++)
       is >> _estimate[i];
     return true;
   }
   virtual bool write(std::ostream& os) const
   {
     Vector3d lv=estimate();
     for (int i=0; i<3; i++){
       os << lv[i] << " ";
     }
     return os.good();
   }

   virtual void setToOriginImpl() {
     _estimate.fill(0.);
   }

   virtual void oplusImpl(const double* update)
   {
     Eigen::Map<const Vector3d> v(update);
     _estimate += v;
   }
};

class VertexIdepth : public BaseVertex<1, double>
{
 public:
   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
   VertexIdepth(): BaseVertex<1, double>()
   {
   }

   virtual bool read(std::istream& is)
   {
       return false;
   }
   virtual bool write(std::ostream& os) const
   {
       return false;
   }

   virtual void setToOriginImpl() {
     _estimate=0;
   }

   virtual void oplusImpl(const double* update)
   {
     _estimate += *update;
   }
};

class  EdgeSE3IdealXYZ: public  BaseBinaryEdge<2, Vector2d, VertexXYZ, VertexSE3>{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeSE3IdealXYZ(): BaseBinaryEdge<2, Vector2d, VertexXYZ, VertexSE3>(){
  }

  bool read(std::istream& is){
      for (int i=0; i<2; i++){
        is >> _measurement[i];
      }
      for (int i=0; i<2; i++)
        for (int j=i; j<2; j++) {
          is >> information()(i,j);
          if (i!=j)
            information()(j,i)=information()(i,j);
        }
      return true;
    }

  bool write(std::ostream& os) const{

      for (int i=0; i<2; i++){
        os << measurement()[i] << " ";
      }

      for (int i=0; i<2; i++)
        for (int j=i; j<2; j++){
          os << " " <<  information()(i,j);
        }
      return os.good();
    }


  Vector3d pointLocal(){
      const VertexSE3* v1 = static_cast<const VertexSE3*>(_vertices[1]);
      const VertexXYZ* v2 = static_cast<const VertexXYZ*>(_vertices[0]);
      return v1->estimate().map(v2->estimate());
  }

  bool isDepthPositive() {
    return pointLocal()(2)>0.0;
  }

  void computeError()  {
    Vector2d obs(_measurement);
    _error = obs-cam_project(pointLocal());
  }

  virtual void linearizeOplus(){
      VertexXYZ* vi = static_cast<VertexXYZ*>(_vertices[0]);
      VertexSE3* vj = static_cast<VertexSE3*>(_vertices[1]);
      SE3Quat T(vj->estimate());
      Vector3d xyz = vi->estimate();
      Vector3d xyz_trans = T.map(xyz);

      double x = xyz_trans[0];
      double y = xyz_trans[1];
      double invz = 1./xyz_trans[2];
      double invz_2 = invz*invz;

      Matrix<double,2,3> tmp;
      tmp(0,0) = -1;
      tmp(0,1) = 0;
      tmp(0,2) = x*invz;

      tmp(1,0) = 0;
      tmp(1,1) = -1;
      tmp(1,2) = y*invz;

      _jacobianOplusXi = tmp * T.rotation().toRotationMatrix();

      _jacobianOplusXj(0,0) =  x*y*invz_2;
      _jacobianOplusXj(0,1) = -(1+(x*x*invz_2));
      _jacobianOplusXj(0,2) = y*invz;
      _jacobianOplusXj(0,3) = -invz;
      _jacobianOplusXj(0,4) = 0;
      _jacobianOplusXj(0,5) = x*invz_2;

      _jacobianOplusXj(1,0) = (1+y*y*invz_2);
      _jacobianOplusXj(1,1) = -x*y*invz_2;
      _jacobianOplusXj(1,2) = -x*invz;
      _jacobianOplusXj(1,3) = 0;
      _jacobianOplusXj(1,4) = -invz;
      _jacobianOplusXj(1,5) = y*invz_2;
    }


  static Vector2d project2d(const Vector3d& v)  {
    Vector2d res;
    res(0) = v(0)/v(2);
    res(1) = v(1)/v(2);
    return res;
  }

  static Vector3d unproject2d(const Vector2d& v)  {
    Vector3d res;
    res(0) = v(0);
    res(1) = v(1);
    res(2) = 1;
    return res;
  }

  virtual Vector2d cam_project(const Vector3d & trans_xyz) const{
      return project2d(trans_xyz);
    }
};
class  EdgeSE3PinHoleXYZ: public  EdgeSE3IdealXYZ{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeSE3PinHoleXYZ(): fx(1),fy(1),cx(0),cy(0) {
  }

  virtual void linearizeOplus(){
      VertexXYZ* vi = static_cast<VertexXYZ*>(_vertices[0]);
      VertexSE3* vj = static_cast<VertexSE3*>(_vertices[1]);
      SE3Quat T(vj->estimate());
      Vector3d xyz = vi->estimate();
      Vector3d xyz_trans = T.map(xyz);

      double x = xyz_trans[0];
      double y = xyz_trans[1];
      double z = xyz_trans[2];
      double z_2 = z*z;

      Matrix<double,2,3> tmp;
      tmp(0,0) = fx;
      tmp(0,1) = 0;
      tmp(0,2) = -x/z*fx;

      tmp(1,0) = 0;
      tmp(1,1) = fy;
      tmp(1,2) = -y/z*fy;

      _jacobianOplusXi =  -1./z * tmp * T.rotation().toRotationMatrix();

      _jacobianOplusXj(0,0) =  x*y/z_2 *fx;
      _jacobianOplusXj(0,1) = -(1+(x*x/z_2)) *fx;
      _jacobianOplusXj(0,2) = y/z *fx;
      _jacobianOplusXj(0,3) = -1./z *fx;
      _jacobianOplusXj(0,4) = 0;
      _jacobianOplusXj(0,5) = x/z_2 *fx;

      _jacobianOplusXj(1,0) = (1+y*y/z_2) *fy;
      _jacobianOplusXj(1,1) = -x*y/z_2 *fy;
      _jacobianOplusXj(1,2) = -x/z *fy;
      _jacobianOplusXj(1,3) = 0;
      _jacobianOplusXj(1,4) = -1./z *fy;
      _jacobianOplusXj(1,5) = y/z_2 *fy;
    }

  virtual Vector2d cam_project(const Vector3d & trans_xyz) const{
      Vector2d proj = project2d(trans_xyz);
      Vector2d res;
      res[0] = proj[0]*fx + cx;
      res[1] = proj[1]*fy + cy;
      return res;
    }

  double fx, fy, cx, cy;
};

class EdgeSE3 : public BaseBinaryEdge<6, SE3Quat, VertexSE3, VertexSE3>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeSE3(): BaseBinaryEdge<6, SE3Quat, VertexSE3, VertexSE3>(){
  }

  bool read(std::istream& is){
      return false;
    }

  bool write(std::ostream& os) const{
      return false;
    }

  void computeError()
  {
      const VertexSE3* v1 = static_cast<const VertexSE3*>(_vertices[0]);
      const VertexSE3* v2 = static_cast<const VertexSE3*>(_vertices[1]);

      SE3Quat C(_measurement);
      SE3Quat error_= v2->estimate().inverse()*C*v1->estimate();
      _error = error_.log();
  }

  void linearizeOplus() {
    VertexSE3 * vi = static_cast<VertexSE3 *>(_vertices[0]);
    SE3Quat Ti(vi->estimate());

    VertexSE3 * vj = static_cast<VertexSE3 *>(_vertices[1]);
    SE3Quat Tj(vj->estimate());

    const SE3Quat & Tij = _measurement;
    SE3Quat invTij = Tij.inverse();

    SE3Quat invTj_Tij = Tj.inverse()*Tij;
    SE3Quat infTi_invTij = Ti.inverse()*invTij;

    _jacobianOplusXi = invTj_Tij.adj();
    _jacobianOplusXj = -infTi_invTij.adj();
  }

  virtual double initialEstimatePossible(const OptimizableGraph::VertexSet& , OptimizableGraph::Vertex* ) { return 1.;}
  virtual void initialEstimate(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* /*to*/)
  {
    VertexSE3* v1 = static_cast<VertexSE3*>(_vertices[0]);
    VertexSE3* v2 = static_cast<VertexSE3*>(_vertices[1]);
    if (from.count(v1) > 0)
        v2->setEstimate(measurement()*v1->estimate());
    else
        v1->setEstimate(measurement().inverse()*v2->estimate());
  }
};

class EdgeSim3 : public BaseBinaryEdge<7, Sim3, VertexSim3, VertexSim3>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgeSim3():
      BaseBinaryEdge<7, Sim3, VertexSim3, VertexSim3>()
  {
  }
  virtual bool read(std::istream& is){return false;}
  virtual bool write(std::ostream& os) const{return false;}
  void computeError()
  {
    const VertexSim3* v1 = static_cast<const VertexSim3*>(_vertices[0]);
    const VertexSim3* v2 = static_cast<const VertexSim3*>(_vertices[1]);

    Sim3 C(_measurement);
    Sim3 error_=C*v1->estimate()*v2->estimate().inverse();
    _error = error_.log();
  }

  virtual double initialEstimatePossible(const OptimizableGraph::VertexSet& , OptimizableGraph::Vertex* ) { return 1.;}
  virtual void initialEstimate(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* /*to*/)
  {
    VertexSim3* v1 = static_cast<VertexSim3*>(_vertices[0]);
    VertexSim3* v2 = static_cast<VertexSim3*>(_vertices[1]);
    if (from.count(v1) > 0)
        v2->setEstimate(measurement()*v1->estimate());
    else
        v1->setEstimate(measurement().inverse()*v2->estimate());
  }
};

class EdgeSE3GPS : public  BaseUnaryEdge<6, SE3Quat, VertexSE3>{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeSE3GPS(): BaseUnaryEdge<6, SE3Quat, VertexSE3>(){
  }

  bool read(std::istream& is){
      return false;
    }

  bool write(std::ostream& os) const{
      return false;
    }

  void computeError()  {
      const VertexSE3* fr = static_cast<const VertexSE3*>(_vertices[0]);
      _error=(_measurement*fr->estimate()).log();
//      <<e(3),e(4),e(5),e(0),e(1),e(2);
//          auto e = obs.translation()-fr->estimate().inverse().translation();
//          _error<<e(0),e(1),e(2),0,0,0;//e(3),e(4),e(5),
  }

    virtual void linearizeOplus(){
//      VertexSE3 * vi = static_cast<VertexSE3 *>(_vertices[0]);
//      SE3Quat Ti(vi->estimate());

//      SE3Quat Tj;

//      const SE3Quat & Tij = _measurement;
//      SE3Quat invTij = Tij.inverse();

//      SE3Quat invTj_Tij = Tij;
//      SE3Quat infTi_invTij = Ti.inverse()*invTij;

      _jacobianOplusXi = _measurement.adj();
//      _jacobianOplusXj = -infTi_invTij.adj();
    }
};

class EdgeSE3Epipolar : public BaseUnaryEdge<2, Vector2d, VertexSE3>{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeSE3Epipolar(): BaseUnaryEdge<2, Vector2d, VertexSE3>(){
  }

  bool read(std::istream& is){
      return false;
    }

  bool write(std::ostream& os) const{
      return false;
    }

  void computeError()  {
//    const VertexSE3* fr = static_cast<const VertexSE3*>(_vertices[0]);
//    Measurement obs(_measurement);
//    _error = (obs*fr->estimate()).log();
  }
};

class EdgeSE3InvDepth : public BaseBinaryEdge<2, double*, VertexSE3,VertexIdepth>{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeSE3InvDepth(): BaseBinaryEdge<2, double*, VertexSE3,VertexIdepth>(){
  }

  bool read(std::istream& is){
      return false;
    }

  bool write(std::ostream& os) const{
      return false;
    }

  void computeError()  {
    const VertexSE3*    fr = static_cast<const VertexSE3*>(_vertices[0]);
    const VertexIdepth* idepth= static_cast<const VertexIdepth*>(_vertices[1]);
    Measurement match;
    const Vector3d   p1(match[0],match[1],match[2]);
    auto p=fr->estimate().map(p1/idepth->estimate());

    const double invW=match[5]/p[2];
    _error=Vector2d(invW*p[0]-match[3],invW*p[1]-match[4]);
  }
};

