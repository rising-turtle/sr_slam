#include "g2o_copy_graph.h"
#include "g2o/types/slam3d/edge_se3.h"
#include "edge_se3_euler2_zh.cpp"

namespace g2o{

  void jac_quat3_euler3(Eigen::Matrix<double, 6, 6, Eigen::ColMajor>& J, const Isometry3D& t)
  {
    Vector7d t0 = g2o::internal::toVectorQT(t);

    double delta=1e-6;
    double idelta= 1. / (2. * delta);

    Vector7d ta = t0;
    Vector7d tb = t0;
    for (int i=0; i<6; i++){
      ta=tb=t0;
      ta[i]-=delta;
      tb[i]+=delta;
      Vector6d ea = g2o::internal::toVectorET(g2o::internal::fromVectorQT(ta));
      Vector6d eb = g2o::internal::toVectorET(g2o::internal::fromVectorQT(tb));
      // J.col(3)=(eb-ea)*idelta;
      J.col(i) = (eb-ea)*idelta;
    }
  }

template<>
EdgeSE3 * copyEdge(EdgeSE3Euler2ZH* pin, SparseOptimizer*g)
{
  EdgeSE3 * er = new EdgeSE3; 
  int id1 = (pin->vertices()[0])->id(); 
  int id2 = (pin->vertices()[1])->id(); 
  er->vertices()[0] = g->vertex(id1); 
  er->vertices()[1] = g->vertex(id2); 
  er->setMeasurement(pin->measurement());
  // information matrix has to be recomputed 
  Matrix<double, 6, 6, Eigen::ColMajor> J; 
  jac_quat3_euler3(J, pin->measurement()); 
  Matrix<double, 6, 6, Eigen::ColMajor> infMat = J.transpose() * pin->information() *J ; 
  er->setInformation(infMat); 
  return er; 
}

template<>
EdgeSE3Euler2ZH * copyEdge(EdgeSE3* pin, SparseOptimizer* g)
{
  EdgeSE3Euler2ZH * er = new EdgeSE3Euler2ZH; 
  int id1 = (pin->vertices()[0])->id(); 
  int id2 = (pin->vertices()[1])->id(); 
  er->vertices()[0] = g->vertex(id1); 
  er->vertices()[1] = g->vertex(id2); 
  er->setMeasurement(pin->measurement());
  // information matrix has to be recomputed 
  Matrix<double, 6, 6, Eigen::ColMajor> J; 
  jac_quat3_euler3(J, pin->measurement()); 
  // invert tje jacobian to simulate the inverse derivative 
  J = J.inverse(); 
  Matrix<double, 6, 6, Eigen::ColMajor> infMat = J.transpose() * pin->information() *J ; 
  er->setInformation(infMat); 
  return er; 
}

}
