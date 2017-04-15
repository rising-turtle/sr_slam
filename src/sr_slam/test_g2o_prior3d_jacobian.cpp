#include "g2o/core/jacobian_workspace.h"
#include "g2o/stuff/macros.h"
#include "g2o/types/slam3d/edge_se3.h"
#include "g2o/types/slam3d/edge_se3_prior.h"
#include "edge_se3_prior_zh.h"
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "g2o/core/sparse_optimizer.h"

using namespace std;
using namespace g2o;
using namespace Eigen;

void printIsometryMatrix(Isometry3D m, const char* prefix = "");
void printSquareMatrix(double * m, int l = 6, const char * prefix = "");

Isometry3D randomIsometry3d()
{
  Vector3D rotAxisAngle = Vector3D::Random();
  rotAxisAngle += Vector3D::Random();
  Eigen::AngleAxisd rotation(rotAxisAngle.norm(), rotAxisAngle.normalized());
  Isometry3D result = (Isometry3D)rotation.toRotationMatrix();
  result.translation() = Vector3D::Random();
  return result;
}

void test1();
void test2();
void test3();
void test4();
Vector6d computeError(Isometry3D& Z, Isometry3D& Xi, Isometry3D& Xj);

int main(int argc, char* argv[])
{
  test4(); 
  // test1();
  // test2(); 
  // test3();
  return 0;
}

Vector3d computetrans(Isometry3D Z, Isometry3D Xi, Isometry3D Xj)
{
  Isometry3D iXi = Xi.inverse(); 
  // Matrix<double, 3, 1> i1 = iXi.block<3,1>(0,0); 
  // Matrix<double, 3, 1> ti = Xi.block<3,1>(3, 0);

  Matrix3d Rz = g2o::internal::extractRotation(Z); 
  Matrix3d RXi = g2o::internal::extractRotation(Xi); 
  Matrix3d RXj = g2o::internal::extractRotation(Xj); 
  Matrix3d RiXi = g2o::internal::extractRotation(iXi); 
  Vector3d tz = Z.translation(); 
  Vector3d txi = Xi.translation(); 
  Vector3d txj = Xj.translation(); 

  // Vector3d i1 = iXi.block<3,1>(0,0); 
  // Vector3d ti = Xi.block<3,1>(3,0); 
  Vector3d tixi = iXi.translation(); 
  double x1, y1, z1; 
  Vector3d RiXi1 = RXi.block<3,1>(0,0); 
  Vector3d RiXi2 = RXi.block<3,1>(0,1); 
  Vector3d RiXi3 = RXi.block<3,1>(0,2); 

  // printIsometryMatrix(Xi, "Xi: ");
  // cout<<"RiXi1: "<<RiXi1<<endl<<"RiXi2: "<<RiXi2<<endl<<"RiXi3: "<<RiXi3<<endl;
  
  x1 = -RiXi1.dot(txi.transpose());
  y1 = -RiXi2.dot(txi.transpose());
  z1 = -RiXi3.dot(txi.transpose()); 

  Vector3d tiXiXj; 
  tiXiXj(0) = x1 + RiXi1.dot(txj.transpose());
  tiXiXj(1) = y1 + RiXi2.dot(txj.transpose()); 
  tiXiXj(2) = z1 + RiXi3.dot(txj.transpose());
  Isometry3D iXiXj = iXi * Xj; 

  printIsometryMatrix(iXiXj, "iXiXj: "); 
  // cout<<"check itxi: "<<x1<<" "<<y1<<" "<<z1<<endl;
  cout<<"check tiXiXj: "<<tiXiXj<<endl;
  Vector3d tiXiXj2; 
  Vector3d tt = txj - txi; 
  tiXiXj2(0) = RiXi1.dot(tt.transpose()); 
  tiXiXj2(1) = RiXi2.dot(tt.transpose());
  tiXiXj2(2) = RiXi3.dot(tt.transpose()); 
  cout<<"check tiXiXj2: "<<tiXiXj2<<endl;

  Vector3d et; 
  Vector3d Rz1 = Rz.block<1,3>(0,0).transpose();
  Vector3d Rz2 = Rz.block<1,3>(1,0).transpose(); 
  Vector3d Rz3 = Rz.block<1,3>(2,0).transpose(); 
  et(0) = Rz1.dot(tiXiXj.transpose()) + tz(0); 
  et(1) = Rz2.dot(tiXiXj.transpose()) + tz(1); 
  et(2) = Rz3.dot(tiXiXj.transpose()) + tz(2); 

  // et(0) = Rz1.dot(tiXiXj2.transpose()) + tz(0); 
  // cout<<"check e03 = "<<et(0)<<endl;
  return et;
}

double computeE03(Isometry3D Z, Isometry3D Xi, Isometry3D Xj)
{
  Matrix3d Rz = g2o::internal::extractRotation(Z); 
  Matrix3d Ri = g2o::internal::extractRotation(Xi);
  Vector3d Ri1 = Ri.block<3,1>(0,0); 
  Vector3d Ri2 = Ri.block<3,1>(0,1); 
  Vector3d Ri3 = Ri.block<3,1>(0,2); 

  Vector3d ti = Xi.translation();
  Vector3d tj = Xj.translation(); 
  Vector3d tz = Z.translation();
  Vector3d tij = tj - ti; 
  Vector3d Rz1 = Rz.block<1,3>(0,0).transpose(); 
  

  // double e03 = Rz1(0)*(Ri1.dot(tij.transpose())) + 
  //             Rz1(1)*(Ri2.dot(tij.transpose())) + 
  //             Rz1(2)*(Ri3.dot(tij.transpose())) + tz(0);
  double e03 = Rz(0,0)*(Ri(0,0)*tij(0) + Ri(1,0)*tij(1) + Ri(2,0)*tij(2)) + 
               Rz(0,1)*(Ri(0,1)*tij(0) + Ri(1,1)*tij(1) + Ri(2,1)*tij(2)) + 
               Rz(0,2)*(Ri(0,2)*tij(0) + Ri(1,2)*tij(1) + Ri(2,2)*tij(2));
  double tk = Rz(0,0) * Ri(0,0) + Rz(0,1)*Ri(0,1) + Rz(0,2)*Ri(0,2); 
  cout<<"check tk = "<<tk<<endl;
  return e03;
}

void test2() // test entrances in Jacobian matrix 
{
  Isometry3D Z = randomIsometry3d(); 
  Isometry3D Xi = randomIsometry3d(); 
  Isometry3D Xj = randomIsometry3d(); 
  Isometry3D iXi = Xi.inverse();

  // printIsometryMatrix(Xi, "Xi: ");
  // printIsometryMatrix(iXi, "Xi.inverse(): ");
  // printIsometryMatrix(Xj, "Xj: ");
  // printIsometryMatrix(Z, "Z: ");
  // printIsometryMatrix(Z.inverse(), "Z.inverse(): ");

  Isometry3D E = Z*Xi.inverse()*Xj; 
  printIsometryMatrix(E, "E: ");
  
  // check de/dti
  double delta = 1;//1e-9; 
  double scale = 1./(2*delta); 
  Vector6d xi = g2o::internal::toVectorMQT(Xi); 
  Vector6d xi_p = Vector6d::Zero(); xi_p(0,0) = delta; 
  Vector6d xi_n = Vector6d::Zero(); xi_n(0,0) = -delta; 
  Isometry3D Xi_p = g2o::internal::fromVectorMQT(xi_p); 
  
  printIsometryMatrix(Xi, "Xi: ");
  Xi_p = Xi * Xi_p; 
  printIsometryMatrix(Xi_p, "Xi_p:");

  Isometry3D Xi_n = g2o::internal::fromVectorMQT(xi_n); 
  Xi_n = Xi * Xi_n;
  Vector6d e_p = computeError(Z, Xi_p, Xj); 
  Vector6d e_n = computeError(Z, Xi_n, Xj); 
  Matrix3d numeric_J = Matrix3d::Zero(); 
  cout<<"test_g2o_prior3d_jacobian.cpp: numeric jacobian: "<<endl;
  for(int i=0; i<3; i++)
  {
    numeric_J(0, i) = (e_p(i,0) - e_n(i,0))*scale; 
    cout<<numeric_J(0, i)<<" "; 
  }
  cout<<endl;
  cout<<"test_g2o_prior3d_jacobian.cpp: Z col 0: "<<endl;
  Matrix3d R = g2o::internal::extractRotation(Z); 
  for(int i=0; i<3; i++)
  {
    cout<<R(i, 0)<<" ";
  }
  cout<<endl;

  // check only e03 
  double e03_p = computeE03(Z, Xi_p, Xj); 
  double e03_n = computeE03(Z, Xi_n, Xj); 
  double J00 = (e03_p - e03_n)*scale ;

  printf("e03_p = %.20f, e03_n = %0.20f \n", e03_p, e03_n);
  cout<<"J00 = "<<J00<<endl;
}

Vector6d computeError(Isometry3D& Z, Isometry3D& Xi, Isometry3D& Xj)
{
  Isometry3D delta = Z * Xi.inverse() * Xj; 
  Vector6d error = g2o::internal::toVectorMQT(delta); 
  return error;
}

void test3()
{
  Isometry3D Z = randomIsometry3d(); 
  Isometry3D Xi = randomIsometry3d(); 
  Isometry3D Xj = randomIsometry3d(); 
  
  Isometry3D iXi = Xi.inverse(); 
  // Matrix<double, 3, 1> i1 = iXi.block<3,1>(0,0); 
  // Matrix<double, 3, 1> ti = Xi.block<3,1>(3, 0);

  Matrix3d Rz = g2o::internal::extractRotation(Z); 
  Matrix3d RXi = g2o::internal::extractRotation(Xi); 
  Matrix3d RXj = g2o::internal::extractRotation(Xj); 
  Matrix3d RiXi = g2o::internal::extractRotation(iXi); 
  Vector3d tz = Z.translation(); 
  Vector3d txi = Xi.translation(); 
  Vector3d txj = Xj.translation(); 

  // Vector3d i1 = iXi.block<3,1>(0,0); 
  // Vector3d ti = Xi.block<3,1>(3,0); 
  Vector3d tixi = iXi.translation(); 
  double x1, y1, z1; 
  Vector3d RiXi1 = RXi.block<3,1>(0,0); 
  Vector3d RiXi2 = RXi.block<3,1>(0,1); 
  Vector3d RiXi3 = RXi.block<3,1>(0,2); 

  // printIsometryMatrix(Xi, "Xi: ");
  // cout<<"RiXi1: "<<RiXi1<<endl<<"RiXi2: "<<RiXi2<<endl<<"RiXi3: "<<RiXi3<<endl;
  
  x1 = -RiXi1.dot(txi.transpose());
  y1 = -RiXi2.dot(txi.transpose());
  z1 = -RiXi3.dot(txi.transpose()); 

  Vector3d tiXiXj; 
  tiXiXj(0) = x1 + RiXi1.dot(txj.transpose());
  tiXiXj(1) = y1 + RiXi2.dot(txj.transpose()); 
  tiXiXj(2) = z1 + RiXi3.dot(txj.transpose());
  Isometry3D iXiXj = iXi * Xj; 

  // printIsometryMatrix(iXiXj, "iXiXj: "); 
  // cout<<"check itxi: "<<x1<<" "<<y1<<" "<<z1<<endl;
  // cout<<"check tiXiXj: "<<tiXiXj<<endl;

  Vector3d et; 
  Vector3d Rz1 = Rz.block<1,3>(0,0).transpose();
  Vector3d Rz2 = Rz.block<1,3>(1,0).transpose(); 
  Vector3d Rz3 = Rz.block<1,3>(2,0).transpose(); 
  et(0) = Rz1.dot(tiXiXj.transpose()) + tz(0); 
  et(1) = Rz2.dot(tiXiXj.transpose()) + tz(1); 
  et(2) = Rz3.dot(tiXiXj.transpose()) + tz(2); 
  Isometry3D E = Z * iXiXj; 

  printIsometryMatrix(E, "E: ");
  cout<<"check et: "<<et<<endl;

  printIsometryMatrix(Z, "Z: ");
  double dtdx = -Rz1.dot(RiXi1.transpose());
  cout<<"check dtdx: "<<dtdx<<endl;
}

void test4()
{
  cout<<"test_g2o_prior3d_jacobian.cpp in test4()"<<endl;
  // SparseOptimizer* po1 = createOptimizeClass(); 
  SparseOptimizer* po1 = new SparseOptimizer;
  VertexSE3* v1 = new VertexSE3;
  v1->setId(0); 
  po1->addVertex(v1);
  EdgeSE3PriorZH * e = new EdgeSE3PriorZH;
  // EdgeSE3Prior * e = new EdgeSE3Prior; 
  // e->setVertex(0, v1); 
  e->setInformation(Eigen::Matrix<double, 6, 6, Eigen::ColMajor>::Identity()); 
  e->setMeasurement(Isometry3D::Identity());
  e->vertices()[0] = v1; 

  ParameterSE3Offset* odomOffset=new ParameterSE3Offset();
  odomOffset->setId(0);
  po1->addParameter(odomOffset);

  cout<<"test_g2o_prior3d_jacobian.cpp: edge has parameters: "<<e->numParameters()<<endl;
  e->setParameterId(0,0);
  // e->resolveCacheAndParameters();
  cout<<"test_g2o_prior3d_jacobian.cpp: edge has parameters: "<<e->numParameters()<<endl;
  // e->resizeParameters(0); 
  // cout<<"test_g2o_prior3d_jacobian.cpp: edge has parameters: "<<e->numParameters()<<endl;

  po1->addEdge(e);

  cout<<"test_g2o_prior3d_jacobian.cpp: succeed to addEdge!"<<endl;
  return ;
}

void test1()
{
  VertexSE3* v1 = new VertexSE3;
  VertexSE3* v2 = new VertexSE3; 
  v1->setId(0); 
  v2->setId(1); 
  // EdgeSE3Prior* e = new EdgeSE3Prior; 
  EdgeSE3PriorZH * e = new EdgeSE3PriorZH;
  // EdgeSE3 * e = new EdgeSE3; 
  e->setVertex(0, v1); 
  // e->setVertex(1, v2);
  e->setInformation(Eigen::Matrix<double, 6, 6, Eigen::ColMajor>::Identity()); 

  JacobianWorkspace jacobianWorkspace; 
  JacobianWorkspace numericJacobianWorkspace; 
  
  numericJacobianWorkspace.updateSize(e); 
  if(numericJacobianWorkspace.allocate())
  {
    cout<<"test_g2o_prior3d_jacobian.cpp: succeed to allocate space !"<<endl;
  }
  
  jacobianWorkspace.updateSize(e); 
  jacobianWorkspace.allocate();

  for(int k=0; k<1; ++ k)
  {
    v1->setEstimate(randomIsometry3d()); 
    e->setMeasurement(randomIsometry3d()); 
    if(e->resolveCacheAndParameters())
    {
      cout<<"test_g2o_prior3d_jacobian.cpp: succeed to resolveCacheAndParameters()"<<endl;
    }else
    {
      cout<<"test_g2o_prior3d_jacobian.cpp: failed to resolveCacheAndParameters()"<<endl;
    }
    // calling the analytic jacobian 
    e->BaseUnaryEdge<6, Isometry3D, VertexSE3>::linearizeOplus(jacobianWorkspace); 
    // e->BaseBinaryEdge<6, Isometry3D, VertexSE3, VertexSE3>::linearizeOplus(numericJacobianWorkspace);
    e->analyticJacobian();
    // jacobianWorkspace = numericJacobianWorkspace; 
    double *a = jacobianWorkspace.workspaceForVertex(0); 
    printSquareMatrix(a, 6, "test_g2o_prior3d_jacobian.cpp: jacobianWorkspace: ");

    // calling the numeric jacobian 
    e->BaseUnaryEdge<6, Isometry3D, VertexSE3>::linearizeOplus(numericJacobianWorkspace);
    e->numericJacobian();
    // e->linearizeOplus();
    // e->BaseBinaryEdge<6, Isometry3d, VertexSE3, VertexSE3>::linearizeOplus();
    // compare the two jacobians 
    const double allowedDifference = 1e-6; 
    double *n = numericJacobianWorkspace.workspaceForVertex(0); 
    // double *a = jacobianWorkspace.workspaceForVertex(0); 
    
    // cout<<"test_g2o_prior3d_jacobian.cpp: numericJacobianWorkspace: "<<endl<< numericJacobianWorkspace<<endl;
    printSquareMatrix(n, 6, "test_g2o_prior3d_jacobian.cpp: numericJacobianWorkspace: ");
    // cout<<"test_g2o_prior3d_jacobian.cpp: jacobianWorkspace: "<<endl<< jacobianWorkspace<<endl;
    // printSquareMatrix(a, 6, "test_g2o_prior3d_jacobian.cpp: jacobianWorkspace: ");

    cout<<"test_g2o_prior3d_jacobian.cpp: before compare!"<<endl;

    for(int i = 0; i<6; i++)
    {
      for(int j = 0; j<6; j++)
      {
        double d = fabs(n[i*6+j] - a[i*6+j]);
        if(d > allowedDifference)
        {
          cout<<"* ";
        }else
        {
          cout<<"0 ";
        }
      }
      cout<<endl;
    }
  }
}

void printIsometryMatrix(Isometry3D m, const char * prefix)
{
  cout<<prefix<<endl; 
  Matrix3d R = g2o::internal::extractRotation(m); 
  Isometry3D::TranslationPart t = m.translation();
  for(int i = 0; i<3; i++)
  {
    for(int j = 0; j < 3; j++)
    {
      cout<<R(i,j)<<" ";
    }
    cout<<t(i)<<endl;
  }
}

void printSquareMatrix(double * m, int l, const char * prefix)
{
  cout<<prefix<<endl;
    for(int i = 0; i<l; i++)
    {
      for(int j = 0; j<l; j++)
      {
          cout<<m[i*l+j]<<" ";
      }
      cout<<endl;
    }

}
