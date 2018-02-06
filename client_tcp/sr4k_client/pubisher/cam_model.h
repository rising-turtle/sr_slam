/*
 * David Z, Apr 21, 2015
 *
 * camera model, to access the parameters of a camera model, 
 * and, provide functions to convert (X,Y,Z) to (u,v,1)*lambda, 
 * convert (u,v,Z) to (X,Y,Z) with the parameters of the model
 *
 * */
#ifndef CAM_MODEL_H
#define CAM_MODEL_H

#include "opencv2/opencv.hpp"

#define SQ(x) ((x)*(x))

class CamModel
{
  public:
    CamModel(cv::Mat& mat)
    {
      _toCam(mat);
    }
    CamModel(){}
    CamModel(double fx1, double fy1, double cx1, double cy1, double k11=0, double k22=0, double p11=0, double p22=0, double k33=0, double k44=0, double k55=0, double k66=0): 
    fx(fx1), fy(fy1), cx(cx1), cy(cy1), k1(k11), k2(k22), p1(p11), p2(p22), k3(k33), k4(k44), k5(k55), k6(k66)
  {}
    virtual ~CamModel(){}
  public:
    // convert (u, v, Z) to (X, Y, Z) with the current parameters 
    void convertUVZ2XYZ(float u, float v, double z, double &ox, double &oy, double& oz); 
    
    // TODO: convert (X,Y,Z) to (u,v,1)
    // convertXYZ2UV()

    void _toMat(cv::Mat& mat)
    {
       double * _cm = (double*)mat.ptr(); 
      // camera matrix, fx, cx, fy, cy, 
      _cm[0] = fx; _cm[2] = cx; 
      _cm[4] = fy; _cm[5] = cy; 
      _cm[8] = 1.0; 
    }
    void _toCam(cv::Mat& mat)
    {
      double * _cm = (double*)mat.ptr(); 
      fx = _cm[0]; cx = _cm[2] ; 
      fy = _cm[4]; cy = _cm[5] ; 
    }

    // intrinsic matrix 
    double fx; 
    double fy; 
    double cx; 
    double cy;

    // distortion parameters
    double k1; 
    double k2; 
    double k3; 
    double k4;
    double k5; 
    double k6; 
    double p1; 
    double p2;
};



#endif
