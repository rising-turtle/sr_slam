/*
 * David Z, Apr 21, 2015
 *
 * camera model, to access the parameters of a camera model, 
 * and, provide functions to convert (X,Y,Z) to (u,v,1)*lambda, 
 * convert (u,v,Z) to (X,Y,Z) with the parameters of the model
 *
 * */

#include "cam_model.h"

void CamModel::convertUVZ2XYZ(float u, float v, double z, double &ox, double& oy, double &oz)
{
  // just assume x' = x'', without considering the distortion effect 
  double fx_inv = 1./fx; 
  double fy_inv = 1./fy;
  double x = (u - cx)*fx_inv;
  double y = (v - cy)*fy_inv; 
  ox = x*z ;
  oy = y*z ; 
  oz = z; 
}

