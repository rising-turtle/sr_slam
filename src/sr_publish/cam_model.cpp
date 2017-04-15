/*
 * David Z, Apr 21, 2015
 *
 * camera model, to access the parameters of a camera model, 
 * and, provide functions to convert (X,Y,Z) to (u,v,1)*lambda, 
 * convert (u,v,Z) to (X,Y,Z) with the parameters of the model
 *
 * */

#include "cam_model.h"

#ifndef SQ(x)
#define SQ(x) ((x)*(x))
#endif

void CamModel::distortCorrection(double x_d, double y_d, double& x_u, double &y_u)
{
    if(k1 == 0) // no distortion correction needed 
    {
      x_u = x_d; 
      y_u = y_d;
    }else if(k2 == 0)// only k1 is given 
    {
      // TODO :  not  quite understand what's the magic 
      double rd2 = SQ(x_d) + SQ(y_d); 
      double r_distoration = 1 + k1*rd2; 
      double ru2 = rd2 / r_distoration; 
      r_distoration = 1 + k1*ru2; 
      x_u = x_d / r_distoration; 
      y_u = y_d / r_distoration; 
      
    }else
    {
        // using iterative fixed_point root finding algorithm 
        x_u = x_d; 
        y_u = y_d; 

        double rd2, rd4, rd6, k_radial, dx, dy; 
        for(int i=0; i<20; i++)
        {
          rd2 = SQ(x_u) + SQ(y_u); 
          rd4 = SQ(rd2); 
          rd6 = rd4 * rd2; 
          k_radial = 1 + k1 * rd2 + k2 * rd4 + k3 * rd6; 
          dx = 2 * p1* x_u * y_u + p2 * (rd2 + 2 * SQ(x_u)); 
          dy = p1 * (rd2 + 2 * SQ(y_u)) + 2 * p2 * x_u * y_u; 
          x_u = (x_d - dx)/k_radial; 
          y_u = (y_d - dy)/k_radial; 
        }
    }
    return ;
} 

void CamModel::convertUVZ2XYZ(float u, float v, double z, double &ox, double& oy, double &oz)
{
  // just assume x' = x'', without considering the distortion effect 
  double fx_inv = 1./fx; 
  double fy_inv = 1./fy;
  // double x = (u - cx)*fx_inv;
  // double y = (v - cy)*fy_inv; 
  double x_d = (u-cx)*fx_inv;  // distorted point on the normalized image plane 
  double y_d = (v-cy)*fy_inv;   
  double x_u, y_u;  // undistorted point on the normalized image plane 
  
  distortCorrection(x_d, y_d, x_u, y_u);

  ox = x_u*z ;
  oy = y_u*z ; 
  oz = z; 
}

