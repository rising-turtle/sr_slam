/*
 *  Sep. 22, 2015, David Z
 *
 *  Camera observation for a plane given the plane parameters 
 *
 * */


#ifndef CAM_OBSERVATION_H
#define CAM_OBSERVATION_H

#include "plane_set.h"
#include "plane.h"
#include <map>

typedef pair<int, int> Pair; 
typedef vector<Pair>   VPair;

typedef pair<float, float> FPair; 
typedef vector<FPair> VFPair;

extern VectorPF3 genSR4k(CPlane* p);   // generate point cloud for SR4K towards a plane 
extern VectorPF3 genSR4k_Noise(CPlane *p, double std = 0.01);  // add gaussian noise N(0, sigma)  

extern VPair findMatchSR4k(VectorPF3& src, tf::Transform tf);   // find matched pairs of points after transformation
extern VectorPF3 genSR4k_set(CPlane*p, VFPair);   // generate point cloud given point set 
extern VectorPF3 genSR4k_set(CPlane*p, VectorPF3&);   // generate point cloud given point set 
extern CPointF3 genSR4k_data(CPlane*p, CPointF3&);
extern VectorPF3 genSR4k_set(CPlaneSet* p, VectorPF3&, VPair& , vector<int>&, vector<bool>& belong_to_plane);
extern VectorPF3 genSR4k_set(CPlane* p, VectorPF3&, VPair& , vector<int>&, vector<bool>& belong_to_plane);

#endif
