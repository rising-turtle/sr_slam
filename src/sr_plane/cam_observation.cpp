#include "cam_observation.h"
#include <utility>
#include <cmath>
#include <cfloat>

VectorPF3 genSR4k(CPlane* p)
{
    VectorPF3 ret = genSR4k_Noise(p, 0);
    return ret;
}

inline int uv2index(int u, int v)
{
  return (u + v*176);
}

int voteByNeighbor(int u, int v, int M, vector<int>& p_index)
{
  static int N_neighbors = 8; //20; //8; 
  static vector<int> neighbors(N_neighbors); 
  
  int boarder = 5;
  if(u <= boarder || u >= 176 - boarder || v <= boarder || v >= 144 - boarder)
  {
    return -1; 
  }

  // inner circle neighbors 
  neighbors[0] = uv2index(u-1,v-1); 
  neighbors[1] = uv2index(u-1,v); 
  neighbors[2] = uv2index(u-1,v+1); 
  neighbors[3] = uv2index(u,v-1); 
  neighbors[4] = uv2index(u,v+1); 
  neighbors[5] = uv2index(u+1,v-1); 
  neighbors[6] = uv2index(u+1,v); 
  neighbors[7] = uv2index(u+1,v+1); 
  
  // outer circle neighbors 
 /* neighbors[8] = uv2index(u-2,v-1); 
  neighbors[9] = uv2index(u-2,v);
  neighbors[10] = uv2index(u-2,v+1);
  neighbors[11] = uv2index(u-1,v+2);
  neighbors[12] = uv2index(u, v+2); 
  neighbors[13] = uv2index(u+1, v+2);
  neighbors[14] = uv2index(u+2, v+1); 
  neighbors[15] = uv2index(u+2, v);
  neighbors[16] = uv2index(u+2, v-1);
  neighbors[17] = uv2index(u+1, v-2);
  neighbors[18] = uv2index(u, v-2);
  neighbors[19] = uv2index(u-1, v-2);
*/
  // start to vote
  vector<int> vote(M, 0); 
  int vote_id;
  int vote_n = N_neighbors/2;
  for(int k=0; k<N_neighbors; k++)
  {
    vote_id = p_index[neighbors[k]];
    if(vote_id == -1) continue; 
    if(++vote[vote_id] >= vote_n) return vote_id;
  }
  return -1;
}

VectorPF3 genSR4k_set(CPlane* p, VectorPF3& pf, VPair& pf2d, vector<int>& p_index, vector<bool>& belong_to_plane)
{
  VectorPF3 ret;
  int M = pf.size(); 
  ret.reserve(M);
  belong_to_plane.resize(M);

  int u, v, index; 
  const static int NUM = 144*176;

  for(int i=0; i<M; i++)
  {
    u = pf2d[i].first; v = pf2d[i].second; 
    if(v == 144) v = 143; 
    if(u == 176) u = 175;
    index = uv2index(u, v); 
    // cout<<"cam_observation.cpp: u: "<<u<<" v: "<<v<<" index: "<<index<<endl;
    // this point already in the plane, do not disturbe it 
    if(p_index[index] != -1)     
    {
      // cout<<"cam_observation.cpp: p_index[index] "<<p_index[index]<<endl;
      // CPointF3 rpf = genSR4k_data(pset->v_plane_set_[p_index[index]], pf[i]); 
      CPointF3 rpf = genSR4k_data(p, pf[i]);
      ret.push_back(rpf);
      belong_to_plane[i] = true;
    }else
    {
      // int vote_by_neighbor = voteByNeighbor(u,v, pset->v_plane_set_.size(), p_index);
      int vote_by_neighbor = voteByNeighbor(u,v, 1, p_index);
      // cout<<"cam_observation.cpp: vote_by_neighbor "<<vote_by_neighbor<<endl;
      if(vote_by_neighbor != -1)
      {
        // CPointF3 rpf = genSR4k_data(pset->v_plane_set_[vote_by_neighbor], pf[i]);
        CPointF3 rpf = genSR4k_data(p, pf[i]);
        ret.push_back(rpf);
        belong_to_plane[i] = true;
      }else
      {
        ret.push_back(CPointF3(NAN));
        belong_to_plane[i] = false;
        // cout<<"cam_observation.cpp: i = "<<i<<" not belong to plane!"<<endl;
      }
    }
  }
  return ret;
}

VectorPF3 genSR4k_set(CPlaneSet* pset, VectorPF3& pf, VPair& pf2d, vector<int>& p_index, vector<bool>& belong_to_plane)
{
  VectorPF3 ret;
  int M = pf.size(); 
  ret.reserve(M);
  belong_to_plane.resize(M);

  int u, v, index; 
  const static int NUM = 144*176;

  for(int i=0; i<M; i++)
  {
    u = pf2d[i].first; v = pf2d[i].second; 
    if(v == 144) v = 143; 
    if(u == 176) u = 175;
    index = uv2index(u, v); 
    // cout<<"cam_observation.cpp: u: "<<u<<" v: "<<v<<" index: "<<index<<endl;
    // this point already in a plane, do not disturbe it 
    if(p_index[index] != -1)
    {
      // cout<<"cam_observation.cpp: p_index[index] "<<p_index[index]<<endl;
      CPointF3 rpf = genSR4k_data(pset->v_plane_set_[p_index[index]], pf[i]); 
      ret.push_back(rpf);
      belong_to_plane[i] = true;
      // ret.push_back(CPointF3(NAN));
      // belong_to_plane[i] = false;
    }else
    {

      ret.push_back(CPointF3(NAN));
      belong_to_plane[i] = false;
      continue; 

      int vote_by_neighbor = voteByNeighbor(u,v, pset->v_plane_set_.size(), p_index);
      // cout<<"cam_observation.cpp: vote_by_neighbor "<<vote_by_neighbor<<endl;
      if(vote_by_neighbor != -1)
      {
        CPointF3 rpf = genSR4k_data(pset->v_plane_set_[vote_by_neighbor], pf[i]);
        ret.push_back(rpf);
        belong_to_plane[i] = true;
        // ret.push_back(CPointF3(NAN));
        // belong_to_plane[i] = false;
      }else
      {
        ret.push_back(CPointF3(NAN));
        belong_to_plane[i] = false;
        // cout<<"cam_observation.cpp: i = "<<i<<" not belong to plane!"<<endl;
      }
    }
  }
  return ret;
}

CPointF3 genSR4k_data(CPlane* p, CPointF3& pf)
{
  double nx, ny, nz, d; 
   nx = p->nx_; ny = p->ny_; nz = p->nz_; d = p->d1_;
   
   // invalid types
   int invalid_t = 0;
   double x, y, z; 
   double t, scale;
   CPointF3 rpf;
  // for(int i=0; i<pf.size(); i++)
   {
      x = pf[0];  y = pf[1]; z = pf[2]; 
      t = nx * x + ny * y + nz * z; 
      if(fabs(t) < 1e-5)
      {
        cerr<<"cam_observation.cpp: something error here, t: "<<t<<endl;
        rpf = CPointF3(NAN);
      }else
      {
        scale = d/t;
        x = x*scale; y = y*scale; z = z*scale; 
        rpf = CPointF3(x, y, z);
      }
      // ret.push_back(rpf);
   }
   return rpf;
}

VectorPF3 genSR4k_set(CPlane* p, VectorPF3& pf)
{
   VectorPF3 ret;
   ret.reserve(pf.size());
   double nx, ny, nz, d; 
   nx = p->nx_; ny = p->ny_; nz = p->nz_; d = p->d1_;
   
   // invalid types
   int invalid_t = 0;
   double x, y, z; 
   double t, scale;
   CPointF3 rpf;
   for(int i=0; i<pf.size(); i++)
   {
      x = pf[i][0];  y = pf[i][1]; z = pf[i][2]; 
      t = nx * x + ny * y + nz * z; 
      if(fabs(t) < 1e-5)
      {
        cerr<<"cam_observation.cpp: something error here, t: "<<t<<endl;
        rpf = CPointF3(NAN);
      }else
      {
        scale = d/t;
        x = x*scale; y = y*scale; z = z*scale; 
        rpf = CPointF3(x, y, z);
      }
      ret.push_back(rpf);
   }
   return ret;
}

VectorPF3 genSR4k_set(CPlane* p, VFPair index)
{
  VectorPF3 ret;
  // for SR 4k 
  int w = 176; 
  int h = 144;
  int cx = w/2;  // 88
  int cy = h/2;  // 72
  
  double f = 225; 
  double inv_f = 1./f;

  float ui, vi, u, v;
  double t;
  double nx, ny, nz, d; 
  nx = p->nx_; ny = p->ny_; nz = p->nz_; d = p->d1_;
  
  // invalid types
  int invalid_t = 0;
  int invalid_z = 0;
  int valid_p = 0;

  // cout<<"plane: nx: "<<nx<<" ny: "<<ny<<" nz: "<<nz<<" d: "<<d<<endl;
  double xi, yi, zi;
  // for(int u = 0; u<w; u++)
  //   for(int v = 0; v<h; v++)
  VFPair::iterator it = index.begin(); 
  CPointF3 pf;
  while(it != index.end())
  {
    u = it->first; 
    v = it->second;
    ui = u - cx ; 
    vi = v - cy ;
    t = ( nx*ui*inv_f + ny*vi*inv_f + nz);
    if(t == 0)
    {
      // invalid point 
      // cout<<"cam_observation.cpp: t = 0 at ui "<<ui<<" vi "<<vi<<endl;
      pf = CPointF3(NAN);
      ++invalid_t; 
    }else
    {
      zi = d/t ; //+ p->gaussValue(); 
      if(zi <= 0 || zi > 5) // maximum range here, currently set it as 10
      {
        // invalid point 
        // cout<<"cam_observation.cpp: zi is "<<zi<<" t = "<<t<<endl;
        pf = CPointF3(NAN);
        ++invalid_z;
      }else
      {
        // valid point 
        xi = ui*inv_f *zi; 
        yi = vi*inv_f *zi; 
        // CPointF3 p(xi, yi, zi); 
        pf = CPointF3(xi, yi, zi);
        ++valid_p;
      }
    }
    ret.push_back(pf);
    ++it;
  }
  // cout<<"cam_observation.cpp: SR4k valid: "<<valid_p<<" invalid_t: "<<invalid_t<<" invalid_z: "<<invalid_z<<endl;

  assert(ret.size() == index.size());
  return ret;
}

VectorPF3 genSR4k_Noise(CPlane* p, double std)
{
  // for ret
  VectorPF3 ret;
  p->setGaussSigma(std);

  // for SR 4k 
  int w = 176; 
  int h = 144;
  int cx = w/2;  // 88
  int cy = h/2;  // 72
  
  double f = 225; 
  double inv_f = 1./f;

  int ui, vi;
  double t;
  double nx, ny, nz, d; 
  nx = p->nx_; ny = p->ny_; nz = p->nz_; d = p->d1_;
  
  // invalid types
  int invalid_t = 0;
  int invalid_z = 0;
  int valid_p = 0;

  // cout<<"plane: nx: "<<nx<<" ny: "<<ny<<" nz: "<<nz<<" d: "<<d<<endl;
  double xi, yi, zi;
  for(int u = 0; u<w; u++)
    for(int v = 0; v<h; v++)
    {
       ui = u - cx ; 
       vi = v - cy ;
       t = ( nx*ui*inv_f + ny*vi*inv_f + nz);
       if(t == 0)
       {
          // invalid point 
          // cout<<"cam_observation.cpp: t = 0 at ui "<<ui<<" vi "<<vi<<endl;
          ++invalid_t; 
       }else
       {
         zi = d/t + p->gaussValue(); 
         if(zi <= 0 || zi > 10) // maximum range here, currently set it as 10
         {
          // invalid point 
          // cout<<"cam_observation.cpp: zi is "<<zi<<" t = "<<t<<endl;
          ++invalid_z;
         }else
         {
          // valid point 
          xi = ui*inv_f *zi; 
          yi = vi*inv_f *zi; 
          CPointF3 p(xi, yi, zi); 
          ret.push_back(p);
          ++valid_p;
         }
       }
    }
    // cout<<"cam_observation.cpp: SR4k valid: "<<valid_p<<" invalid_t: "<<invalid_t<<" invalid_z: "<<invalid_z<<endl;

    return ret;
}

bool addPairSR4k(map<int, pair<int, double> >& z_match, int src_id, int tar_id, double z)
{
  bool ret = true;
  map<int, pair<int, double> >::iterator it = z_match.find(tar_id);
  if(it == z_match.end())
  {
    // z_match[tar_id] = make_pair<int, double>(src_id, z); 
    z_match[tar_id] = std::pair<int, double>(src_id, z); 
  }else
  {
      // overlap here
      double cur_z = (*it).second.second; 
      if(cur_z > z) // the new one is smaller, replace it
      {
        // z_match[tar_id] = make_pair<int, double>(src_id, z);
        z_match[tar_id] = std::pair<int, double>(src_id, z);
      }
      ret = false;
  }
  return ret;
}

VPair findMatchSR4k(VectorPF3& src, tf::Transform tr)
{
  VectorPF3 tar; 
  transformPC(tar, src, tr); // transform the srouce point cloud into the target camera frame
  
  map<int, pair<int, double> > z_depth; // for each matched pair, the smaller z is valid

  // detect weather the transformed points are in the field-of-view of the camera in target location 
  int N = tar.size();
  double xi, yi, zi; 

  // for SR 4k 
  int w = 176; 
  int h = 144;
  int cx = w/2;  // 88
  int cy = h/2;  // 72
  
  double f = 225; 
  double inv_f = 1./f;

  int ui, vi;  
  double uf, vf;

  int cnt = 0;

  for(int i=0; i<N; i++)
  {
    CPointF3 p = tar[i];
    zi = p[2]; 
    if(zi <= 0 || zi >= 10) // not valid 
    {
      continue; 
    }else
    {
      // point x, y, z
      xi = p[0]; 
      yi = p[1];
      uf = xi*f/zi;  // pixel at uf 
      vf = yi*f/zi;  // pixel at vf
      
      // uf, vf -> ui, vi; 
      ui = round(uf) + cx ; 
      vi = round(vf) + cy ;
      
      // cout<<"xi: "<<xi<<" yi: "<<yi<<" zi: "<<zi<<" uf: "<<uf<<" vf: "<<vf<<" ui: "<<ui<<" vi: "<<vi<<endl;

      // if it is in the image 
      if( ui >= 0 && ui < w && vi >= 0 && vi < h)
      {
         // get target id 
         int tar_id = ui*h + vi; 
         // cout<<"tar_id: "<<tar_id<<" src_id "<<i<<endl;
         if(addPairSR4k(z_depth, i, tar_id, zi))
           ++cnt;
      }
    }
  }
  
  cout<<"cam_observation.cpp: valid match pair number: "<<cnt<<endl;

  // return result 
  VPair ret; 
  map<int, pair<int,double> >::iterator it = z_depth.begin(); 
  while(it!=z_depth.end())
  {
    // ret.push_back( make_pair<int, int>(it->first, (*it).first)); 
    ret.push_back( std::pair<int, int>(it->second.first, it->first )); 
    ++it;
  }
  return ret;
}
