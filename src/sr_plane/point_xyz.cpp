#include "point_xyz.h"
#include <algorithm>
#include <fstream>

void dumpPC2File(VectorPF3& ret, string fname)
{
  ofstream ouf(fname.c_str());
  for(int i=0; i<ret.size(); i++)
    ouf<<ret[i]<<" "<<ret[i].dis_()<<endl;
}

VectorPF3 copyPercent(VectorPF3& in, float percent)  // maintain percent portion of the original points according to distance
{
  VectorPF3 ret = in; 
  std::sort(ret.begin(), ret.end()); 

  int location = ret.size()*percent; 
  if(location >= 0 && location < ret.size())
    // ret.erase(ret.begin() + location, ret.end());
    {
      ret.erase(ret.end() - location/2, ret.end());
      ret.erase(ret.begin(), ret.begin() + location/2);
    }
  return ret;
}

ostream& operator<<(ostream& out, CPointF3& p)
{
  out<<p[0]<<" "<<p[1]<<" "<<p[2];
  return out;
}


