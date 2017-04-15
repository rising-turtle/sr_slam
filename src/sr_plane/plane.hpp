
template<typename PointT>
void toPC(vector<CPointF3>& pts, boost::shared_ptr<pcl::PointCloud<PointT> >& pc)
{
  int N = pts.size();
  pc->points.resize(N); 
  pc->width = N;
  pc->height = 1;
  for(int i=0; i<N; i++)
  {
    PointT& pt = pc->points[i]; 
    CPointF3& p = pts[i]; 
    pt.x = p[0]; pt.y = p[1]; pt.z = p[2]; 
  }
}
template<typename PointT>
void toPC(vector<CPointF3>& pts, boost::shared_ptr<pcl::PointCloud<PointT> >& pc, pcl::PointIndices::Ptr& inliers)
{
  int N = inliers->indices.size();
  pc->points.resize(N); 
  pc->width = N;
  pc->height = 1;
  for(int i=0; i<N; i++)
  {
    PointT& pt = pc->points[i]; 
    CPointF3& p = pts[inliers->indices[i]]; 
    pt.x = p[0]; pt.y = p[1]; pt.z = p[2]; 
  }
}

template<typename PointT>
void fromPC(vector<CPointF3>& pts, boost::shared_ptr<pcl::PointCloud<PointT> >& pc)
{
  int N = pc->points.size(); 
  pts.resize(N); 
  for(int i=0; i<N; i++)
  {
    CPointF3& p = pts[i]; 
    PointT& pt = pc->points[i]; 
    p[0] = pt.x; p[1] = pt.y; p[2] = pt.z;
  } 
}

template<typename PointT>
void fromPC(vector<CPointF3>& pts, boost::shared_ptr<pcl::PointCloud<PointT> >& pc, pcl::PointIndices::Ptr& inliers)
{
  int N = inliers->indices.size(); // pc->points.size(); 
  pts.resize(N); 
  for(int i=0; i<N; i++)
  {
    CPointF3& p = pts[i]; 
    PointT& pt = pc->points[inliers->indices[i]]; 
    p[0] = pt.x; p[1] = pt.y; p[2] = pt.z;
  }  
}

template<typename PointT>
bool calPCCorvariance(boost::shared_ptr<pcl::PointCloud<PointT> >& pc, Eigen::Matrix3f& cov)
{
  double X = 0, Y = 0, Z= 0, XY =0, XZ=0, YZ=0;
  double X2 = 0, Y2 =0, Z2 =0;

  double x,y,z;
  for(int i=0; i<pc->size(); i++)
  {
    PointT& pt = pc->points[i];
    x = pt.x; 
    y = pt.y;
    z = pt.z;
    X += x; 
    Y += y; 
    Z += z; 
    XY += x*y;
    XZ += x*z; 
    YZ += y*z;
    X2 += SQ(x);
    Y2 += SQ(y);
    Z2 += SQ(z);
  }
  double N = pc->size(); 
  N = 1./N;
  X *= N; // /= N;
  Y *= N; // /= N;
  Z *= N; // /= N;
  XY *=N; // /= N; 
  XZ *=N; // /= N;
  YZ *=N; // /= N;
  X2 *=N; // /= N;
  Y2 *=N; // /= N;
  Z2 *=N; // /= N; 

  // 
  float s = 1; 
  cov(0,0) = X2 - SQ(X); 
  cov(1,0) = cov(0,1) = s*(XY - X*Y); 
  cov(2,0) = cov(0,2) = s*(XZ - X*Z);
  cov(1,1) = Y2 - SQ(Y);
  cov(2,1) = cov(1,2) = s*(YZ - Y*Z);
  cov(2,2) = Z2 - SQ(Z);
  return true;
}

template<typename PointT>
void CPlane::computeParameters(boost::shared_ptr<pcl::PointCloud<PointT> >& pc)
{
  VectorPF3 pts; 
  fromPC<PointT>(pts, pc); 
  
  computeParameters(pts);
  return ;
}


template<typename PointT>
void CPlane::computeCenter(boost::shared_ptr<pcl::PointCloud<PointT> >& in, pcl::PointIndices::Ptr& inliers)
{
  double tx =0; double ty = 0; double tz = 0; 
  int N = inliers->indices.size(); 
  int cnt = 0;
  for(int i=0; i<inliers->indices.size(); i++)
  {
    PointT& pt = in->points[inliers->indices[i]]; 
    if(pt.z > 1.5) continue;
    tx += pt.x; ty += pt.y; tz += pt.z; 
    ++cnt;
  }
  if(cnt>0) 
  {
    double s = 1./(double)(cnt); 
    cx_ = tx * s; 
    cy_ = ty * s; 
    cz_ = tz * s;
  }
}


template<typename PointT>
void CPlane::computePCL(boost::shared_ptr<pcl::PointCloud<PointT> >& in, pcl::PointIndices::Ptr& inliers, double dis_threshold )
{
  pcl::SACSegmentation<PointT> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);    
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(dis_threshold); // 0.01 0.0025
  
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

  seg.setInputCloud(in);
  seg.segment(*inliers, *coefficients);
  
  nx_ = coefficients->values[0];
  ny_ = coefficients->values[1];
  nz_ = coefficients->values[2];
  d1_ = coefficients->values[3];

  return ;
}

