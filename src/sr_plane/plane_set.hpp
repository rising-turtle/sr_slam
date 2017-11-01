#include <pcl/filters/extract_indices.h>

template<typename PointT>
int CPlaneSet::extractPlanes(boost::shared_ptr<pcl::PointCloud<PointT> >& in, double dis_threshold)
{
  // first clear planes 
  clearPlanes(); 

  int ret = 0;
  int M = in->points.size(); 
  int i_thresh_number = M * d_percent_threshold_; 
  
  if(i_thresh_number > d_threhold_num_) 
    i_thresh_number = d_threhold_num_; 

  boost::shared_ptr<pcl::PointCloud<PointT> > in_copy = in->makeShared();

  do
  {
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    CPlane * p = new CPlane(); 
    p->computePCL(in_copy, inliers, dis_threshold); 
    if(inliers->indices.size() < i_thresh_number)
    {
      // cout <<"inliers has size() = "<<inliers->indices.size()<<" i_thresh_number = "<<i_thresh_number<<endl; 
      // cout <<"M = "<<M<<" d_percent_threshold_ = "<<d_percent_threshold_<<endl;
      break;  // number of the points must large than i_thresh_number 
    }
    
    int cur_number = in_copy->points.size();
    int inlier_number  = inliers->indices.size();
    p->computeCenter(in_copy, inliers); 
    // p->inliers_->indices = inliers->indices;
    v_plane_set_.push_back(p); 
    ++ret; 
    if(cur_number - inlier_number < i_thresh_number)
    {
      break;  // number of the rest points is less than i_thresh_number
    }
    
    // use the rest points 
    boost::shared_ptr<pcl::PointCloud<PointT> > tmpPC(new pcl::PointCloud<PointT>);
    extractOutlier(in_copy, tmpPC, inliers);
    
    in_copy.swap(tmpPC);

  }while(1);

  return ret;
}

template<typename PointT>
int CPlaneSet::extractPlanes(boost::shared_ptr<pcl::PointCloud<PointT> >& in, boost::shared_ptr<pcl::PointCloud<PointT> >& out,
    std::vector<int>& plane_num, double dis_threshold)
{
  // first clear planes 
  clearPlanes(); 

  int ret = 0;
  int M = in->points.size(); 
  int i_thresh_number = M * d_percent_threshold_; 
  
  if(i_thresh_number > d_threhold_num_) 
    i_thresh_number = d_threhold_num_; 

  // clear history 
  boost::shared_ptr<pcl::PointCloud<PointT> > in_copy = in->makeShared();
  out.reset(new pcl::PointCloud<PointT>);
  plane_num.clear(); 

  do
  {
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    CPlane * p = new CPlane(); 
    p->computePCL(in_copy, inliers, dis_threshold); 
    if(inliers->indices.size() < i_thresh_number)
    {
      // cout <<"inliers has size() = "<<inliers->indices.size()<<" i_thresh_number = "<<i_thresh_number<<endl; 
      // cout <<"M = "<<M<<" d_percent_threshold_ = "<<d_percent_threshold_<<endl;
      break;  // number of the points must large than i_thresh_number 
    }
    
    int cur_number = in_copy->points.size();
    int inlier_number  = inliers->indices.size();
    p->computeCenter(in_copy, inliers); 
    // p->inliers_->indices = inliers->indices;
    v_plane_set_.push_back(p); 

    // save the points of this plane 
    boost::shared_ptr<pcl::PointCloud<PointT> > plane_PC(new pcl::PointCloud<PointT>);
    extractInlier(in_copy, plane_PC, inliers); 
    (*out) = (*out) + (*plane_PC); 
    plane_num.push_back(inlier_number); 
    
    ++ret; 
    if(cur_number - inlier_number < i_thresh_number)
    {
      break;  // number of the rest points is less than i_thresh_number
    }
    
    // use the rest points 
    boost::shared_ptr<pcl::PointCloud<PointT> > tmpPC(new pcl::PointCloud<PointT>);
    extractOutlier(in_copy, tmpPC, inliers);
    
    in_copy.swap(tmpPC);

  }while(1);

  return ret;
}




template<typename PointT>
void CPlaneSet::extractOutlier(boost::shared_ptr<pcl::PointCloud<PointT> >& in, 
    boost::shared_ptr<pcl::PointCloud<PointT> >& out, pcl::PointIndices::Ptr& inliers)
{
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud (in);
  extract.setIndices (inliers);
  extract.setNegative (true);
  extract.filter (*out);
  return ;
}


template<typename PointT>
void CPlaneSet::extractInlier(boost::shared_ptr<pcl::PointCloud<PointT> >& in, 
    boost::shared_ptr<pcl::PointCloud<PointT> >& out, pcl::PointIndices::Ptr& inliers)
{
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud (in);
  extract.setIndices (inliers);
  extract.setNegative (false);
  extract.filter (*out);
  return ;
}

