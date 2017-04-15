#include "pos_msg_pub.h"
#include "std_msgs/Float32MultiArray.h"

C2DPosPub::C2DPosPub()
{
  ros::NodeHandle n; 
  r_pos_pub_ = n.advertise<std_msgs::Float32MultiArray>("/robot_pos", 10);
}
C2DPosPub::~C2DPosPub()
{
  r_pos_pub_.shutdown();
 }

bool C2DPosPub::readRecord()
{
  ros::NodeHandle np("~"); 
  string fname("pos_2D_record.log"); 
  np.param("record_name", fname, fname); 
  ifstream in_msg(fname.c_str());
  if(!in_msg.is_open())
  { 
    ROS_ERROR("pos_msg_pub.cpp: failed to open msg file pos_2D_record.log"); 
    return false; 
  }

  // clear msgs 
  msgs_.clear();

  float x,y,theta; 
  int pose_id, N_PT; 
  // read records 
  while(!in_msg.eof())
  {
    in_msg>>x>>y>>theta>>pose_id>>N_PT;  
    vector<float> msg(N_PT*2+4);
    msg[0] = x; msg[1] = y; msg[2] = theta; msg[3] = pose_id; 
    for(int i=0; i<N_PT; i++)
    {
      in_msg>>msg[4+i*2]>>msg[4+i*2+1]; 
    }
    msgs_.push_back(msg); // read this record
  }
  in_msg.close(); 
  return true;
}

void C2DPosPub::publishPosOneLoop(double s)
{
   if(!readRecord())
   {
     ROS_ERROR("pos_msg_pub.cpp: failed to read record!"); 
     return ;
   } 

   ros::Rate loop_rate(s);  // publish speed 

   for(int i=0; i<msgs_.size(); i++)
   {
     publishPos(i); 
     // sleep time 
     ros::spinOnce();
     loop_rate.sleep();
   }
}

void C2DPosPub::publishPos(int k)
{
  if(k<0 || k>msgs_.size())
  {
    ROS_ERROR("pos_msg_pub.cpp: failed to get msg k = %d", k);
    return; 
  }
  
  vector<float>& msg = msgs_[k]; 

  std_msgs::Float32MultiArray pos_msg; 
  pos_msg.data.resize(msg.size()); 
  for(int i=0; i<msg.size(); i++)
  {
    pos_msg.data[i] =  msg[i]; 
  }

  r_pos_pub_.publish(pos_msg); 
}





