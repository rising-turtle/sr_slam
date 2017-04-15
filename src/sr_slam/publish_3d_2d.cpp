/*
 *  June. 27, 2016 David Z 
 * 
 *  Subscribe 3D pose, and translate it into a 2D pose, drawn in a floor plan, 
 *
 * */

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <fstream>

#define SQ(x) ((x)*(x))

using namespace std;

// 2D publisher 
ros::Publisher talk_2d ; 

// transformation 
tf::Transform T_g2o;
tf::Transform T_g2c;

// mapping rule 
int mapping_rule_select = 4;

// whether to display recevied pose data 
bool b_display_recevied_pose;


void init_transformation();
void poseCb(geometry_msgs::PoseStampedConstPtr msgP3d);
void generate2D();

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "publish_3d_2d"); 
  ros::NodeHandle n; 
  
  // 3D subscriber
  ros::Subscriber listen_3d = n.subscribe("/pos_3d", 7, poseCb);
  talk_2d = n.advertise<std_msgs::Float32MultiArray>("/pos_2d", 7); 
  
  // first transformation 
  init_transformation(); 

  // second mapping rule, from 3D to 2D 
  ros::NodeHandle nh("~");
  // nh.param("2d_mapping_rule", mapping_rule_select, 2);     
  int tmp_rule;
  if(nh.getParam("mapping_rule_2d", tmp_rule))
  {
    mapping_rule_select = tmp_rule;
  }

  nh.param("display_received_pose", b_display_recevied_pose, false); 

  ros::spin();
  ros::shutdown(); 
  printf("publish_3d_2d.cpp: exit!\n");
  return 0; 
}

void display(tf::Transform& tf_p)
{
  float x,y,z,qx,qy,qz,qw;
  x = tf_p.getOrigin()[0];
  y = tf_p.getOrigin()[1];
  z = tf_p.getOrigin()[2];
  qx = tf_p.getRotation().x();
  qy = tf_p.getRotation().y();
  qz = tf_p.getRotation().z(); 
  qw = tf_p.getRotation().getW();

  static int cnt = 0;
  ROS_INFO("publish_3d_2d.cpp: received %d pose: %f %f %f %f %f %f %f", ++cnt, x, y, z, qx, qy, qz, qw);
  static ofstream ouf("./publish_3d_2d.log");
  ouf<<cnt<<" "<<x<<" "<<y<<" "<<z<<" "<<qx<<" "<<qy<<" "<<qz<<" "<<qw<<endl;
}

void poseCb(geometry_msgs::PoseStampedConstPtr msgP3d)
{
  tf::Transform T_o2c;
  tf::poseMsgToTF(msgP3d->pose, T_o2c); 
  if(b_display_recevied_pose)
  {
    display(T_o2c); 
  }
  T_g2c = T_g2o * T_o2c; 
  generate2D(); 
}

void generate2D()
{
  static int cnt = 0;
  std_msgs::Float32MultiArray pos_msg; 
  pos_msg.data.resize(4);     // x, y, theta, node_id there is coordinate translation  
  tf::Vector3 t = T_g2c.getOrigin(); 
  
  float x1, y1, z1; 
  float x2, y2, z2; 
  x1 = t.getX(); y1 = t.getY(); z1 = t.getZ(); 
    
  // quaternion q w,x,y,z -> q0, q1, q2, q3
  tf::Quaternion q = T_g2c.getRotation();    
    
    // http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    // float yaw = asin(2*(q.w()*q.y() - q.x()*q.z())); 
  float yaw = atan2(2*(q.w()*q.z() + q.x()*q.y()), 1-2*(SQ(q.y()) + SQ(q.z())));

    // 2D map reference 
    //
    //    O ------- x2
    //      |
    //      |
    //      |
    //      y2 
    //

    if(mapping_rule_select ==1 )
    {
      //     O ----- y
      //      |
      //      |
      //      x
      x2 = y1; 
      y2 = x1; 
    }else if(mapping_rule_select == 2)
    {
      //        y
      //        |
      //        |
      //        |
      //       O ----- x 
     
      x2 = x1;
      y2 = -y1;
      // ROS_ERROR("graph_wrapper.cpp: in mapping_rule_ 2");
    }else if(mapping_rule_select == 3)
    {
      //        x
      //        |
      //        |
      //        |  
      // y ----- O
      //
      x2 = -y1; 
      y2 = -x1;
    }else if(mapping_rule_select == 4)
    {
      // x ----- O     
      //        |
      //        |
      //        | 
      //        y
      x2 = -x1; 
      y2 = y1;
    }
   pos_msg.data[0] = x2; 
   pos_msg.data[1] = y2;
    // July 16, 2015, this is a bug, retrieve the reference transformation 
   pos_msg.data[2] = -1*(yaw+M_PI);  
   pos_msg.data[3] = ++cnt;
  
   // 
   talk_2d.publish(pos_msg);
}

void init_transformation()
{
  // parameterized transformation 
  tf::Transform T_g2b;        // from global to base  
  tf::Matrix3x3 R_y, R_z; 
  R_y.setValue( 0, 0, 1,      // first, rotate along y 90'
                0, 1, 0, 
                -1, 0, 0);
  R_z.setValue(0, 1, 0,       // second, rotate along z -90'
              -1, 0, 0,
              0, 0, 1); 
  T_g2b = tf::Transform(R_y*R_z); 

  tf::Transform T_b2o;       // from base to original camera pose 
  double pitch; 
  ros::NodeHandle nh("~"); 
  nh.param("original_pitch_degree", pitch, 0.); 
  pitch = pitch * M_PI/ 180. ;  // degree to radius 
  
  float cp = cos(pitch);  
  float sp = sin(pitch); 

  tf::Matrix3x3 R_b2o; 
  R_b2o.setValue(1, 0, 0,
                 0, cp, -sp, 
                 0, sp, cp); 
  T_b2o = tf::Transform(R_b2o);
  
  // finally 
  T_g2o = T_g2b * T_b2o;
}




