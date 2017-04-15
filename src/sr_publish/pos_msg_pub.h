/*
 *  Jan. 13, 2016 David Z, 
 *  get msg from records, and then publish it 
 *
 * */

#include <ros/ros.h>
#include <fstream>
#include <vector>

using namespace std; 

class C2DPosPub
{
public:
  C2DPosPub();
  ~C2DPosPub();
  bool readRecord(); // read all records in log file
  void publishPos(int k);
  void publishPosOneLoop(double speed); // publish msg one loop at rate = speed

  vector<vector<float> > msgs_; 

  ros::Publisher r_pos_pub_;  // publish robot's current position and observation after projecting onto floor 
};



