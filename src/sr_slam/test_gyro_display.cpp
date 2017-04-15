/*
 *
 *  test whether the repaint of the cube works well 
 *
 * */

#include <ros/ros.h>
#include "ros_gl_cube.h"
#include <GL/glut.h>
#include <signal.h>
#include <sys/time.h>

static int rotationAngleX = 0;
static int rotationAngleY = 0;
static int rotationAngleZ = 0;

void rotate(int signum);

int main(int argc, char* argv[])
{
   ros::init(argc, argv, "test_gyro_display", ros::init_options::NoSigintHandler); 
   ros::NodeHandle n;
  
   CGLCubic* pcube = new CRosGLCube; 
   pcube->setup(); 

   // signal(SIGALRM, rotate); 
   // set_ticker(1000 / 10); 
   // alarm(1);

   pcube->drawCubic(argc, argv);
   // printf("test_gyro_display.cpp: executate after mainloop!\n");
   
   return 0; 
}


void rotate(int signum)
{
  static int rotate_XYZ= 1; 
  static int rotate_ANGLE = 5; 
  static int rotate_max = 360; 
  signal(SIGALRM, SIG_IGN); 
  if(rotate_XYZ == 1) // rotate Z 
  {
    if(rotationAngleZ + rotate_ANGLE == rotate_max) 
    {
      rotationAngleZ = 0; 
      rotate_XYZ +=1; 
    }else
      rotationAngleZ += rotate_ANGLE; 
  }else if(rotate_XYZ == 2) // rotate X 
  {
    if(rotationAngleX + rotate_ANGLE == rotate_max)
    {
      rotationAngleX = 0; 
      rotate_XYZ += 1; 
    }else
      rotationAngleX += rotate_ANGLE; 
  }else if(rotate_XYZ == 3) // rotate Y
  {
    if(rotationAngleY + rotate_ANGLE == rotate_max) 
    {
      rotationAngleY = 0; 
      rotate_XYZ = 1; 
    }else
      rotationAngleY += rotate_ANGLE; 
  }
  
  printf("test_gyro_display.cpp: rotate X: %d, Y: %d, Z: %d\n", rotationAngleX, rotationAngleY, rotationAngleZ);
  CGLCubic::resetRPY(rotationAngleX, rotationAngleY, rotationAngleZ);
  signal(SIGALRM, rotate);
  glutPostRedisplay();
  alarm(1);
}




