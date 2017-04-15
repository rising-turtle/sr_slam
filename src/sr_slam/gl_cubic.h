/*
 *  David Z, April 15, 2016 
 *  
 *  use openGL to draw a cubic 
 *
 * */

#ifndef GL_CUBIC_H
#define GL_CUBIC_H

class CGLCubic
{
  public:
    CGLCubic(); 
    virtual ~CGLCubic(); 
    void drawCubic(int argc, char* argv[]);
    void setup(); 
    void resize(int w, int h);  // reshape callback function 
    void display();   // display callback function 
    void drawCube();  // draw a cube at (0, 0, 0) with w = h = l = 1; 
    void drawAxes();  // draw three axes at (0, 0, 0) with len = 1; 
    void idleCheck(); // during idle cycle, check whether need to repaint the screen 
  public:
    float roll_; 
    float pitch_; 
    float yaw_; 
  
  public:
    static CGLCubic * single_instance; 
    static void drawCallback(); 
    static void resizeCallback(int w, int h); 
    static void idleCallback();
    static bool b_needed_to_repaint_;     // 
    static void resetRPY(float r, float p, float y);  // reset rpy 
};


#endif
