#include "gl_cubic.h"

#include <GL/gl.h>
#include <GL/glut.h>
#include <GL/glu.h>
#include <stdio.h>
#include <unistd.h>
// #include <conio.h>//needed for getch
#include <ros/ros.h>

CGLCubic::CGLCubic()
{
  roll_ = pitch_ = yaw_ = 0;
}
CGLCubic::~CGLCubic(){}

static CGLCubic* CGLCubic::single_instance = 0; 
static bool CGLCubic::b_needed_to_repaint_ = false;

void CGLCubic::setup()
{
  // glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
  CGLCubic::single_instance = this; 
}

/*  
 *  Function:       DrawCube
    Purpose:        As the name would suggest, this is
                                the function for drawing the cubes.
*/
void CGLCubic::drawCube()
{
        glPushMatrix();
        // glTranslatef(xPos, yPos, zPos);
        // glBegin(GL_POLYGON);
        glBegin(GL_QUADS); // has to be GL_QUADS, to draw a rectangle in 3D space, not GL_POLYGON

                /*      This is the top face*/
                glColor3f(1.0f, 1.0f, 0.0f);     // Yellow

                glVertex3f(0.0f, 0.0f, 0.0f);
                glVertex3f(0.0f, 0.0f, -1.0f);
                glVertex3f(-1.0f, 0.0f, -1.0f);
                glVertex3f(-1.0f, 0.0f, 0.0f);

                /*      This is the front face*/
                glColor3f(1.0f, 0.5f, 0.0f);     // Orange
                glVertex3f(0.0f, 0.0f, 0.0f);
                glVertex3f(-1.0f, 0.0f, 0.0f);
                glVertex3f(-1.0f, -1.0f, 0.0f);
                glVertex3f(0.0f, -1.0f, 0.0f);

                /*      This is the right face*/
                glColor3f(0.0f, 1.0f, 0.0f);     // Green

                glVertex3f(0.0f, 0.0f, 0.0f);
                glVertex3f(0.0f, -1.0f, 0.0f);
                glVertex3f(0.0f, -1.0f, -1.0f);
                glVertex3f(0.0f, 0.0f, -1.0f);

          // glEnd(); 

          //      glBegin(GL_QUADS); 
                /*      This is the left face*/
                glColor3f(1.0f, 0.0f, 0.0f);     // Red

                // glVertex3f(0, 0,0);
                glVertex3f(-1.0f, 0.0f, 0.0f);
                glVertex3f(-1.0f, 0.0f, -1.0f);
                glVertex3f(-1.0f, -1.0f, -1.0f);
                glVertex3f(-1.0f, -1.0f, 0.0f);

                /*      This is the bottom face*/
                glColor3f(1.0f, 0.0f, 1.0f);     // Magenta
                // glVertex3f(0, 0,0);
                glVertex3f(0.0f, -1.0f, 0.0f);
                glVertex3f(0.0f, -1.0f, -1.0f);
                glVertex3f(-1.0f, -1.0f, -1.0f);
                glVertex3f(-1.0f, -1.0f, 0.0f);

                /*      This is the back face*/
                glColor3f(0.0f, 0.0f, 1.0f);     // Blue

                // glVertex3f(0, 0,0);
                glVertex3f(0.0f, 0.0f, -1.0f);
                glVertex3f(-1.0f, 0.0f, -1.0f);
                glVertex3f(-1.0f, -1.0f, -1.0f);
                glVertex3f(0.0f, -1.0f, -1.0f);

        glEnd();
        glPopMatrix();
}

void CGLCubic::drawAxes(void)
{
    glColor3ub(255, 0, 0);
    glBegin(GL_LINE_STRIP);
    glVertex3f(0.0, 0.0, 0.0);
    glVertex3f(1.0, 0.0, 0.0);
    glVertex3f(0.75, 0.25, 0.0);
    glVertex3f(0.75, -0.25, 0.0);
    glVertex3f(1.0, 0.0, 0.0);
    glVertex3f(0.75, 0.0, 0.25);
    glVertex3f(0.75, 0.0, -0.25);
    glVertex3f(1.0, 0.0, 0.0);
    glEnd();
    glColor3ub(0, 255, 0);
    glBegin(GL_LINE_STRIP);
    glVertex3f(0.0, 0.0, 0.0);
    glVertex3f(0.0, 1.0, 0.0);
    glVertex3f(0.0, 0.75, 0.25);
    glVertex3f(0.0, 0.75, -0.25);
    glVertex3f(0.0, 1.0, 0.0);
    glVertex3f(0.25, 0.75, 0.0);
    glVertex3f(-0.25, 0.75, 0.0);
    glVertex3f(0.0, 1.0, 0.0);
    glEnd();
    glColor3ub(0, 0, 255);
    glBegin(GL_LINE_STRIP);
    glVertex3f(0.0, 0.0, 0.0);
    glVertex3f(0.0, 0.0, 1.0);
    glVertex3f(0.25, 0.0, 0.75);
    glVertex3f(-0.25, 0.0, 0.75);
    glVertex3f(0.0, 0.0, 1.0);
    glVertex3f(0.0, 0.25, 0.75);
    glVertex3f(0.0, -0.25, 0.75);
    glVertex3f(0.0, 0.0, 1.0);
    glEnd();
 
    glColor3ub(255, 255, 0);
    glRasterPos3f(1.1, 0.0, 0.0);
    glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, 'x');
    glRasterPos3f(0.0, 1.1, 0.0);
    glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, 'y');
    glRasterPos3f(0.0, 0.0, 1.1);
    glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, 'z');
}
 

void CGLCubic::display()
{
  // printf("gl_cubic.cpp: in display(), hey, let's paint!\n");
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  /* drawing commands would go here, if we had any yet... */
  
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity(); 
  glTranslatef(0 , 0, -1);
  glRotatef(90, 0., 0., 1.0);     // x axis towards to forward, rotate around z axis 
  glRotatef(180, 1., 0., 0);      // x forward, y right, z downward

  glPushMatrix(); 

    // rotation using euler angles 
     glRotatef(roll_, 1, 0, 0); 
     glRotatef(pitch_, 0, 1, 0); 
     glRotatef(yaw_, 0, 0, 1); 

    // draw the cube 
    glPushMatrix();
      glScalef(0.6, 0.4, 0.4); // scale to cube: width, height, length
      glTranslatef(0.5f, 0.5f, 0.5f); // draw a cube origen start at (0.5, 0.5, 0.5)
      drawCube();   // draw a cube from (0,0,0) to (-1,-1,-1)
    glPopMatrix(); 

   // Draw the triangle !
   // glDrawArrays(GL_TRIANGLES, 0, 12*3); // 12*3 indices starting at 0 -> 12 triangles -> 6 squares
  
    // draw the axes 
    glScalef(0.6f, 0.6f, 0.6f); 
    drawAxes(); 

  glPopMatrix(); 
  glutSwapBuffers();
}

void CGLCubic::idleCheck()
{
  if(b_needed_to_repaint_)
  {
    printf("gl_cubic.cpp: needed to repaint the screen in idleCallback\n");
    // glutPostRedisplay(); 
    b_needed_to_repaint_ = false;
  }

  ros::spinOnce(); // call ros msg dispatch 
}

void CGLCubic::resize(int width, int height)
{
    glViewport(0, 0, width, height);
    if (height == 0) height = 1;
    // gluLookAt(0, 0, 5.0, 0.0, 0.0, 0.0, 0, 1.0, 0.0);

    glMatrixMode(GL_PROJECTION); // projection model 
    glLoadIdentity();                 
    // gluPerspective(45.0, width/height, 0.1, 10.0); // project models into camera screen 
    glOrtho(-1.2, 1.2, -1.2, 1.2, -2, 2);
}

void CGLCubic::drawCubic(int argc, char* argv[])
{
  printf("gl_cubic.cpp: in drawCubic, glutInit called!\n");
  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);
  glutInitWindowSize(800,800);
  glutCreateWindow("Cube for testing Euler Angle");

  glClearColor(0.0, 0.0, 0.0, 0.0);
  glEnable(GL_DEPTH_TEST);   // Enable depth testing for z-culling
  // glDepthFunc(GL_LEQUAL);    // Set the type of depth-test
  glDepthFunc(GL_TRUE);

  setup(); 
  glutDisplayFunc(CGLCubic::drawCallback);
  glutReshapeFunc(CGLCubic::resizeCallback);
  glutIdleFunc(CGLCubic::idleCallback);

  
  //for(int i=0; i<36; i++)
  {
    // yaw_ += 50; 
    // glutPostRedisplay(); 
    // usleep(10000);
  }

  glutMainLoop();

  // getch();//pause here to see results or lack there of
}

void CGLCubic::drawCallback()
{
  single_instance->display();
}

void CGLCubic::resizeCallback(int w, int h)
{
  single_instance->resize(w, h); 
}

void CGLCubic::idleCallback()
{
  single_instance->idleCheck();
}

void CGLCubic::resetRPY(float r, float p, float y)
{
  single_instance->roll_ = r; 
  single_instance->pitch_ = p; 
  single_instance->yaw_ = y;
}
