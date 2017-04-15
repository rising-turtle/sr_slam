/*
 *  cube.c
 */
#include <GL/glut.h>
#include <GL/glu.h>
#include <GL/gl.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include        <sys/time.h>
// #include "set_ticker.c" 

static int rotationAngleX = 0;
static int rotationAngleY = 0;
static int rotationAngleZ = 0;
 
float g_fDistance = -4.5f;
float g_fSpinX = 0.0f;
float g_fSpinY = 0.0f;
float g_fSpinZ = 0.0f;
float zoomFactor = 1.0f;
 

void Display(void);
// void set_ticker( int n_msecs );

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
  
  printf("cubic.c: rotate X: %d, Y: %d, Z: %d\n", rotationAngleX, rotationAngleY, rotationAngleZ);
  signal(SIGALRM, rotate);
  // Display();
  // glFlush();
  glutPostRedisplay();
  alarm(1);
}


/*      Function:       DrawCube
        Purpose:        As the name would suggest, this is
                                the function for drawing the cubes.
*/

void DrawCube(float xPos, float yPos, float zPos)
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

static void draw3DRectangle(double dMinX_kls, double dMaxX_kls, double dMinY_kls, double dMaxY_kls, double dMinZ_kls, double dMaxZ_kls)
{ // werkt goed!!
 
  double MinX = fabs(dMinX_kls);
  double MaxX = fabs(dMaxX_kls);
  double MinY = fabs(dMinY_kls);
  double MaxY = fabs(dMaxY_kls);
  double MinZ = fabs(dMinZ_kls);
  double MaxZ = fabs(dMaxZ_kls);
 
  double sizeX = fabs( fabs(dMaxX_kls)-fabs(dMinX_kls) );
  double sizeY = fabs( fabs(dMaxY_kls)-fabs(dMinY_kls) );
  double sizeZ = fabs( fabs(dMaxZ_kls)-fabs(dMinZ_kls) );
 
  GLdouble V0[] = { 0.0, 0.0, 0.0};
  GLdouble V1[] = { sizeX, 0.0, 0.0};
  GLdouble V2[] = { sizeX, sizeY, 0.0};
  GLdouble V3[] = { 0.0, sizeY, 0.0};
  GLdouble V4[] = { 0.0, 0.0, sizeZ};
  GLdouble V5[] = { sizeX, 0.0, sizeZ};
  GLdouble V6[] = { sizeX, sizeY, sizeZ};
  GLdouble V7[] = { 0.0, sizeY, sizeZ};
 
  glPushMatrix();
  glTranslatef(dMinX_kls, dMinY_kls, dMinZ_kls);
 
  glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
 
  glBegin(GL_QUADS);
	glVertex3dv(V0); glVertex3dv(V1); glVertex3dv(V2); glVertex3dv(V3);        // Surface 1
	glVertex3dv(V1); glVertex3dv(V5); glVertex3dv(V6); glVertex3dv(V2);        // Surface 2
	glVertex3dv(V5); glVertex3dv(V4); glVertex3dv(V7); glVertex3dv(V6);        // Surface 3
	glVertex3dv(V4); glVertex3dv(V0); glVertex3dv(V3); glVertex3dv(V7);        // Surface 4
	glVertex3dv(V3); glVertex3dv(V2); glVertex3dv(V6); glVertex3dv(V7);        // Surface 5
	glVertex3dv(V0); glVertex3dv(V4); glVertex3dv(V5); glVertex3dv(V1);        // Surface 6
  glEnd();
 
  glPopMatrix();
 
}
 
void drawaxes(void)
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
 
 
void Display(void)
{
    glEnable(GL_BLEND); // Enable the OpenGL Blending functionality  
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); // Set the blend mode to blend our current RGBA with what is already in the buffer  
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glColor3f(0.0, 1.0, 0.0);
    glLoadIdentity();
    glRotatef(90, 0.0, 0.0, 1.0); // rotate around z axis
    glPushMatrix(); 
        //gluLookAt(3.0, 2.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
	//gluLookAt(3.0, 2.0, 8.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
	//gluLookAt(0.0, 0.0, 3.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
        // gluLookAt(0, 0, 3.0, 0.0, 0.0, 0.0, 0, 1.0, 0.0);
 
	// rotate complete axis system
	// glRotatef(-90, 1.0, 0.0, 0.0); // rotate around x axis
       glRotatef(rotationAngleZ, 0., 0., 1); 
       glRotatef(rotationAngleY, 0., 1., 0); 
       glRotatef(rotationAngleX, 1., 0., 0); 
       glPushMatrix(); 
         // glTranslatef(-0.5f, -0.5f, -0.5f);
         glScalef(0.6f,0.4f, 0.4f);  // scale to cube: width, height, length
         glTranslatef(0.5f, 0.5f, 0.5f); // draw a cube origen start at (0.5, 0.5, 0.5)
         DrawCube(0, 0, 0);   // draw a cube from (0,0,0) to (-1,-1,-1)
       glPopMatrix(); 
	// cabinets
       // glColor3f(0.5, 0.5, 0.5); // grey
       // draw3DRectangle(0, 1.0, 0, 0.5, 0.0, 0.5); 
      glScalef(0.6f, 0.6f, 0.6f); 
      drawaxes();

    glPopMatrix(); 
    gluLookAt(0, 0, 5.0, 0.0, 0.0, 0.0, 0, 1.0, 0.0);

    // glFlush();
    glutSwapBuffers();
}
 
void Init(void)
{
    glClearColor(0.0, 0.0, 0.0, 0.0);
    glEnable(GL_DEPTH_TEST);   // Enable depth testing for z-culling
    glDepthFunc(GL_LEQUAL);    // Set the type of depth-test
    // glShadeModel(GL_SMOOTH);   // Enable smooth shading
}
 
void Resize(int width, int height) // called at the first display, and resize the screen 
{
    glViewport(0, 0, width, height);
    if (height == 0) height = 1;
    // gluLookAt(0, 0, 5.0, 0.0, 0.0, 0.0, 0, 1.0, 0.0);

    glMatrixMode(GL_PROJECTION); // projection model 
    glLoadIdentity();                 
    gluPerspective(45.0, width/height, 0.1, 10.0); // project models into camera screen 

    // glRotatef(rotationAngleZ, 0, 0, 1); // rotate around z axis 
    // glRotatef(rotationAngleY, 0, 1, 0); // rotate around y axis 
    // glRotatef(rotationAngleX, 1, 0, 0); // rotate around x axis

    // glOrtho(-3, 3, -1.5, 1.5, -1, 1000.0);
    // gluLookAt(-1.0, -1.0, 3.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0);

    // glMatrixMode(GL_MODELVIEW);
    // glLoadIdentity();
}
 
int main(int argc, char **argv)
{
    
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH); // double: buffer, used to swap, depth: z-buffer 
    glutInitWindowSize(800, 800);
    glutInitWindowPosition(200, 200);
    glutCreateWindow("3D Object in OpenGL");
    Init();
    // gluLookAt(0, 0, 3.0, 0.0, 0.0, 0.0, 0, 1.0, 0.0);
    // glOrtho(-3, 3, -1.5, 1.5, -1, 1000.0);
    // gluPerspective(60.0, 1, 0.1, 10.0);

    glutDisplayFunc(Display);
    glutReshapeFunc(Resize);
    
    signal(SIGALRM, rotate); 
    // set_ticker(1000 / 10); 
    alarm(1);

    glutMainLoop();
    return 0;
}



