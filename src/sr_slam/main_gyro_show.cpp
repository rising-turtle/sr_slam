
#include "qtros.h"
#include <QApplication>
#include <QObject>
#include "qt_gui.h"
#include "glviewer.h"
#include "parameter_server.h"

#include "global_def.h"
#include "graph_plane.h"
#include "gl_cubic.h"
#include "ros_gl_cube.h"
#include "gyro_euler.h"
#include "misc.h"
#include "std_msgs/Float32MultiArray.h"

void ui_connections(QObject* ui, CGraphPlane* graph_mgr);
void gui_connections(Graphical_UI* gui, CGraphPlane* graph_mgr);
pointcloud_type* create_rectangle();

void publishIMUReading(string f); 

int main(int argc, char** argv)
{
  setlocale(LC_NUMERIC,"C");//Avoid expecting german decimal separators in launch files

  //create thread object, to run the ros event processing loop in parallel to the qt loop
  QtROS qtRos(argc, argv, "sr_gyro_show"); //ros node name & namespace

  //Depending an use_gui on the Parameter Server, a gui- or a headless application is used
  QApplication app(argc, argv, ParameterServer::instance()->get<bool>("use_gui")); 

  // initParameters(); // call before the other instance is initialized 

  // GraphManager* graph_mgr = new CGraphWrapper;
  CGraphPlane* graph_mgr = new CGraphPlane; 
  Graphical_UI* gui = NULL;
  {
    gui = new Graphical_UI();
    gui->show();
    gui_connections(gui, graph_mgr);
    ui_connections(gui, graph_mgr);
  } 
  
  // test function 
  // QtConcurrent::run(graph_mgr, &CGraphPlane::imu_gui_display);
  
  // CGLCubic* p_cube = new CRosGLCube;
  // QtConcurrent::run(*p_cube, &CGLCubic::drawCubic, argc, argv);  // create openGL display thread 
  // usleep(5000);

  // string file = "/media/work/work/data/sr4000/imu_fuse/imu_creep_ob1.dat";
  // QtConcurrent::run(publishIMUReading, file);     // create a thread to read IMU data 
  graph_mgr->loadSkData(); 

  //If one thread receives a exit signal from the user, signal the other thread to quit too
  QObject::connect(&app, SIGNAL(aboutToQuit()), &qtRos, SLOT(quitNow()));
  QObject::connect(&qtRos, SIGNAL(rosQuits()), &app, SLOT(quit()));

  qtRos.start();// Run main loop.
  app.exec();
}


void publishIMUReading(string f)
{
  CGyroEuler ge; 
  if(!ge.readGyro(f))
  {
    ROS_ERROR("main_gyro_show.cpp: failed to load IMU data %s", f.c_str()); 
    return ;
  }

  // first compute bias 
  ge.computeBias();

  // declare a publisher 
  ros::Publisher euler_pub; 
  ros::NodeHandle n; 
  euler_pub = n.advertise<std_msgs::Float32MultiArray>("/euler_msg", 10);
  
  std_msgs::Float32MultiArray msg; 
  msg.data.resize(3);

  ros::Rate rate(20);
  float rpy[3] = {0}; 
  for(int i=0; ; i++)
  {
    if(!ge.getEulerAt(i , rpy) || !ros::ok())
    {
      ROS_WARN("main_gyro_show.cpp: finish read IMU data."); 
      break; 
    }
    // display the IMU data 
    ROS_WARN("main_gyro_show.cpp: read %d IMU: %f %f %f", i, R2D(rpy[0]), R2D(rpy[1]), R2D(rpy[2]));

    // publish a msg 
    msg.data[0] = rpy[0]; msg.data[1] = rpy[1]; msg.data[2] = rpy[2]; 
    euler_pub.publish(msg);
    ros::spinOnce(); 
    rate.sleep(); 
  }

}

pointcloud_type *  create_rectangle()
{
  pointcloud_type* rectangle = new pointcloud_type(); 
  float l, w , h;
  l = w = h = 1; 
  point_type p1(-l, -w, -h);   point_type p2(-l, -w, h);   point_type p3(-l, w, -h);   point_type p4(-l, w, h); 
  point_type p5( l, -w, -h);   point_type p6( l, -w, h);   point_type p7( l, w, -h);   point_type p8( l, w, h); 
  rectangle->points.push_back(p1);  rectangle->points.push_back(p2);   rectangle->points.push_back(p3);
  rectangle->points.push_back(p4);  rectangle->points.push_back(p5);   rectangle->points.push_back(p6);
  rectangle->points.push_back(p7);  rectangle->points.push_back(p8);
  return rectangle;
}


void ui_connections(QObject* ui, CGraphPlane* graph_mgr)
{
  Qt::ConnectionType ctype = Qt::AutoConnection;
  if (ParameterServer::instance()->get<bool>("concurrent_io")) 
    ctype = Qt::DirectConnection;
  QObject::connect(ui, SIGNAL(reset()), graph_mgr, SLOT(reset()), ctype);
  QObject::connect(ui, SIGNAL(optimizeGraph()), graph_mgr, SLOT(optimizeGraph()), ctype);
  // QObject::connect(ui, SIGNAL(togglePause()), listener, SLOT(togglePause()), ctype);
  QObject::connect(ui, SIGNAL(togglePause()), graph_mgr, SLOT(togglePause()), ctype);

  // QObject::connect(ui, SIGNAL(toggleBagRecording()), listener, SLOT(toggleBagRecording()), ctype);
  // QObject::connect(ui, SIGNAL(getOneFrame()), listener, SLOT(getOneFrame()), ctype);
  // QObject::connect(ui, SIGNAL(getOneFrame()), listener, SLOT(getOneFrame()), ctype);
  QObject::connect(ui, SIGNAL(getOneFrame()), graph_mgr, SLOT(getOneFrame()), ctype);

  QObject::connect(ui, SIGNAL(deleteLastFrame()), graph_mgr, SLOT(deleteLastFrame()), ctype);
  QObject::connect(ui, SIGNAL(sendAllClouds()), graph_mgr, SLOT(sendAllClouds()), ctype);
  QObject::connect(ui, SIGNAL(saveAllClouds(QString)), graph_mgr, SLOT(saveAllClouds(QString)), ctype);
  // QObject::connect(ui, SIGNAL(saveOctomapSig(QString)), graph_mgr, SLOT(saveOctomap(QString)), ctype);
  QObject::connect(ui, SIGNAL(saveAllFeatures(QString)), graph_mgr, SLOT(saveAllFeatures(QString)), ctype);
  QObject::connect(ui, SIGNAL(saveIndividualClouds(QString)), graph_mgr, SLOT(saveIndividualClouds(QString)), ctype);
  QObject::connect(ui, SIGNAL(saveTrajectory(QString)), graph_mgr, SLOT(saveTrajectory(QString)), ctype);
  QObject::connect(ui, SIGNAL(toggleMapping(bool)), graph_mgr, SLOT(toggleMapping(bool)), ctype);
  QObject::connect(ui, SIGNAL(saveG2OGraph(QString)), graph_mgr, SLOT(saveG2OGraph(QString)), ctype);
} 
void gui_connections(Graphical_UI* gui, CGraphPlane* graph_mgr)
{
  // QObject::connect(listener,  SIGNAL(newVisualImage(QImage)), gui, SLOT(setVisualImage(QImage)));
  QObject::connect(graph_mgr,  SIGNAL(newVisualImage(QImage)), gui, SLOT(setVisualImage(QImage)));
  
  // QObject::connect(listener,  SIGNAL(newFeatureFlowImage(QImage)), gui, SLOT(setFeatureFlowImage(QImage)));
  QObject::connect(graph_mgr,  SIGNAL(newFeatureFlowImage(QImage)), gui, SLOT(setFeatureFlowImage(QImage)));

  // QObject::connect(listener,  SIGNAL(newDepthImage(QImage)), gui, SLOT(setDepthImage(QImage)));
  QObject::connect(graph_mgr,  SIGNAL(newDepthImage(QImage)), gui, SLOT(setDepthImage(QImage)));

  QObject::connect(graph_mgr, SIGNAL(sendFinished()), gui, SLOT(sendFinished()));
  QObject::connect(graph_mgr, SIGNAL(iamBusy(int, const char*, int)), gui, SLOT(showBusy(int, const char*, int)));
  QObject::connect(graph_mgr, SIGNAL(progress(int, const char*, int)), gui, SLOT(setBusy(int, const char*, int)));
  QObject::connect(graph_mgr, SIGNAL(setGUIInfo(QString)), gui, SLOT(setInfo(QString)));
  QObject::connect(graph_mgr, SIGNAL(setGUIStatus(QString)), gui, SLOT(setStatus(QString)));
  QObject::connect(gui, SIGNAL(printEdgeErrors(QString)), graph_mgr, SLOT(printEdgeErrors(QString)));
  QObject::connect(gui, SIGNAL(pruneEdgesWithErrorAbove(float)), graph_mgr, SLOT(pruneEdgesWithErrorAbove(float)));
  QObject::connect(gui, SIGNAL(clearClouds()), graph_mgr, SLOT(clearPointClouds()));
  if (ParameterServer::instance()->get<bool>("use_glwidget") && gui->getGLViewer() != NULL) {
    GLViewer* glv = gui->getGLViewer();
    QObject::connect(graph_mgr, SIGNAL(setPointCloud(pointcloud_type *, QMatrix4x4)), glv, SLOT(addPointCloud(pointcloud_type *, QMatrix4x4)), Qt::BlockingQueuedConnection ); //Needs to block, otherwise the opengl list compilation makes the app unresponsive. This effectively throttles the processing rate though
    typedef const std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >* cnst_ft_vectors;
    QObject::connect(graph_mgr, SIGNAL(setFeatures(const std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >*)), glv, SLOT(addFeatures(const std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >*))); //, Qt::DirectConnection);
    QObject::connect(graph_mgr, SIGNAL(setGraphEdges(const QList<QPair<int, int> >*)), glv, SLOT(setEdges(const QList<QPair<int, int> >*)));
    QObject::connect(graph_mgr, SIGNAL(updateTransforms(QList<QMatrix4x4>*)), glv, SLOT(updateTransforms(QList<QMatrix4x4>*)));
    QObject::connect(graph_mgr, SIGNAL(deleteLastNode()), glv, SLOT(deleteLastNode()));
    QObject::connect(graph_mgr, SIGNAL(resetGLViewer()),  glv, SLOT(reset()));
    if(!ParameterServer::instance()->get<bool>("store_pointclouds")) {
        QObject::connect(glv, SIGNAL(cloudRendered(pointcloud_type const *)), graph_mgr, SLOT(clearPointCloud(pointcloud_type const *))); // 
    } else if(ParameterServer::instance()->get<double>("voxelfilter_size") > 0.0) {
        QObject::connect(glv, SIGNAL(cloudRendered(pointcloud_type const *)), graph_mgr, SLOT(reducePointCloud(pointcloud_type const *))); // 
    }
  }
  // QObject::connect(listener, SIGNAL(setGUIInfo(QString)), gui, SLOT(setInfo(QString)));
  QObject::connect(graph_mgr, SIGNAL(setGUIInfo(QString)), gui, SLOT(setInfo(QString)));

  // QObject::connect(listener, SIGNAL(setGUIStatus(QString)), gui, SLOT(setStatus(QString)));
  QObject::connect(graph_mgr, SIGNAL(setGUIStatus(QString)), gui, SLOT(setStatus(QString)));

  QObject::connect(graph_mgr, SIGNAL(setGUIInfo2(QString)), gui, SLOT(setInfo2(QString)));
}


