
#include "qtros.h"
#include <QApplication>
#include <QObject>
#include "qt_gui.h"
#include "glviewer.h"
#include "parameter_server.h"

#include "graph_plane.h"

void ui_connections(QObject* ui, CGraphPlane* graph_mgr);
void gui_connections(Graphical_UI* gui, CGraphPlane* graph_mgr);

void initParameters();  // set parameters that suitable for this app


int main(int argc, char** argv)
{
  setlocale(LC_NUMERIC,"C");//Avoid expecting german decimal separators in launch files

  //create thread object, to run the ros event processing loop in parallel to the qt loop
  QtROS qtRos(argc, argv, "sr_slam_plane"); //ros node name & namespace

  //Depending an use_gui on the Parameter Server, a gui- or a headless application is used
  QApplication app(argc, argv, ParameterServer::instance()->get<bool>("use_gui")); 

  // initParameters(); // call before the other instance is initialized 

  // GraphManager graph_mgr;
  // GraphManager* graph_mgr = new CGraphWrapper;
  CGraphPlane* graph_mgr = new CGraphPlane;
  //Instantiate the kinect image listener
  
  Graphical_UI* gui = NULL;
  if (app.type() == QApplication::GuiClient)
  {
    gui = new Graphical_UI();
    gui->show();
    // gui_connections(gui, graph_mgr, listener);
    gui_connections(gui, graph_mgr);
    // ui_connections(gui, graph_mgr, listener);//common connections for the user interfaces
    ui_connections(gui, graph_mgr);
  } else {
    ROS_WARN("Running without graphical user interface! See README or wiki page for how to interact with RGBDSLAM.");
  }
  //Create Ros service interface with or without gui
  // RosUi ui("rgbdslam"); //ui namespace for service calls
  // ui_connections(&ui, &graph_mgr, &listener);//common connections for the user interfaces
  // ui_connections(&ui, graph_mgr, listener);
  
  graph_mgr->loadSkData(); // create another thread to load data from disk
  
  // const float max_dist_m = ParameterServer::instance()->get<double>("max_dist_for_inliers"); 
  // ROS_WARN("main_plane.cpp: get max_dist_m %f", max_dist_m);

  //If one thread receives a exit signal from the user, signal the other thread to quit too
  QObject::connect(&app, SIGNAL(aboutToQuit()), &qtRos, SLOT(quitNow()));
  QObject::connect(&qtRos, SIGNAL(rosQuits()), &app, SLOT(quit()));

  qtRos.start();// Run main loop.
  app.exec();
  //if(ros::ok()) ros::shutdown();//If not yet done through the qt connection
  //ros::waitForShutdown(); //not sure if necessary. 
}

void initParameters()
{
  
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


