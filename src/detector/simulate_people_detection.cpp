#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/people/ground_based_people_detection_app.h>
#include <pcl/common/time.h>

typedef pcl::PointXYZRGBA PointT;
typedef sensor_msgs::PointCloud2 PointCloudT;

// PCL viewer //
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("PCL Viewer"));

// Mutex: //
boost::mutex cloud_mutex;

ros::Publisher pub;

enum { COLS = 640, ROWS = 480 };

boost::shared_ptr<pcl::visualization::PCLVisualizer> Visualization ()
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("PCL Viewer Detections"));
  viewer->addCoordinateSystem (1.0);
  viewer->setCameraPosition(0,0,-2,0,-1,0,0);
  viewer->initCameraParameters ();
  return viewer;
}

void Visualization_update(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
  pcl::PointCloud<PointT>::ConstPtr cloud)
{
  // Create PCL Viewer Window
  viewer->removeAllPointClouds();
  viewer->removeAllShapes();
  pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud);
  viewer->addPointCloud<PointT> (cloud, rgb);
  // viewer->spinOnce();
}
////////////////////////////////////////////////////////////////////////////////
struct callback_args{
  // structure used to pass arguments to the callback function
  pcl::PointCloud<PointT>::Ptr clicked_points_3d;
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewerPtr;
};

void
pp_callback (const pcl::visualization::PointPickingEvent& event, void* args)
{
  struct callback_args* data = (struct callback_args *)args;
  if (event.getPointIndex () == -1)
    return;
  PointT current_point;
  event.getPoint(current_point.x, current_point.y, current_point.z);
  data->clicked_points_3d->points.push_back(current_point);
  // Draw clicked points in red:
  pcl::visualization::PointCloudColorHandlerCustom<PointT> red (data->clicked_points_3d, 255, 0, 0);
  data->viewerPtr->removePointCloud("clicked_points");
  data->viewerPtr->addPointCloud(data->clicked_points_3d, red, "clicked_points");
  data->viewerPtr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 0.2, "clicked_points");
  std::cout << current_point.x << " " << current_point.y << " " << current_point.z << std::endl;
}
////////////////////////////////////////////////////////////////////////////////

void detection (const PointCloudT::ConstPtr& input)
{
  // Algorithm parameters:
  std::string svm_filename = "/home/../catkin_ws/src/cimat_sort_rgbd/utils/people/data/trainedLinearSVMForPeopleDetectionWithHOG.yaml";
  float min_confidence = -1.5;
  float voxel_size = 0.06;
  Eigen::Matrix3f rgb_intrinsics_matrix;
    rgb_intrinsics_matrix << 525, 0.0, 316.7, 0.0, 525, 238.5, 0.0, 0.0, 1.0; // Kinect RGB camera intrinsics

  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  pcl::PointCloud<PointT> callback_cloud;
  pcl::fromROSMsg (*input, callback_cloud);

  // setting alpha = 1.0 for rviz
  for (size_t i = 0; i < callback_cloud.points.size(); ++i){
    callback_cloud.points[i].a = 255;
  }

  pub.publish(callback_cloud);
  viewer->initCameraParameters ();
  viewer->removePointCloud();
  viewer->addCoordinateSystem(1.0);

  pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
  *cloud = callback_cloud;

  // Wait for the first frame:
  boost::this_thread::sleep(boost::posix_time::milliseconds(10));

  //////////////////////////////////////////////////////////////////////////////
  cloud_mutex.lock ();    // for not overwriting the point cloud

  // Display pointcloud:
  pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud);
  viewer->addPointCloud<PointT> (cloud, rgb);
  viewer->setCameraPosition(0,0,-2,0,-1,0,0);

  // Add point picking callback to viewer:
  struct callback_args cb_args;
  pcl::PointCloud<PointT>::Ptr clicked_points_3d (new pcl::PointCloud<PointT>);
  *clicked_points_3d = callback_cloud;

  cb_args.clicked_points_3d = clicked_points_3d;
  cb_args.viewerPtr = viewer;
  // cb_args.viewerPtr = pcl::visualization::PCLVisualizer::Ptr(&_viewer_);
  viewer->registerPointPickingCallback (pp_callback, (void*)&cb_args);
  std::cout << "Shift+click on three floor points, then press 'Q'..." << std::endl;

  // Spin until 'Q' is pressed:
  viewer->spin();
  std::cout << "done." << std::endl;

  cloud_mutex.unlock ();

  // Ground plane estimation:
  Eigen::VectorXf ground_coeffs;
  ground_coeffs.resize(4);
  std::vector<int> clicked_points_indices;
  for (unsigned int i = 0; i < clicked_points_3d->points.size(); i++)
    clicked_points_indices.push_back(i);
  pcl::SampleConsensusModelPlane<PointT> model_plane(clicked_points_3d);
  model_plane.computeModelCoefficients(clicked_points_indices,ground_coeffs);
  std::cout << "Ground plane: " << ground_coeffs(0) << " " << ground_coeffs(1) << " " << ground_coeffs(2) << " " << ground_coeffs(3) << std::endl;
  //////////////////////////////////////////////////////////////////////////////

}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "simulate_people_detection");
  ros::NodeHandle nh;

  // Set ROS param
  ros::param::set("dist_th", 0.1);

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, detection);

  // Create a ROS publisher for the model coefficients
  pub = nh.advertise<pcl::PointCloud<PointT> > ("simulate_people_detection", 1);

  // Spin
  ros::spin ();
}
