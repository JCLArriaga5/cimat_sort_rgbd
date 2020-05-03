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

// Ground plane init //
Eigen::VectorXf ground_coeffs;

ros::Publisher pub;

enum { COLS = 640, ROWS = 480 };

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

void gp_estimation_clicked_points (const pcl::PointCloud<PointT> &callback_cloud)
{
  // Initialize Viewer
  viewer->initCameraParameters ();
  viewer->removeAllPointClouds();
  viewer->addCoordinateSystem(1.0);

  // Display pointcloud:
  cloud_mutex.lock ();    // for not overwriting the point cloud

  pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
  *cloud = callback_cloud;

  pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud);
  viewer->addPointCloud<PointT> (cloud, rgb);
  viewer->setCameraPosition(0,0,-2,0,-1,0,0);

  // Add point picking callback to viewer:
  struct callback_args cb_args;
  pcl::PointCloud<PointT>::Ptr clicked_points_3d (new pcl::PointCloud<PointT>);
  *clicked_points_3d = callback_cloud;

  cb_args.clicked_points_3d = clicked_points_3d;
  cb_args.viewerPtr = pcl::visualization::PCLVisualizer::Ptr(viewer);
  viewer->registerPointPickingCallback (pp_callback, (void*)&cb_args);
  std::cout << "Shift+click on three floor points, then press 'Q'..." << std::endl;

  // Spin until 'Q' is pressed:
  viewer->spin();
  std::cout << "done." << std::endl;

  cloud_mutex.unlock ();

  // Ground plane estimation:
  ground_coeffs.resize(4);
  std::vector<int> clicked_points_indices;
  for (unsigned int i = 0; i < clicked_points_3d->points.size(); i++)
    clicked_points_indices.push_back(i);
  pcl::SampleConsensusModelPlane<PointT> model_plane(clicked_points_3d);
  model_plane.computeModelCoefficients(clicked_points_indices,ground_coeffs);
  return;
}

void cloud_cb (const PointCloudT::ConstPtr& input)
{
  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  pcl::PointCloud<PointT> callback_cloud;
  pcl::fromROSMsg (*input, callback_cloud);

  // Proofs generate ground plane estimation with clicked_points
  gp_estimation_clicked_points(callback_cloud);
  std::cout << "Ground plane: " << ground_coeffs(0) << " " << ground_coeffs(1) << " " << ground_coeffs(2) << " " << ground_coeffs(3) << std::endl;

  // setting alpha = 1.0 for rviz
  for (size_t i = 0; i < callback_cloud.points.size(); ++i){
    callback_cloud.points[i].a = 255;
  }

}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "simulate_people_detection");
  ros::NodeHandle nh;

  // Set ROS param
  // ros::param::set("dist_th", 0.1);

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  // Create a ROS publisher for the model coefficients
  pub = nh.advertise<pcl::PointCloud<PointT> > ("simulate_people_detection", 1);

  // Spin
  ros::spin ();
}
