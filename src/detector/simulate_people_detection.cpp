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
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

// Mutex: //
boost::mutex cloud_mutex;
ros::Publisher pub;

enum { COLS = 640, ROWS = 480 };

boost::shared_ptr<pcl::visualization::PCLVisualizer> Visualization ()
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("PCL Viewer"));
  viewer->addCoordinateSystem (1.0);
  viewer->setCameraPosition(0,0,-2,0,-1,0,0);
  viewer->initCameraParameters ();
  return viewer;
}

void Visualization_update(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
  pcl::PointCloud<PointT>::ConstPtr cloud)
{
  // Create PCL Viewer Window
  pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud);
  viewer->removePointCloud();
  viewer->addPointCloud<PointT> (cloud, rgb);
  viewer->spinOnce();
}

void cloud_cb_ (const PointCloudT::ConstPtr& input)
{
  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  pcl::PointCloud<PointT> callback_cloud;
  pcl::fromROSMsg (*input, callback_cloud);

  pcl::PointCloud<PointT>::Ptr _cloud_ (new pcl::PointCloud<PointT>);
  *_cloud_ = callback_cloud;

  // setting alpha = 1.0 for rviz
  for (size_t i = 0; i < callback_cloud.points.size(); ++i){
    callback_cloud.points[i].a = 255;
  }
  pub.publish(callback_cloud);

  // Display pointcloud:
  Visualization_update(viewer, _cloud_);
}

int main (int argc, char** argv)
{
  // Initialization of PCL Viewer
  viewer = Visualization();

  // Initialize ROS
  ros::init (argc, argv, "simulate_people_detection");
  ros::NodeHandle nh;

  // Set ROS param
  ros::param::set("dist_th", 0.1);

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb_);

  // Create a ROS publisher for the model coefficients
  pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGBA> > ("simulate_people_detection", 1);

  // Spin
  ros::spin ();
}
