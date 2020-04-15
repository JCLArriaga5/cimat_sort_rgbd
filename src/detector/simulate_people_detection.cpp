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
// boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
pcl::visualization::PCLVisualizer viewer("PCL Viewer");

// Mutex: //
boost::mutex cloud_mutex;
ros::Publisher pub;

enum { COLS = 640, ROWS = 480 };

int print_help()
{
  std::cout << "*******************************************************" << std::endl;
  std::cout << "Ground based people detection app options:" << std::endl;
  std::cout << "   --help    <show_this_help>" << std::endl;
  std::cout << "   --svm     <path_to_svm_file>" << std::endl;
  std::cout << "   --conf    <minimum_HOG_confidence (default = -1.5)>" << std::endl;
  std::cout << "   --min_h   <minimum_person_height (default = 1.3)>" << std::endl;
  std::cout << "   --max_h   <maximum_person_height (default = 2.3)>" << std::endl;
  std::cout << "*******************************************************" << std::endl;
  return 0;
}

void cloud_cb_ (const PointCloudT::ConstPtr& input)
{
  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  pcl::PointCloud<PointT> callback_cloud;
  pcl::fromROSMsg (*input, callback_cloud);

  pcl::PointCloud<PointT>::Ptr cloud;

  cloud_mutex.lock ();    // for not overwriting the point cloud from another thread
  *cloud = callback_cloud;
  bool new_cloud_available_flag = true;
  cloud_mutex.unlock ();

  // setting alpha = 1.0 for rviz
  for (size_t i = 0; i < callback_cloud.points.size(); ++i){
    callback_cloud.points[i].a = 255;
  }

  pub.publish(cloud);
}

struct callback_args{
  // structure used to pass arguments to the callback function
  pcl::PointCloud<PointT>::Ptr clicked_points_3d;
  pcl::visualization::PCLVisualizer::Ptr viewerPtr;
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
  data->viewerPtr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "clicked_points");
  std::cout << current_point.x << " " << current_point.y << " " << current_point.z << std::endl;
}

int main (int argc, char** argv)
{
  if(pcl::console::find_switch (argc, argv, "--help") || pcl::console::find_switch (argc, argv, "-h"))
        return print_help();

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

  // Algorithm parameters:
  std::string svm_filename = "/utils/people/data/trainedLinearSVMForPeopleDetectionWithHOG.yaml";
  float min_confidence = -1.5;
  float voxel_size = 0.06;
  Eigen::Matrix3f rgb_intrinsics_matrix;
    rgb_intrinsics_matrix << 525, 0.0, 316.7, 0.0, 525, 238.5, 0.0, 0.0, 1.0; // Kinect RGB camera intrinsics

  // Read if some parameters are passed from command line:
  pcl::console::parse_argument (argc, argv, "--svm", svm_filename);
  pcl::console::parse_argument (argc, argv, "--conf", min_confidence);

  // Read Kinect live stream:
  // PointCloudT::Ptr cloud (new PointCloudT);
  // bool new_cloud_available_flag = false;
  // boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
  //         boost::bind(&cloud_cb_, _1, cloud, &new_cloud_available_flag);
}
