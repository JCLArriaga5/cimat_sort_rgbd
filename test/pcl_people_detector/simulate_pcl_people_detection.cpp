#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/people/ground_based_people_detection_app.h>
#include <pcl/common/time.h>

typedef pcl::PointXYZRGBA PointT;
typedef sensor_msgs::PointCloud2 PointCloudT;

// PCL viewer //
pcl::visualization::PCLVisualizer viewer ("PCL Viewer");

// Mutex: //
boost::mutex cloud_mutex;

// Ground plane init //
Eigen::VectorXf ground_coeffs;

// Callback cloud init
pcl::PointCloud<PointT> callback_cloud;
pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

// Create classifier for people detection:
pcl::people::PersonClassifier<pcl::RGB> person_classifier;

// People detection app initialization:
pcl::people::GroundBasedPeopleDetectionApp<PointT> people_detector;    // people detection object

// Algorithm parameters:
std::string svm_filename = "/home/ligthsaber/catkin_ws/src/cimat_sort_rgbd/utils/people/data/trainedLinearSVMForPeopleDetectionWithHOG.yaml";
float min_confidence = -1.5;
float voxel_size = 0.06;

ros::Publisher pub;

enum { COLS = 640, ROWS = 480 };

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
  data->viewerPtr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 0.2, "clicked_points");
  std::cout << current_point.x << " " << current_point.y << " " << current_point.z << std::endl;
}

void gp_estimation_clicked_points (const pcl::PointCloud<PointT> &callback_cloud)
{
  // Errors when estimating the ground plane with clicked points, reviewing it and correcting it.

  // Initialize Viewer
  viewer.initCameraParameters ();
  viewer.removeAllPointClouds();
  viewer.addCoordinateSystem(1.0);

  // Display pointcloud:
  cloud_mutex.lock ();    // for not overwriting the point cloud

  pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
  *cloud = callback_cloud;

  pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud);
  viewer.addPointCloud<PointT> (cloud, rgb);
  viewer.setCameraPosition(0,0,-2,0,-1,0,0);

  // Add point picking callback to viewer:
  struct callback_args cb_args;
  pcl::PointCloud<PointT>::Ptr clicked_points_3d (new pcl::PointCloud<PointT>);
  *clicked_points_3d = callback_cloud;

  cb_args.clicked_points_3d = clicked_points_3d;
  cb_args.viewerPtr = pcl::visualization::PCLVisualizer::Ptr(&viewer);
  viewer.registerPointPickingCallback (pp_callback, (void*)&cb_args);
  std::cout << "Shift+click on three floor points, then press 'Q'..." << std::endl;

  // Spin until 'Q' is pressed:
  viewer.spin();
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

void gp_estimation_floor (const pcl::PointCloud<PointT> &callback_cloud)
{
  // Initialize Viewer
  // viewer.initCameraParameters ();
  // viewer.removeAllPointClouds();
  // viewer.addCoordinateSystem(1.0);

  pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
  *cloud = callback_cloud;

  // pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud);
  // viewer.addPointCloud<PointT> (cloud, rgb);
  // viewer.setCameraPosition(0,0,-2,0,-1,0,0);

  // Floor segmentation
  ground_coeffs.resize(4);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::SACSegmentation<PointT> seg; // Create the segmentation object
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);
  seg.setInputCloud (cloud);
  seg.segment (*inliers, *coefficients);

  ground_coeffs[0] = coefficients->values[0];
  ground_coeffs[1] = coefficients->values[1];
  ground_coeffs[2] = coefficients->values[2];
  ground_coeffs[3] = coefficients->values[3];

  // std::cout << "Press 'Q' to finish the ground plane estimation stage..." << std::endl;
  //
  // // Spin until 'Q' is pressed:
  // viewer.spin();
  // std::cout << "done." << std::endl;
}

void detection (const PointCloudT::ConstPtr& input)
{
  Eigen::Matrix3f rgb_intrinsics_matrix;
    rgb_intrinsics_matrix << 525, 0.0, 316.7, 0.0, 525, 238.5, 0.0, 0.0, 1.0; // Kinect RGB camera intrinsics

  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  pcl::fromROSMsg (*input, callback_cloud);
  *cloud = callback_cloud;

  // setting alpha = 1.0 for rviz
  for (size_t i = 0; i < callback_cloud.points.size(); ++i){
    callback_cloud.points[i].a = 255;
  }

  // Proofs generate ground plane estimation with clicked_points (With errors - Check)
  // gp_estimation_clicked_points(callback_cloud);

  // Proofs generate ground plane whith floor segmentation
  gp_estimation_floor(callback_cloud);
  std::cout << "Ground plane: " << ground_coeffs(0) << " " << ground_coeffs(1) << " " << ground_coeffs(2) << " " << ground_coeffs(3) << std::endl;

                              // Detections //
  // Create classifier for people detection:
  person_classifier.loadSVMFromFile(svm_filename);   // load trained SVM

  people_detector.setVoxelSize(voxel_size);                        // set the voxel size
  people_detector.setIntrinsics(rgb_intrinsics_matrix);            // set RGB camera intrinsic parameters
  people_detector.setClassifier(person_classifier);                // set person classifier

  // Perform people detection on the new cloud:
  std::vector<pcl::people::PersonCluster<PointT> > clusters;   // vector containing persons clusters
  people_detector.setInputCloud(cloud);
  people_detector.setGround(ground_coeffs);                    // set floor coefficients
  people_detector.compute(clusters);                           // perform people detection

  ground_coeffs = people_detector.getGround();                 // get updated floor coefficients

  // Draw cloud and people bounding boxes in the viewer:
  viewer.removeAllPointClouds();
  viewer.removeAllShapes();
  pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud);
  viewer.addPointCloud<PointT> (cloud, rgb);
  unsigned int k = 0;
  for(std::vector<pcl::people::PersonCluster<PointT> >::iterator it = clusters.begin(); it != clusters.end(); ++it)
    {
    if(it->getPersonConfidence() > min_confidence)             // draw only people with confidence above a threshold
    {
      // draw theoretical person bounding box in the PCL viewer:
      it->drawTBoundingBox(viewer, k);
      k++;

      pcl::PointIndices clusters_indices = it->getIndices();
      std::vector<int> indices_ = clusters_indices.indices;
      for (unsigned int i = 0; i < indices_.size(); i++)
      {
          std::cout << indices_[i] << " People Bounding Box" << std::endl;
      }
    }
    }
  std::cout << k << " People found" << std::endl;
  viewer.spinOnce();
}

int main (int argc, char** argv)
{
  // Initialize Viewer
  viewer.initCameraParameters ();
  viewer.removeAllPointClouds();
  viewer.addCoordinateSystem(1.0);

  // Initialize ROS
  ros::init (argc, argv, "simulate_people_detection");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, detection);

  // Create a ROS publisher for the model coefficients
  pub = nh.advertise<pcl::PointCloud<PointT> > ("simulate_people_detection", 1);

  // Spin
  ros::spin ();
}
