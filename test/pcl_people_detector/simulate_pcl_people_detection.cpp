#include <Eigen/Geometry>
#include <vector>
#include <set>

#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>

// PCL specific includes
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
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>

using namespace std;
typedef pcl::PointXYZRGBA PointT;
typedef sensor_msgs::PointCloud2 PointCloudT;

// PCL viewer //
pcl::visualization::PCLVisualizer viewer ("PCL Viewer");

// Mutex: //
boost::mutex cloud_mutex;

// Global Parameters
Eigen::VectorXf ground_coeffs; // Ground plane init
pcl::PointCloud<PointT> callback_cloud;
pcl::PointCloud<PointT>::Ptr cloud;
pcl::people::PersonClassifier<pcl::RGB> person_classifier; // Create classifier for people detection:
pcl::people::GroundBasedPeopleDetectionApp<PointT> people_detector;    // people detection object
string g_detectionFrame, g_baseLinkFrame, g_imageSourceRGB, g_imageSourceDepth, g_modalityName;
Eigen::Matrix3d g_intrinsicsMatrix = Eigen::Matrix3d::Zero();
boost::mutex g_intrinsicsMutex;

// ROS Components
boost::shared_ptr<ros::NodeHandle> g_paramNodeHandle;
boost::shared_ptr<tf::TransformListener> g_transformListener;
boost::shared_ptr<tf::TransformBroadcaster> g_transformBroadcaster;
ros::Publisher pub;

// Funtion to transform the Kinect coordinate system.
Eigen::Affine3d getCameraTransform(string frameId) {
    // Get the current position of the camera with regard to base_link
    tf::StampedTransform opticalFrameToBaseLinkTransform;
    g_transformListener->lookupTransform(g_baseLinkFrame, frameId, ros::Time(0), opticalFrameToBaseLinkTransform);

    Eigen::Affine3d opticalFrameToBaseLinkTransformEigen;
    tf::transformTFToEigen(opticalFrameToBaseLinkTransform, opticalFrameToBaseLinkTransformEigen);

    return opticalFrameToBaseLinkTransformEigen;
}

Eigen::Hyperplane<float, 3> getGroundPlane()
{
    // Get ground plane estimate [normal_x normal_y normal_z d] from parameters
    std::vector<float> original_ground_coeffs;
    string original_ground_coeffs_string;
    g_paramNodeHandle->getParam("ground_coeffs", original_ground_coeffs_string); // FIXME: Replace by dynparam
    {
        std::istringstream is(original_ground_coeffs_string);
        double value;
        while( is >> value ) {
            original_ground_coeffs.push_back(value);
        }
    }

    if (original_ground_coeffs.size() < 4) {
        ROS_WARN("Ground plane parameters not set, using defaults!");
        original_ground_coeffs.clear();
        original_ground_coeffs.push_back(0);
        original_ground_coeffs.push_back(0);
        original_ground_coeffs.push_back(1);
        original_ground_coeffs.push_back(0);
    }

    return Eigen::Hyperplane<float, 3>(
            Eigen::Vector3f(original_ground_coeffs[0], original_ground_coeffs[1], original_ground_coeffs[2]),
            original_ground_coeffs[3]);
}

Eigen::Quaterniond getGroundPlaneRotation(Eigen::Hyperplane<float, 3> groundPlane) {
    Eigen::Quaternion<double> groundPlaneRotation;
    groundPlaneRotation.setFromTwoVectors(Eigen::Vector3d(0,1,0), groundPlane.normal().cast<double>());
    return groundPlaneRotation;
}

Eigen::Affine3d getGroundPlaneTransform(Eigen::Hyperplane<float, 3> groundPlane) {
    return Eigen::Translation3d(groundPlane.normal().cast<double>() * groundPlane.offset())
            * getGroundPlaneRotation(groundPlane);
}

void cameraInfoCallback(const sensor_msgs::CameraInfo::Ptr& msg) {
    boost::array<double, 9> intrinsics = msg->K;
    Eigen::Matrix3d intrinsicsMatrix;
    for(int row = 0; row < 3; row++) for(int col = 0; col < 3; col++) intrinsicsMatrix(row, col) = intrinsics[row * 3 + col];

    boost::lock_guard<boost::mutex> lock(g_intrinsicsMutex);
    if(intrinsicsMatrix !=  g_intrinsicsMatrix) {
        g_intrinsicsMatrix = intrinsicsMatrix;
        std::cout << "Generate rgb intrinsics matrix done" << '\n';
    }
}

// Parameters
float min_confidence = -1.6;
float voxel_size = 0.06;

void gp_estimation_floor (const pcl::PointCloud<PointT>::Ptr &cloud)
{
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
}

void detection (const PointCloudT::Ptr& msg)
{
  // Get camera transform
  Eigen::Affine3d cameraTransform;

  string cameraFrame = msg->header.frame_id;
  try {
      cameraTransform = getCameraTransform(cameraFrame);
  }
  catch (tf::TransformException& ex) {
      ROS_WARN_THROTTLE(5.0, "Transform from %s to %s unavailable: %s",
      cameraFrame.c_str(), g_baseLinkFrame.c_str(), ex.what());
      return;
  }

  // Set up transform from base_link into our own coordinate frame
  Eigen::Affine3d baseLinkToDetectionFrame = Eigen::Affine3d(
          Eigen::AngleAxisd(0.5*M_PI, Eigen::Vector3d::UnitX()) *
          Eigen::AngleAxisd(0.5*M_PI, Eigen::Vector3d::UnitZ())
      );

  // Publish inverse transform on TF
  tf::Transform detectionFrameToBaseLinkTF;
  tf::transformEigenToTF(baseLinkToDetectionFrame.inverse(), detectionFrameToBaseLinkTF);
  g_transformBroadcaster->sendTransform(tf::StampedTransform(detectionFrameToBaseLinkTF,
    msg->header.stamp, g_baseLinkFrame, g_detectionFrame));

  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  pcl::fromROSMsg (*msg, *cloud);

  // cameraTransform is from optical frame to base_link
  // baseLinkToDetectionFrame is from base_link into an y-pointing-down reference frame
  Eigen::Affine3d cameraToDetectionFrame = baseLinkToDetectionFrame * cameraTransform;

  // Rotate pointcloud so it is upright
  pcl::transformPointCloud(*cloud, *cloud, cameraToDetectionFrame);
  cloud->header.frame_id = g_detectionFrame; // Update frame, since we are applying a transform (important for visualization)

  // Proofs generate ground plane whith floor segmentation
  gp_estimation_floor(cloud);
  std::cout << "Ground plane: " << ground_coeffs(0) << " " << ground_coeffs(1) << " " << ground_coeffs(2) << " " << ground_coeffs(3) << std::endl;
  people_detector.setGround(ground_coeffs);                    // set floor coefficients

                              // Detections //
  people_detector.setVoxelSize(voxel_size);                        // set the voxel size
  // Set RGB camera intrinsics matrix from camera info topic
  {
    boost::lock_guard<boost::mutex> lock(g_intrinsicsMutex);
    if(g_intrinsicsMatrix == Eigen::Matrix3d::Zero()) {
      ROS_WARN_THROTTLE(5.0, "Intrinsics matrix not set, make sure that the camera_info topic is set correctly and being published!");
      return;
    }
    else {
      people_detector.setIntrinsics(g_intrinsicsMatrix.cast<float>());
    }
  }
  people_detector.setClassifier(person_classifier);                // set person classifier

  // Detections
  // Perform people detection on the new cloud:
  std::vector<pcl::people::PersonCluster<PointT> > clusters;   // vector containing persons clusters
  people_detector.setInputCloud(cloud);
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
          std::cout << indices_[i] << " People Bounding Box" << std::endl; // Proofs, no correctly obtain coordinates
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
  ros::init (argc, argv, "simulate_pcl_people_detection");
  ros::NodeHandle nh("");
  g_paramNodeHandle = boost::shared_ptr<ros::NodeHandle>(new ros::NodeHandle("~"));

  // Parameters
  string svm_filename = ros::package::getPath(ROS_PACKAGE_NAME) + "/utils/people/data/trainedLinearSVMForPeopleDetectionWithHOG.yaml";
  g_detectionFrame = "pcl_people_detector_frame";
  g_baseLinkFrame = "camera_link";
  string input_cloud_topic = "/camera/depth_registered/points";
  string camera_info_topic = "/camera/rgb/camera_info";

  g_paramNodeHandle->getParam("svm_file", svm_filename);
  g_paramNodeHandle->getParam("base_link_frame", g_baseLinkFrame);
  g_paramNodeHandle->getParam("detection_frame", g_detectionFrame);
  g_paramNodeHandle->getParam("input_topic", input_cloud_topic);
  g_paramNodeHandle->getParam("camera_info_topic", camera_info_topic);

  // Set up TF listener & broadcaster
  g_transformListener = boost::shared_ptr<tf::TransformListener>(new tf::TransformListener(nh, ros::Duration(1.0)));
  g_transformBroadcaster = boost::shared_ptr<tf::TransformBroadcaster>(new tf::TransformBroadcaster());

  person_classifier.loadSVMFromFile(svm_filename);   // load trained SVM

  cloud = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
  // Create a ROS subscriber for the input point clou
  ros::Subscriber cis = nh.subscribe(camera_info_topic, 1, &cameraInfoCallback);
  ros::Subscriber pcs = nh.subscribe (input_cloud_topic, 1, &detection);

  // Spin
  ros::spin ();
  return 0;
}
