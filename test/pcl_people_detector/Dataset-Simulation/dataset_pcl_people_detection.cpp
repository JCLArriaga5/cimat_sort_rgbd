// PCL specific includes
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/people/ground_based_people_detection_app.h>
#include <pcl/common/time.h>

#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <yaml-cpp/yaml.h>

using namespace cv;
using namespace std;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

// PCL viewer //
pcl::visualization::PCLVisualizer viewer("PCL Viewer");

// Dataset path
string dataset_path = "../utils/dataset/epfl_lab/20140804_160621_00";

// Mutex: //
boost::mutex cloud_mutex;

// Global parameters
Eigen::VectorXf ground_coeffs;
cv::Mat rgb_image;
cv::Mat depth_image;
char buffer [9];
float min_confidence = -1.7;

enum { COLS = 512, ROWS = 424 };
// enum { COLS = 640, ROWS = 480 };

struct callback_args{
  // structure used to pass arguments to the callback function
  PointCloudT::Ptr clicked_points_3d;
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

PointCloudT::Ptr depth2cloud( cv::Mat rgb_image, cv::Mat depth_image )
{
  float f = 570.3;
  float cx = 256.0, cy = 212.0;

  PointCloudT::Ptr cloud_ptr( new PointCloudT () );
  cloud_ptr->width  = rgb_image.cols;
  cloud_ptr->height = rgb_image.rows;
  cloud_ptr->is_dense = false;

  for ( int y = 0; y < rgb_image.rows; ++ y ) {
    for ( int x = 0; x < rgb_image.cols; ++ x ) {
      pcl::PointXYZRGB pt;
      if ( depth_image.at<unsigned short>(y, x) != 0 )
      {
          pt.z = depth_image.at<unsigned short>(y, x) / 1000.0;
          pt.x = (x - cx) * pt.z / f;
          pt.y = (y - cy) * pt.z / f;
          pt.r = rgb_image.at<cv::Vec3b>(y, x)[2];
          pt.g = rgb_image.at<cv::Vec3b>(y, x)[1];
          pt.b = rgb_image.at<cv::Vec3b>(y, x)[0];
          cloud_ptr->points.push_back( pt );
      }
      else
      {
          pt.z = 0;
          pt.x = 0;
          pt.y = 0;
          pt.r = 0;
          pt.g = 0;
          pt.b = 0;
          cloud_ptr->points.push_back( pt );
      }
    }
  }
  return cloud_ptr;
}

void visualization(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector<pcl::people::PersonCluster<PointT> > clusters)
{
  // Display pointcloud:
  viewer.removeAllPointClouds();
  viewer.removeAllShapes();
  pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud);
  viewer.addPointCloud<PointT> (cloud, rgb);
  viewer.setCameraPosition(0,0,-2,0,-1,0,0);

  unsigned int k = 0;
  for(std::vector<pcl::people::PersonCluster<PointT> >::iterator it = clusters.begin(); it != clusters.end(); ++it)
  {
    if(it->getPersonConfidence() > min_confidence)             // draw only people with confidence above a threshold
    {
      // draw theoretical person bounding box in the PCL viewer:
      it->drawTBoundingBox(viewer, k);
      k++;
    }
  }
  std::cout << k << " People found" << std::endl;
  viewer.spinOnce();
}

int main (int argc, char** argv)
{
  // Algorithm parameters:
  std::string svm_filename = "../utils/people/data/trainedLinearSVMForPeopleDetectionWithHOG.yaml";
  float voxel_size = 0.06;
  float min_height = 0.6;
  float max_height = 1.8;
  Eigen::Matrix3f rgb_intrinsics_matrix;
    rgb_intrinsics_matrix << 365.8046307762528, 0.0, 254.31510758228475, 0.0, 365.80463336811664, 206.98513348550657, 0.0, 0.0, 1.0; // Camera matrix of dataset epfl_lab

  int frame = 0;
  sprintf(buffer, "%06d.png", frame);
  string file_name = buffer;
  rgb_image   = cv::imread(dataset_path + "/rgb" + file_name, CV_LOAD_IMAGE_COLOR);
  depth_image = cv::imread(dataset_path + "/depth" + file_name, CV_LOAD_IMAGE_UNCHANGED);

  PointCloudT::Ptr cloud_init (new PointCloudT);
  cloud_init = depth2cloud(rgb_image, depth_image);

  // Display pointcloud:
  pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud_init);
  viewer.addPointCloud<PointT> (cloud_init, rgb);
  viewer.setCameraPosition(0,0,-2,0,-1,0,0);

  // Add point picking callback to viewer:
  struct callback_args cb_args;
  PointCloudT::Ptr clicked_points_3d (new PointCloudT);
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
  std::cout << "Ground plane: " << ground_coeffs(0) << " " << ground_coeffs(1) << " " << ground_coeffs(2) << " " << ground_coeffs(3) << std::endl;

  // Create classifier for people detection:
  pcl::people::PersonClassifier<pcl::RGB> person_classifier;
  person_classifier.loadSVMFromFile(svm_filename);   // load trained SVM

  // People detection app initialization:
  pcl::people::GroundBasedPeopleDetectionApp<PointT> people_detector;    // people detection object
  people_detector.setVoxelSize(voxel_size);                        // set the voxel size
  people_detector.setIntrinsics(rgb_intrinsics_matrix);            // set RGB camera intrinsic parameters
  people_detector.setClassifier(person_classifier);                // set person classifier
  people_detector.setPersonClusterLimits(min_height, max_height, 0.1, 8.0);  // set person classifier

  // Main loop:
  while (true)
  {
    sprintf(buffer, "%06d.png", frame);
    string file_name = buffer;
    rgb_image   = cv::imread(dataset_path + "/rgb" + file_name, CV_LOAD_IMAGE_COLOR);
    depth_image = cv::imread(dataset_path + "/depth" + file_name, CV_LOAD_IMAGE_UNCHANGED);

    PointCloudT::Ptr cloud (new PointCloudT);
    cloud = depth2cloud(rgb_image, depth_image);

    cloud_mutex.lock ();    // for not overwriting the point cloud
    // Perform people detection on the new cloud:
    std::vector<pcl::people::PersonCluster<PointT> > clusters;   // vector containing persons clusters
    people_detector.setInputCloud(cloud);
    people_detector.setGround(ground_coeffs);                    // set floor coefficients
    people_detector.compute(clusters);                           // perform people detection

    ground_coeffs = people_detector.getGround();                 // get updated floor coefficients

    // visualization(cloud, clusters);
    // Display pointcloud:
    viewer.removeAllPointClouds();
    viewer.removeAllShapes();
    pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud);
    viewer.addPointCloud<PointT> (cloud, rgb);
    viewer.setCameraPosition(0,0,-2,0,-1,0,0);

    unsigned int k = 0;
    for(std::vector<pcl::people::PersonCluster<PointT> >::iterator it = clusters.begin(); it != clusters.end(); ++it)
    {
      if(it->getPersonConfidence() > min_confidence)             // draw only people with confidence above a threshold
      {
        // draw theoretical person bounding box in the PCL viewer:
        it->drawTBoundingBox(viewer, k);
        k++;
      }
    }
    std::cout << k << " People found" << std::endl;
    viewer.spinOnce();

    cloud_mutex.unlock ();

    if (frame >= 949){
      break;
    }

    ++frame;
  }
  return 0;
}
