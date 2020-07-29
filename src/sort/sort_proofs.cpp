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
#include <stdlib.h>
#include <time.h>
#include <glob.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <yaml-cpp/yaml.h>

#include <fstream>
#include <iomanip> // to format image names using setw() and setfill()
#include <unistd.h>    // to check file existence using POSIX function access(). On Linux include <unistd.h>.
#include <set>
#include <math.h>
#include <KalmanBoxTracker.h>
#include <Hungarian.h>

using namespace cv;
using namespace std;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

// PCL viewer //
pcl::visualization::PCLVisualizer viewer("PCL Viewer");

// Dataset path
string dataset_path = "../dataset/epfl_lab/20140804_160621_00";
string dataset_path_pcd = "../dataset/epfl_lab/pcd_files";

// Mutex: //
boost::mutex cloud_mutex;

// Global parameters
Eigen::VectorXf ground_coeffs;
cv::Mat rgb_image;
cv::Mat depth_image;
char buffer [9];
float min_confidence = -1.3;

enum { COLS = 512, ROWS = 424 };

typedef struct Tracking_X{
  std::vector<float> v_x_;
  int p_id;
}Tracking_X;

// Intersection over Union (IoU) for bounding circles
double iou(std::vector<float> data_test,  std::vector<float> data_gt)
{
  float xd = data_test[0] - data_gt[0];
  float yd = data_test[1] - data_gt[1];
  float zd = data_test[2] - data_gt[2];
  // Because for draw bounding cube the value of set dimension is (0.5 u).
  float r = sqrt(2) / 4;
  // for coordinate system, the axes that correspond to plane  2D is x-axe and z-axe.
  float d = sqrt(pow(xd, 2) + pow(zd, 2));

  // if the distance between centers is greater than or equal to the diameter
  // of the circle there is no intersection between the circles
  if (d >= (2 * r)){
    return double(0.0);
  }
  else{
    float a1 = (2 * r) * (sqrt(pow(r, 2) - (pow(d, 2) / 4))) / 4;
    float num = 2 * (sqrt(pow(r, 2) - (pow(d, 2) / 4)));
    float angle = atan(num / d);
    float a2 = (angle * pow(r, 2)) / 2;
    float a3 = a2 - a1;
    float a4 = (2 * r) * (sqrt(pow(r, 2) - (pow(d, 2) / 4))) / 2;
    float a5 = a4 + 2 * a3;
    float i = 2 * a5;
    float u = ((2 * 3.141592 * pow(r, 2)) - i);

    return (double)(i / u);
  }
}

// Generate the state vector u, v, w, h, gamma.
std::vector<float> x_state (std::vector<float> &TTop, double &Height){
  std::vector<float> state(5);

  state[0] = TTop[0]; // u
  state[1] = TTop[1]; // v
  state[2] = TTop[2]; // w
  state[3] = Height;  // h
  state[4] = sqrt(2) / 4; // fixed value for all cylinders - gamma

  return state;
}

// Function to draw the bounding cylinder
void drawTCylinderBox(pcl::visualization::PCLVisualizer& viewer, std::vector<float> &x,
  int person_number, std::vector<float> &_rgb_){

  pcl::ModelCoefficients coeffs_cylinder;
  // Plot Cylinder //
  // Cordinates of top center of detection cluster person
  coeffs_cylinder.values.push_back (x[0]);  // Cordinate x of top center
  coeffs_cylinder.values.push_back (x[1]);  // Cordinate y of top center
  coeffs_cylinder.values.push_back (x[2]);  // Cordinate z of top center
  coeffs_cylinder.values.push_back (0.0);
  coeffs_cylinder.values.push_back (x[3]);  // Height on y-axis
  coeffs_cylinder.values.push_back (0.0);
  coeffs_cylinder.values.push_back (x[4]);  // Cylinder radius - gamma

  std::stringstream bbox_name;
  bbox_name << "bbox_person_" << person_number;

  viewer.removeShape (bbox_name.str());
  viewer.addCylinder(coeffs_cylinder, bbox_name.str());
  viewer.setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, _rgb_[0], _rgb_[1], _rgb_[2], bbox_name.str());
  viewer.setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 1.3, bbox_name.str());
}

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
  // Parameters
  float fx = 365.8046307762528; // Values with intrinsics matrix
  float fy = 365.80463336811664; // Values with intrinsics matrix
  float cx = 254.31510758228475, cy = 206.98513348550657; // Values with intrinsics matrix

  // Conversion
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
          pt.x = (x - cx) * pt.z / fx;
          pt.y = (y - cy) * pt.z / fy;
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

int main (int argc, char** argv)
{
  // sort parameters
  int frame_count = 0;
  int max_age = 1;
  int min_hints = 3;
  double iou_threshold = 0.5;
  std::vector<KalmanBoxTracker> trackers;
  KalmanBoxTracker::_count_ = 0;

  // variables for SORT
  std::vector<std::vector<Tracking_X> > dets;
  std::vector<Tracking_X> vx_tmp;
	std::vector<std::vector<float> > predicted_dets;
	std::vector<std::vector<double> > iou_mx;
	std::vector<int> assignment;
	std::set<int> unmatchedDetections;
	std::set<int> unmatchedTrajectories;
  std::set<int> allItems;
	std::set<int> matchedItems;
  std::vector<cv::Point> matchedPairs;
  std::vector<Tracking_X> Tracking_result;
	unsigned int trk_num = 0;
	unsigned int det_num = 0;

  // string seq = "epfl_lab";
  ofstream xstate_file;
	string xsf = "../src/sort-rgbd/output/epfl_lab.txt";
	xstate_file.open(xsf.c_str());

  // Algorithm parameters:
  std::string svm_filename = "../people/data/trainedLinearSVMForPeopleDetectionWithHOG.yaml";
  float voxel_size = 0.06;
  float min_height = 1.0;
  float max_height = 1.87; //1.87
  // Format of intrinsics matrix
  // K = [fx 0 cx;
  //      0 fy cy; intrinsics matrix
  //      0  0  1]
  Eigen::Matrix3f rgb_intrinsics_matrix;
    rgb_intrinsics_matrix << 365.8046307762528, 0.0, 254.31510758228475, 0.0, 365.80463336811664, 206.98513348550657, 0.0, 0.0, 1.0; // Camera matrix of dataset epfl_lab

  int frame = 0;
  sprintf(buffer, "%06d.png", frame);
  string file_name = buffer;
  rgb_image   = cv::imread(dataset_path + "/rgb" + file_name, CV_LOAD_IMAGE_COLOR);
  depth_image = cv::imread(dataset_path + "/depth" + file_name, CV_LOAD_IMAGE_UNCHANGED);

  PointCloudT::Ptr cloud_init (new PointCloudT);
  cloud_init = depth2cloud(rgb_image, depth_image);
  // pcl::io::loadPCDFile(dataset_path_pcd + "/pcd" + file_name, *cloud_init);

  // Display pointcloud:
  pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud_init);
  viewer.addPointCloud<PointT> (cloud_init, rgb);
  viewer.setCameraPosition(0,0,-2,0,-1,0,0);
  // viewer.addCoordinateSystem(1.0);

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
  unsigned int n_dets = 0;
  std::vector<double> range_colors(300);
  for (int colors = 0; colors < 300; colors++){
    int clrs = rand()%255;
    range_colors[colors] = double(clrs / 255.0);
  }

  // Main loop:
  while (true)
  {
    sprintf(buffer, "%06d.png", frame);
    string file_name = buffer;
    rgb_image   = cv::imread(dataset_path + "/rgb" + file_name, CV_LOAD_IMAGE_COLOR);
    depth_image = cv::imread(dataset_path + "/depth" + file_name, CV_LOAD_IMAGE_UNCHANGED);

    PointCloudT::Ptr cloud (new PointCloudT);
    cloud = depth2cloud(rgb_image, depth_image);
    // pcl::io::loadPCDFile(dataset_path_pcd + "/pcd" + file_name, *cloud);

    cloud_mutex.lock ();    // for not overwriting the point cloud
    // Perform people detection on the new cloud:
    std::vector<pcl::people::PersonCluster<PointT> > clusters;   // vector containing persons clusters
    people_detector.setInputCloud(cloud);
    people_detector.setGround(ground_coeffs);                    // set floor coefficients
    people_detector.compute(clusters);                           // perform people detection

    ground_coeffs = people_detector.getGround();                 // get updated floor coefficients

    // // Display pointcloud:
    viewer.removeAllPointClouds();
    // viewer.removeAllShapes();
    pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud);
    viewer.addPointCloud<PointT> (cloud, rgb);
    viewer.setCameraPosition(0,0,-2,0,-1,0,0);

    unsigned int k = 0;
    vx_tmp.clear();
    frame_count++;

    // Get detections
    for(std::vector<pcl::people::PersonCluster<PointT> >::iterator it = clusters.begin(); it != clusters.end(); ++it)
    {
      // Get x_state, for detections that meet the pcl threshold
      if(it->getPersonConfidence() > min_confidence)
      {
        Eigen::VectorXf tmp = it->getTTop();
        double Height = it->getHeight();
        std::vector<float> TTop(3);
        TTop[0] = tmp[0];
        TTop[1] = tmp[1];
        TTop[2] = tmp[2];

        Tracking_X x_tmp;
        std::vector<float> x = x_state(TTop, Height);

        x_tmp.v_x_ = x;
        vx_tmp.push_back(x_tmp);
        dets.push_back(vx_tmp);

        k++;
      }
    }

    std::cout << k << " People found with pcl detector" << std::endl;
    viewer.spinOnce();

    if (trackers.size() == 0 && k != 0) // the first frame met
		{
			// initialize kalman trackers using first detections.
			for (unsigned int i = 0; i < k; i++)
			{
        std::cout << "flag 1 - initialize Kalman" << '\n';
        KalmanBoxTracker trk(dets[dets.size() - 1][i].v_x_);
        trackers.push_back(trk);
			}
		}

    // Get predicted locations from existing trackers.
    predicted_dets.clear();
    for (std::vector<KalmanBoxTracker>::iterator pred = trackers.begin(); pred != trackers.end();)
		{
			std::vector<float> pX = pred->predict();
      if (pX[0] != 0 && pX[1] != 0 && pX[2] != 0){
        std::cout << "flag 2 - predicted" << '\n';
        predicted_dets.push_back(pX);
        pred++;
      }
      else{
        std::cout << "flag 3 - erase predictions" << '\n';
        pred = trackers.erase(pred);
      }
		}

    // Associate detections to tracked object
    trk_num = predicted_dets.size();
		det_num = k;
    iou_mx.clear();
		iou_mx.resize(trk_num, std::vector<double>(det_num, 0));
    // compute iou matrix as a distance matrix
    for (unsigned int i = 0; i < trk_num; i++)
		{
			for (unsigned int j = 0; j < det_num; j++)
			{
        std::cout << "flag 4 - computed IoU" << '\n';
				// Use (1 - iou) because the hungarian algorithm computes a minimum-cost assignment.
				iou_mx[i][j] = 1 - iou(predicted_dets[i], dets[dets.size() - 1][j].v_x_);
        // std::cout << "minimum-cost: " << iou_mx[i][j] << std::endl;
			}
		}

    // // Solve the assignment problem using hungarian algorithm.
    HungarianAlgorithm linear_assignment;
		assignment.clear();
    if  (iou_mx.size() != 0){
      std::cout << "flag 5 - Hungarian" << '\n';
      linear_assignment.Solve(iou_mx, assignment);
    }

    // Find matches, unmatched_detections and unmatched_predictions
    unmatchedTrajectories.clear();
		unmatchedDetections.clear();
		allItems.clear();
		matchedItems.clear();

    if (det_num > trk_num) //	There are unmatched detections
		{
			for (unsigned int n = 0; n < det_num; n++)
				allItems.insert(n);

			for (unsigned int i = 0; i < trk_num; ++i)
				matchedItems.insert(assignment[i]);

			set_difference(allItems.begin(), allItems.end(),
				matchedItems.begin(), matchedItems.end(),
				insert_iterator<std::set<int> >(unmatchedDetections, unmatchedDetections.begin()));
		}
		else if (det_num < trk_num) // there are unmatched trajectory/predictions
		{
			for (unsigned int i = 0; i < trk_num; ++i)
				if (assignment[i] == -1) // unassigned label will be set as -1 in the assignment algorithm
					unmatchedTrajectories.insert(i);
		}

    // Filter out matched with low IoU
    matchedPairs.clear();
    for (unsigned int i = 0; i < trk_num; ++i)
		{
			if (assignment[i] == -1) // pass over invalid values
				continue;
			if (1 - iou_mx[i][assignment[i]] < iou_threshold)
			{
				unmatchedTrajectories.insert(i);
				unmatchedDetections.insert(assignment[i]);
			}
			else{
        matchedPairs.push_back(cv::Point(i, assignment[i]));
      }
		}

    // Updating trackers
    int detIdx, trkIdx;
		for (unsigned int i = 0; i < matchedPairs.size(); i++)
		{
			trkIdx = matchedPairs[i].x;
			detIdx = matchedPairs[i].y;
			trackers[trkIdx].update(dets[dets.size() - 1][detIdx].v_x_);
		}

    // Initialise new trackers for unmatched detections
    for (unsigned int umd = 0; umd < unmatchedDetections.size(); umd++)
    {
      KalmanBoxTracker trks(dets[dets.size() - 1][umd].v_x_);
      trackers.push_back(trks);
    }

    // Get trackers result
    Tracking_result.clear();
    for (std::vector<KalmanBoxTracker>::iterator trk_res = trackers.begin(); trk_res != trackers.end();)
    {
      if ((trk_res->time_since_update < 1) && (trk_res->hit_streak >= min_hints || frame_count <= min_hints))
      {
        std::cout << "flag 6 - Results" << '\n';
        Tracking_X x_get;
        x_get.v_x_ = trk_res->get_state();
        x_get.p_id = trk_res->id + 1;
        Tracking_result.push_back(x_get);

        trk_res++;
      }
      else
      {
        trk_res++;
      }
      // remove dead tracklet
			if (trk_res != trackers.end() && trk_res->time_since_update > max_age){
        trk_res = trackers.erase(trk_res);
        viewer.removeAllShapes();
      }
    }

    // Visualization
    // viewer.removeAllShapes();
    std::cout << "Tracking_result: " << Tracking_result.size() << std::endl;
    for (unsigned int id_vis = 0; id_vis < Tracking_result.size(); id_vis++)
    {
      // std::cout << Tracking_result[id_vis].p_id << '\n';
      std::vector<float> rgb_(3);

      rgb_[0] = range_colors[Tracking_result[id_vis].p_id + 5];
      rgb_[1] = range_colors[Tracking_result[id_vis].p_id + 10];
      rgb_[2] = range_colors[Tracking_result[id_vis].p_id + 15];

      drawTCylinderBox(viewer, Tracking_result[id_vis].v_x_, Tracking_result[id_vis].p_id, rgb_);
      // viewer.saveScreenshot("../images/3D-sort/3D-sort" + file_name);
    }

    // The sequence has 948 frames.
    if (frame >= 948){
      break;
    }

    // Increase frame for visualize point cloud of dataset
    std::cout << "Frame: " << frame << std::endl;
    ++frame;

    cloud_mutex.unlock ();
  }
  return 0;
}
