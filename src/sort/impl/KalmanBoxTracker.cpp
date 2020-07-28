#include <KalmanBoxTracker.h>
#include <iostream>
#include <Eigen/Dense>

int KalmanBoxTracker::_count_ = 0;

void KalmanBoxTracker::init(std::vector<float> x_vector_state)
{
  int dim_x = 8;
  int dim_z = 5;

  kf = KalmanFilter(dim_x, dim_z, 0);
  measurement = Mat::zeros(dim_z, 1, CV_32F);
  kf.transitionMatrix = (Mat_<float>(dim_x, dim_x) <<
      1, 0, 0, 0, 0, 1, 0, 0,
      0, 1, 0, 0, 0, 0, 1, 0,
      0, 0, 1, 0, 0, 0, 0, 1,
      0, 0, 0, 1, 0, 0, 0, 0,
      0, 0, 0, 0, 1, 0, 0, 0,
      0, 0, 0, 0, 0, 1, 0, 0,
      0, 0, 0, 0, 0, 0, 1, 0,
      0, 0, 0, 0, 0, 0, 0, 1);
  // kf.measurementMatrix = *(Mat_<float>(dim_x, dim_z) <<
  //     1, 0, 0, 0, 0, 0, 0,
  //     0, 1, 0, 0, 0, 0, 0,
  //     0, 0, 1, 0, 0, 0, 0,
  //     0, 0, 0, 1, 0, 0, 0,
  //     0, 0, 0, 0, 1, 0, 0);
  setIdentity(kf.measurementMatrix);
  setIdentity(kf.processNoiseCov, Scalar::all(1e-2));
	setIdentity(kf.measurementNoiseCov, Scalar::all(1e-1));
	setIdentity(kf.errorCovPost, Scalar::all(1));

  // initialization of vector sate [u, v, w, h, gamma]
  kf.statePost.at<float>(0, 0) = x_vector_state[0]; // u
	kf.statePost.at<float>(1, 0) = x_vector_state[1]; // v
	kf.statePost.at<float>(2, 0) = x_vector_state[2]; // w
	kf.statePost.at<float>(3, 0) = x_vector_state[3]; // h
  kf.statePost.at<float>(4, 0) = x_vector_state[4]; // gamma
}

// Predict
std::vector<float> KalmanBoxTracker::predict()
{
	// predict
	Mat p = kf.predict();
	age += 1;

	if (time_since_update > 0)
          hit_streak = 0;
  time_since_update += 1;

	std::vector<float> predictdet(5);

  predictdet[0] = p.at<float>(0, 0);
  predictdet[1] = p.at<float>(1, 0);
  predictdet[2] = p.at<float>(2, 0);
  predictdet[3] = p.at<float>(3, 0);
  predictdet[4] = p.at<float>(4, 0);

	history.push_back(predictdet);
	return history.back();
}

// Update
void KalmanBoxTracker::update(std::vector<float> x_vector_state)
{
	time_since_update = 0;
	history.clear();
	hits += 1;
	hit_streak += 1;

	// Measurement
	measurement.at<float>(0, 0) = x_vector_state[0]; // u
	measurement.at<float>(1, 0) = x_vector_state[1]; // v
	measurement.at<float>(2, 0) = x_vector_state[2]; // w
	measurement.at<float>(3, 0) = x_vector_state[3]; // h
  measurement.at<float>(4, 0) = x_vector_state[4]; // gamma

	// update
	kf.correct(measurement);
}

// Return the current state vector
std::vector<float> KalmanBoxTracker::get_state()
{
	Mat s = kf.statePost;
  std::vector<float> state(5);

  state[0] = s.at<float>(0, 0);
  state[1] = s.at<float>(1, 0);
  state[2] = s.at<float>(2, 0);
  state[3] = s.at<float>(3, 0);
  state[4] = s.at<float>(4, 0);

	return state;
}
