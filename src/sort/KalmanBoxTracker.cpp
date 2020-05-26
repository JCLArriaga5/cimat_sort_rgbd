#include <KalmanBoxTracker.h>

int KalmanBoxTracker::_count_ = 0;

void KalmanBoxTracker::init(Eigen::VectorXf x_vector_state)
{
  int dim_x = 8;
  int dim_z = 5;
  x_vector_state.resize(dim_z);

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
  kf.measurementMatrix = (Mat_<float>(dim_x, dim_z) <<
      1, 0, 0, 0, 0, 0, 0,
      0, 1, 0, 0, 0, 0, 0,
      0, 0, 1, 0, 0, 0, 0,
      0, 0, 0, 1, 0, 0, 0,
      0, 0, 0, 0, 1, 0, 0);
  setIdentity(kf.processNoiseCov, Scalar::all(100));
	setIdentity(kf.measurementNoiseCov, Scalar::all(0.01));
	setIdentity(kf.errorCovPost, Scalar::all(1));

  // initialization of vector sate [u, v, w, h, gamma]
  kf.statePost.at<float>(0, 0) = x_vector_state[0]; // u
	kf.statePost.at<float>(1, 0) = x_vector_state[1]; // v
	kf.statePost.at<float>(2, 0) = x_vector_state[2]; // w
	kf.statePost.at<float>(3, 0) = x_vector_state[3]; // h
  kf.statePost.at<float>(4, 0) = x_vector_state[4]; // gamma
}

// Predict
Eigen::VectorXf&
 KalmanBoxTracker::predict()
{
	// predict
	Mat p = kf.predict();
	age += 1;

	if (time_since_update > 0)
          hit_streak = 0;
  time_since_update += 1;

	predictdet = p;

	history.push_back(predictdet);
	return history.back();
}

// Update
void KalmanBoxTracker::update(Eigen::VectorXf x_vector_state)
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
Eigen::VectorXf&
 KalmanBoxTracker::get_state()
{
	Mat s = kf.statePost;
  Eigen::VectorXf state;
  state.resize(5)

  state[0] = s.at<float>(0, 0);
  state[1] = s.at<float>(1, 0);
  state[2] = s.at<float>(2, 0);
  state[3] = s.at<float>(3, 0);
  state[4] = s.at<float>(4, 0);

	return  state;
}
