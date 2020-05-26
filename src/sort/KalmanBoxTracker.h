#ifndef KALMAN_H
#define KALMAN_H 2

#include "opencv2/video/tracking.hpp"
#include "opencv2/highgui/highgui.hpp"

using namespace std;
using namespace cv;

class KalmanBoxTracker{
  public:
    KalmanBoxTracker()
    {
      init(Eigen::VectorXf());
      time_since_update = 0;
      id = _count_;
      hits = 0;
      hit_streak = 0;
      age = 0;
    }

    KalmanBoxTracker(Eigen::VectorXf x_vector_state)
    {
      init(x_vector_state);
      time_since_update = 0;
      id = _count_;
      _count_++;
      hits = 0;
      hit_streak = 0;
      age = 0;
    }

    ~KalmanBoxTracker()
	  {
      history.resize(5);
      history.setZero();
	  }

    Eigen::VectorXf& predict();

    void update(Eigen::VectorXf x_vector_state);

    Eigen::VectorXf& get_state();

    static int _count_;
    int time_since_update;
  	int hits;
  	int hit_streak;
  	int age;
  	int id;
    
  private:
    void init(Eigen::VectorXf x_vector_state);
    cv::KalmanFilter kf;
	  cv::Mat measurement;
    Eigen::VectorXf history;
};

#endif
