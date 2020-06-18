#ifndef KALMAN_H
#define KALMAN_H 2

#include "opencv2/video/tracking.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <Eigen/Dense>

using namespace std;
using namespace cv;

class KalmanBoxTracker{
  public:
    KalmanBoxTracker()
    {
      init(std::vector<float>());
      time_since_update = 0;
      id = _count_;
      hits = 0;
      hit_streak = 0;
      age = 0;
    }

    KalmanBoxTracker(std::vector<float> x_vector_state)
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
      history.clear();
	  }

    std::vector<float> predict();

    void update(std::vector<float> x_u);

    std::vector<float>& get_state();

    static int _count_;
    int time_since_update;
  	int hits;
  	int hit_streak;
  	int age;
  	int id;

  private:
    void init(std::vector<float> x_vector_state);
    cv::KalmanFilter kf;
	  cv::Mat measurement;
    std::vector<float> history;
};

#endif
