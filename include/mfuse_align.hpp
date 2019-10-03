#pragma once

// ROS
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <iostream>

#include <boost/interprocess/sync/interprocess_semaphore.hpp>
#include <boost/format.hpp>
#include <boost/thread/locks.hpp>
#include <boost/thread/shared_mutex.hpp>
//#include <boost/interprocess/sync/sharable_lock.hpp>
//#include <boost/interprocess/sync/interprocess_sharable_mutex.hpp>

// SPD Logger
#include "spdlog/spdlog.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/sinks/basic_file_sink.h"

// OpenCV
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/opencv.hpp"
#include <opencv2/tracking.hpp>

namespace mfuse
{
class CameraAlign 
{
public:

  struct matchPointType { cv::Point2i begin; cv::Point2i end; };
  explicit CameraAlign(ros::NodeHandle nh);
  ~CameraAlign();

private:

  int logloopTimeoutMilliseconds_ = 250;
  std::string rgbInImageShowName_ = "RGB In";
  std::string irInImageShowName_ = "IR In";
  std::string warpFileName = "thewarp.json";
  std::string fusedImageDisplayName_ = "Fused Image";

  bool showCameraInStreams_ = false;
  bool showDebugImages_ = false;
  bool showFusedImage_ = false;
  bool gotWarp_ = false;
  bool gotRgbImage_ = false;
  bool gotIrImage_ = false;

  ros::NodeHandle nodeHandle_;
  image_transport::ImageTransport imageTransport_;

  std::shared_ptr<spdlog::logger> logger_;
  std::thread log_thread; 

  // ROS subscribers 
  image_transport::Subscriber rgbSubscriber_;
  image_transport::Subscriber irSubscriber_;
  boost::shared_mutex mutexRgbCameraImage_;
  boost::shared_mutex mutexIrCameraImage_;
  boost::shared_mutex mutexFusedImage_;

  cv_bridge::CvImagePtr rgbImage_;
  cv_bridge::CvImagePtr irImage_;
  std_msgs::Header rgbImageHeader_;
  std_msgs::Header irImageHeader_;

  int init();
  bool readParameters();
  void CreateLogger(std::string logDirectory);
  int logloop();

  void rgbCameraCallback(const sensor_msgs::ImageConstPtr& msg);
  void irCameraCallback(const sensor_msgs::ImageConstPtr& msg);

  int readWarp(const std::string filename, cv::Mat& warp);
	int saveWarp(const std::string fileName, const cv::Mat& warp);
	void getSideBySideImage(const cv::Mat& im1, const cv::Mat& im2, cv::Mat& imCombined, const std::string windowName);
	cv::Mat calculateHomography(std::vector<matchPointType> matchPoints);
	void rectifyManually(cv::Mat& im1, cv::Mat& im2);
}; // class CameraAlign   
} // namespace mfuse



