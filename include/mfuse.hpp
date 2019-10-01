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
class CameraFuse 
{
private:

  std::string rgbInImageShowName_ = "RGB In";
  std::string irInImageShowName_ = "IR In";
  std::string warpFileName = "thewarp.json";
  std::string fusedImageDisplayName_ = "Fused Image";

  int warpType_ = cv::MOTION_HOMOGRAPHY;
  int iThermalAlpha = 50;
  int iColorAlpha = 50;
  int logloopTimeoutMilliseconds_ = 250;
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
  std::thread fusionThread_;
  std::thread displayThread_;

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

  cv::Mat warpMatrix, roiIncludeMask, roiExcludeMask, fusedImage_;
  std::vector<cv::Point2f> warpedBBox;

  bool readParameters();
  void CreateLogger(std::string logDirectory);
  int logloop();
  int fusionloop();
  int displayloop();

  void rgbCameraCallback(const sensor_msgs::ImageConstPtr& msg);
  void irCameraCallback(const sensor_msgs::ImageConstPtr& msg);

  int readWarpFile(const std::string filename, cv::Mat& warp);
  int DrawROI(cv::Mat image, std::vector<cv::Point2f> outline);
  std::vector<cv::Point2f> getTransposedBBox(const cv::Mat original, const cv::Mat warpMatrix);
  cv::Mat getMask(const cv::Mat original, const std::vector<cv::Point2f> area, const bool include) ;

public:

  explicit CameraFuse(ros::NodeHandle nh);
  ~CameraFuse();

  int init();

}; // class cameraFuse   
} // namespace mfuse


