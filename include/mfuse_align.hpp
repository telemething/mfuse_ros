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

#include <opencv2/core/core.hpp>
#include <pcl_ros/impl/transforms.hpp>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/PointCloud2.h>

#include <mfuse_cloud_ops.hpp>
#include <colmap.hpp>
//#include <mfuse_logger.hpp>

namespace mfuse
{
class CameraAlign 
{
public:

  struct matchPointType { cv::Point2i begin; cv::Point2i end; };
  struct matchPointType2 { cv::Point2i cloud; cv::Point2i visible; cv::Point2i ir; };
  explicit CameraAlign(ros::NodeHandle nh);
  ~CameraAlign();

private:

  int logloopTimeoutMilliseconds_ = 250;
  std::string rgbInImageShowName_ = "RGB In";
  std::string irInImageShowName_ = "IR In";
  std::string warpFileName = "thewarp.json";
  std::string fusedImageDisplayName_ = "Fused Image";
  std::string cloudProjectionDisplayName_ = "Cloud Projection";

  bool showCameraInStreams_ = false;
  bool showDebugImages_ = false;
  bool showFusedImage_ = false;
  bool gotWarp_ = false;
  bool gotRgbImage_ = false;
  bool gotIrImage_ = false;
  bool gotcloudProjectionImage_ = false;  
  bool alignWindowCreated_ = false;
  bool showCloudInStreams_ = false;

  CloudOps cloudOps_;
  std::vector<matchPointType2> matchPoints_;

  ros::NodeHandle nodeHandle_;
  image_transport::ImageTransport imageTransport_;

  std::shared_ptr<spdlog::logger> logger_;
  std::thread log_thread; 
  std::thread displayThread_;

  int iThermalAlpha = 50;
  int iColorAlpha = 50;

  // ROS subscribers 
  image_transport::Subscriber rgbSubscriber_;
  image_transport::Subscriber irSubscriber_;
  ros::Subscriber pcInSubscriber_;

  boost::shared_mutex mutexRgbCameraImage_;
  boost::shared_mutex mutexIrCameraImage_;
  boost::shared_mutex mutexFusedImage_;

  cv_bridge::CvImagePtr rgbImage_;
  cv_bridge::CvImagePtr irImage_;
  cv::Mat cloudProjectionImage_;
  std_msgs::Header rgbImageHeader_;
  std_msgs::Header irImageHeader_;

  std::shared_ptr<pcl::visualization::PCLVisualizer> cloudViewer_;
  int cloudViewPort_ = 0;
  ros::Timer cloudViewerTimer_;

  int init();
  bool readParameters();
  void CreateLogger(std::string logDirectory);
  int logloop();

  void cloudViewerTimerCallback(const ros::TimerEvent&);
  void rgbCameraCallback(const sensor_msgs::ImageConstPtr& msg);
  void irCameraCallback(const sensor_msgs::ImageConstPtr& msg);
  void pcInCallback(const sensor_msgs::PointCloud2ConstPtr& msg);

  int readWarp(const std::string filename, cv::Mat& warp);
	int saveWarp(const std::string fileName, const cv::Mat& warp);

	void getSideBySideImage(const cv::Mat& im1, const cv::Mat& im2, 
    cv::Mat& imCombined, const std::string windowName);
  void showAlignWindow(const cv::Mat& im1, const cv::Mat& im2, 
		const cv::Mat& im3, cv::Mat& imCombined, const std::string windowName);

  void combineImages(const cv::Mat& im1, const cv::Mat& im2, 
		cv::Mat& imCombined);
  void combineImages(const cv::Mat& im1, const cv::Mat& im2, 
		const cv::Mat& im3, cv::Mat& imCombined);
  void drawMatchPoints(const cv::Mat& img, std::vector<CameraAlign::matchPointType2>& points);
	cv::Mat calculateHomography(std::vector<matchPointType> matchPoints);
	void rectifyManually(cv::Mat& im1, cv::Mat& im2, cv::Mat& im3);
  int displayloop();

}; // class CameraAlign   
} // namespace mfuse



