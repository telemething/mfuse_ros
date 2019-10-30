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
#include <mfuse_fuse_ops.hpp>
#include <colmap.hpp>
#include <mfuse_logger.hpp>

namespace mfuse
{
class CameraAlign 
{
public:

  struct matchPointType { cv::Point2i begin; cv::Point2i end; };
  struct matchPointType2 { cv::Point2i cloud; cv::Point2i visible; cv::Point2i ir; };
  explicit CameraAlign(ros::NodeHandle nh);
  void setCloudProjectionDepthMin(int depth);
  void setCloudProjectionDepthMax(int depth);
  void setCloudProjectionDepthTrack(int depth);
  ~CameraAlign();
  void DoFuse();
  void DoAccept();
  void DoQuit();
  bool showLines = false;

private:

  enum doOpEnum {DoOpNothing, DoOpFuse, DoOpAccept, DoOpQuit};
  doOpEnum doOp_;

  int logloopTimeoutMilliseconds_ = 250;
  std::string rgbInImageShowName_ = "RGB In";
  std::string irInImageShowName_ = "IR In";
  std::string warpFileNameIrOnVis = "WarpIrOnVis.json";
  std::string warpFileNameIrOnCloud = "WarpIrOnCloud.json";
  std::string warpFileNameVisOnCloud = "WarpVisOnCloud.json";
  std::string fusedImageDisplayName_ = "Fused Image";
  std::string cloudProjectionDisplayName_ = "Cloud Projection";

  std::string imFusedIrOnVisDisplayName_ = "imFusedIrOnVis";
  std::string imFusedIrOnCloudDisplayName_ = "imFusedIrOnCloud";
  std::string imFusedVisOnCloudDisplayName_ = "imFusedVisOnCloud";

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

  int iThermalAlpha = 50, iColorAlpha = 50;
  int irToVisBlend_ = 50, irToCloudBlend_ = 50, visToCloudBlend_ = 50;

  int projectionImageWidth_ = 500, projectionImageHeight_ = 500, projectionImageScale_ = 10;
  int projectionImageDepthMin_ = 0, projectionImageDepthMax_ = 255, projectionImageDepthTrack_ = 127;

  // ROS subscribers 
  image_transport::Subscriber rgbSubscriber_;
  image_transport::Subscriber irSubscriber_;
  ros::Subscriber pcInSubscriber_;

  boost::shared_mutex mutexRgbCameraImage_;
  boost::shared_mutex mutexIrCameraImage_;
  boost::shared_mutex mutexCloudProjectionImage_;  
  boost::shared_mutex mutexFusedImage_;

	cv::Mat homographyIrOnVis_, homographyIrOnCloud_, homographyVisOnCloud_;
  bool haveHomographyIrOnVis_ = false, haveHomographyIrOnCloud_ = false, haveHomographyVisOnCloud_ = false;

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

  int readWarpFiles();

	void getSideBySideImage(const cv::Mat& im1, const cv::Mat& im2, 
    cv::Mat& imCombined, const std::string windowName);
  void showAlignWindow(const cv::Mat& im1, const cv::Mat& im2, 
		const cv::Mat& im3, cv::Mat& imCombined, const std::string windowName);

  void combineImages(const cv::Mat& im1, const cv::Mat& im2, 
		cv::Mat& imCombined);
  void combineImages(const cv::Mat& im1, const cv::Mat& im2, 
		const cv::Mat& im3, cv::Mat& imCombined);
  void drawMatchPoints(const cv::Mat& img, 
    std::vector<CameraAlign::matchPointType2>& points, 
		CameraAlign::matchPointType2 pointUnderway);
	cv::Mat calculateHomography(std::vector<matchPointType> matchPoints);
  bool calculateHomographyIrOnVis(std::vector<matchPointType2> matchPoints, cv::Mat& warpOut);
  bool calculateHomographyIrOnCloud(std::vector<matchPointType2> matchPoints, cv::Mat& warpOut);
  bool calculateHomographyVisOnCloud(std::vector<matchPointType2> matchPoints, cv::Mat& warpOut);
	void rectifyManually(cv::Mat& im1, cv::Mat& im2, cv::Mat& im3);

  void createFusedWindows();

  void showIrOnVisWindow(const cv::Mat& im1, const std::string windowName);
  void showIrOnCloudWindow(const cv::Mat& im1, const std::string windowName);
  void showVisOnCloudWindow(const cv::Mat& im1, const std::string windowName);

  bool irOnVisWindowCreated_ = false, irOnCloudWindowCreated_ = false, visOnCloudWindowCreated_ = false;

  int displayloop();

}; // class CameraAlign   
} // namespace mfuse



