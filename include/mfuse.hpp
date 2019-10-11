#pragma once

// ROS
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <iostream>
#include <sensor_msgs/PointCloud2.h>

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

#include <pcl/point_types.h>
#include <pcl/common/projection_matrix.h>
#include <pcl/conversions.h>
//#include <pcl_conversions.h>
//#include <pcl/pcl_conversions.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>

#include <mfuse_cloud_ops.hpp>

// http://wiki.ros.org/pcl_conversions

namespace mfuse
{
class CameraFuse 
{
private:

  std::string rgbInImageShowName_ = "RGB In";
  std::string irInImageShowName_ = "IR In";
  std::string warpFileName = "thewarp.json";
  std::string fusedImageDisplayName_ = "Fused Image";
  std::string cloudProjectionDisplayName_ = "Cloud Projection";

  int warpType_ = cv::MOTION_HOMOGRAPHY;
  int iThermalAlpha = 50;
  int iColorAlpha = 50;
  int logloopTimeoutMilliseconds_ = 250;
  bool showCameraInStreams_ = false;
  bool showCloudInStreams_ = false;
  bool showDebugImages_ = false;
  bool showFusedImage_ = false;
  bool gotWarp_ = false;
  bool gotRgbImage_ = false;
  bool gotIrImage_ = false;
  bool collectCloudDataStats_ = false;

  //typedef pcl::PointXYZI VPoint;
  //typedef pcl::PointCloud<VPoint> VPointCloud;
  //typedef pcl::PCLPointCloud2 VPointCloud;

  CloudOps cloudOps_;

  //float maxX = 0, maxY = 0, maxZ = 0, minX = 0, minY = 0, minZ = 0;

  ros::NodeHandle nodeHandle_;
  image_transport::ImageTransport imageTransport_;

  std::shared_ptr<spdlog::logger> logger_;
  std::thread log_thread; 
  std::thread fusionThread_;
  std::thread displayThread_;

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

  cv::Mat warpMatrix, roiIncludeMask, roiExcludeMask, fusedImage_;
  std::vector<cv::Point2f> warpedBBox;

  std::shared_ptr<pcl::visualization::PCLVisualizer> cloudViewer_;
  int cloudViewPort_ = 0;
  ros::Timer cloudViewerTimer_;

  bool readParameters();
  void CreateLogger(std::string logDirectory);
  int logloop();
  int fusionloop();
  int displayloop();

  void rgbCameraCallback(const sensor_msgs::ImageConstPtr& msg);
  void irCameraCallback(const sensor_msgs::ImageConstPtr& msg);
  void pcInCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
  void cloudViewerTimerCallback(const ros::TimerEvent&);

  int readWarpFile(const std::string filename, cv::Mat& warp);
  int DrawROI(cv::Mat image, std::vector<cv::Point2f> outline);
  std::vector<cv::Point2f> getTransposedBBox(const cv::Mat original, const cv::Mat warpMatrix);
  cv::Mat getMask(const cv::Mat original, const std::vector<cv::Point2f> area, const bool include) ;

  void toCvImage(const pcl::PointCloud<pcl::PointXYZI>& cloud, 
    cv::Mat& outImage, int width, int height, int scale);

public:

  explicit CameraFuse(ros::NodeHandle nh);
  ~CameraFuse();

  int init();

}; // class cameraFuse   
} // namespace mfuse



