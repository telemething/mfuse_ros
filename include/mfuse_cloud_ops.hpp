#pragma once

#include <queue>
#include <deque>

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
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <pcl/visualization/cloud_viewer.h>
#pragma GCC diagnostic pop

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <sensor_msgs/point_cloud2_iterator.h>
//#include <octomap_ros/conversions.h>
#include <colmap.hpp>

#include <iostream>
#include <chrono>
#include <thread>

namespace mfuse
{
class CloudOps 
{
public:

  typedef pcl::PointXYZI VPoint;
  typedef pcl::PointCloud<VPoint> VPointCloud;

  explicit CloudOps();
  ~CloudOps();

  void add(const sensor_msgs::PointCloud2 &cloud);
  void set(const sensor_msgs::PointCloud2 &cloud);

  pcl::PointCloud<pcl::PointXYZI> getCurrentCloud();
  cv::Mat getCurrentProjectionImage(int width, int height, int scale);
  cv::Mat getCurrentProjectionImage();

  void Colorize(bool active);
  void SetQueueSize(int queueuSize);

  void fromROSMsg(const sensor_msgs::PointCloud2& cloud, 
    pcl::PointCloud<pcl::PointXYZI>& cloudOut,
    cv::Mat& outImage, int width, int height, int scale);
  void toCvImage(const pcl::PointCloud<pcl::PointXYZI>& cloud, 
    cv::Mat& outImage, int width, int height, int scale);
  void toCvImage2(const pcl::PointCloud<pcl::PointXYZI>& cloud, 
    cv::Mat& outImage, int width, int height, int scale);
  void RunCurrentCloudCreationThread(int width, int height, int scale, int sleepTime);
  void SetDepthRange(int minDepth, int maxDepth);

  void SetCloudToIrWarp(cv::Mat cloudToIrWarp);
  void SetCloudToVisWarp(cv::Mat cloudToVisWarp);
  void SetLookUpIrImage(cv::Mat& LookUpIrImage );
  void SetLookUpVisImage(cv::Mat& LookUpVisImage );

private:

  boost::shared_mutex projectionImageMutex_;
  std::deque<pcl::PointCloud<pcl::PointXYZI>> cloudQueue_;
  pcl::PointCloud<pcl::PointXYZI> currentCloud_;
  colmap::ColMap* colmap_;
  int cloudQueueSizeMax_ = 10;
  int cloudQueueSizeCurrent_ = 0;
  int currentCloudloopSleepTimeMs_ = 100;

  int projectionImageWidth_, projectionImageHeight_, projectionImageScale_;
  int minDepth_ = 0, maxDepth_ = 1000;
  float distToProjPlane_ = 65;

  bool collectCloudDataStats_ = false;
  bool autoScaleColorMap_ = true;
  bool colorize_ = false;
  bool warpNow_ = false;
  cv::Mat currentProjectionImage_;
  std::thread currentCloudloopThread_; 

  float maxX = 0, maxY = 0, maxZ = 0, minX = 255, minY = 255, minZ = 255;

  cv::Mat cloudToIrWarp_, cloudToVisWarp_, lookUpIrImage_, lookUpVisImage_;

  int currentCloudloop();

}; // class CameraAlign   
} // namespace mfuse
