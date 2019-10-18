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

  void Colorize(bool active);

  void fromROSMsg(const sensor_msgs::PointCloud2& cloud, 
    pcl::PointCloud<pcl::PointXYZI>& cloudOut,
    cv::Mat& outImage, int width, int height, int scale);
  void toCvImage(const pcl::PointCloud<pcl::PointXYZI>& cloud, 
    cv::Mat& outImage, int width, int height, int scale);
  void toCvImage2(const pcl::PointCloud<pcl::PointXYZI>& cloud, 
    cv::Mat& outImage, int width, int height, int scale);

private:

  std::deque<pcl::PointCloud<pcl::PointXYZI>> cloudQueue_;
  pcl::PointCloud<pcl::PointXYZI> currentCloud_;
  colmap::ColMap* colmap_;
  int cloudQueueSizeMax_ = 10;
  int cloudQueueSizeCurrent_ = 0;

  bool collectCloudDataStats_ = false;
  bool autoScaleColorMap_ = true;
  bool colorize_ = false;
  cv::Mat currentProjectionImage_;

  float maxX = 0, maxY = 0, maxZ = 0, minX = 255, minY = 255, minZ = 255;
}; // class CameraAlign   
} // namespace mfuse
