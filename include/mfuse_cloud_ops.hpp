#pragma once

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

namespace mfuse
{
class CloudOps 
{
public:

  typedef pcl::PointXYZI VPoint;
  typedef pcl::PointCloud<VPoint> VPointCloud;

  explicit CloudOps();
  ~CloudOps();

  void fromROSMsg(const sensor_msgs::PointCloud2& cloud, 
    pcl::PointCloud<pcl::PointXYZI>& cloudOut,
    cv::Mat& outImage, int width, int height, int scale);
  void toCvImage(const pcl::PointCloud<pcl::PointXYZI>& cloud, 
    cv::Mat& outImage, int width, int height, int scale);

private:

  bool collectCloudDataStats_ = false;

  float maxX = 0, maxY = 0, maxZ = 0, minX = 0, minY = 0, minZ = 0;
}; // class CameraAlign   
} // namespace mfuse
