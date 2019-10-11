#include <mfuse_cloud_ops.hpp>

namespace mfuse
{
//*****************************************************************************
//*
//*
//*
//******************************************************************************

CloudOps::CloudOps()
{
  collectCloudDataStats_ = false;
}

//*****************************************************************************
//*
//*
//*
//******************************************************************************

CloudOps::~CloudOps()
{
}

/*inline
void toROSMsg(const sensor_msgs::PointCloud2 &cloud, sensor_msgs::Image &image)
{
  pcl::PCLPointCloud2 pcl_cloud;
  pcl_conversions::toPCL(cloud, pcl_cloud);
  pcl::PCLImage pcl_image;
  pcl::toPCLPointCloud2(pcl_cloud, pcl_image);
  pcl_conversions::moveFromPCL(pcl_image, image);
}*/

// some dead ends that might be a good reference
//pcl::toPCLPointCloud2(pcl_in_, pcl_image);
//convert ROS cloud to ROS image
//pcl::toROSMsg (*msg, cloudInRosImage_); 
//cloudInImage_ = cv_bridge::toCvCopy(cloudInRosImage_, sensor_msgs::image_encodings::MONO16);

//*****************************************************************************
//*
//* Convert a LIVOX unorganized point cloud to a cv::Mat image
//*
//******************************************************************************

void CloudOps::toCvImage(const pcl::PointCloud<pcl::PointXYZI>& cloud, 
    cv::Mat& outImage, int width = 1000, int height = 1000, int scale = 10)
  {  
    // allocate image
    outImage.create(height,width,CV_32F);
    // set all values to 0
    outImage = 0;

    for (size_t y = 0; y < cloud.height; y++)
    {
      for (size_t x = 0; x < cloud.width; x++)
      {
        if(collectCloudDataStats_)
        {
          maxX = std::max(maxX,cloud[x].x);
          maxY = std::max(maxY,cloud[x].y);
          maxZ = std::max(maxZ,cloud[x].z);
          minX = std::min(minX,cloud[x].x);
          minY = std::min(minY,cloud[x].y);
          minZ = std::min(minZ,cloud[x].z);
        }

        //if(0 < cloud[x].x && 0 < cloud[x].y)
        {
          //X = depth forward, y = width left, z = height up
          outImage.at<float>((scale * cloud[x].y) + height/2, (scale * cloud[x].z) + width/2) = cloud[x].x;
        }
      }
    }

    outImage.convertTo(outImage,CV_8U);

    //logger_->warn("[xy]: {}, {}, {} : {}, {}, {}", maxX, maxY, maxZ, minX, minY, minZ);
  }

//*****************************************************************************
//*
//* Convert a LIVOX unorganized point cloud to a cv::Mat image
//*
//******************************************************************************

void CloudOps::fromROSMsg(const sensor_msgs::PointCloud2 &cloud, 
  pcl::PointCloud<pcl::PointXYZI>& cloudOut, cv::Mat& outImage, int width, int height, int scale)
  {
    // convert ROS cloud to pcl cloud
    pcl::fromROSMsg(cloud, cloudOut);

    // convert pcl cloud to cv image
    toCvImage(cloudOut, outImage, width, height, scale);
  }

} // namespace mfuse
