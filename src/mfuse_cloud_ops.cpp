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
  autoScaleColorMap_ = true;
  cloudQueueSizeMax_ = 50;

  Colorize(true);
}

//*****************************************************************************
//*
//*
//*
//******************************************************************************

CloudOps::~CloudOps()
{
}

//*****************************************************************************
//*
//*
//*
//******************************************************************************

void CloudOps::Colorize(bool active)
{
  if(active)
  {
    colmap_ = colmap::getColorMap(colmap::ColMap::ColormapTypes::COLORMAP_JET);
    colorize_ = true;
  }
    colorize_ = false;
}

//*****************************************************************************
//*
//*
//*
//******************************************************************************

void CloudOps::SetQueueSize(int queueuSize)
{
  cloudQueueSizeMax_ = queueuSize;
}

//*****************************************************************************
//*
//*
//*
//******************************************************************************

void CloudOps::SetDepthRange(int minDepth, int maxDepth)
{
  minDepth_ = minDepth;
  maxDepth_ = maxDepth;
}

//*****************************************************************************
//*
//*
//*
//******************************************************************************

void CloudOps::RunCurrentCloudCreationThread(int width, int height, 
  int scale, int sleepTime)
{
  currentCloudloopSleepTimeMs_ = sleepTime;
  projectionImageWidth_ = width;
  projectionImageScale_ = scale;
  projectionImageHeight_ = height;

  currentCloudloopThread_ = std::thread(&CloudOps::currentCloudloop, this); 
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
    bool colorizeLUT = true;
    float intensityScale = 1; 

    // allocate image
    outImage.create(height,width,CV_8UC3);
    // set all values to 0
    outImage = 0;

    if(autoScaleColorMap_)
    {
      if(0 < maxX)
        intensityScale = 255 / maxX;
    }

    cv::Vec3b theColor;
    theColor[0] = 255;
    theColor[1] = 125;
    theColor[2] = 125;

    for (size_t y = 0; y < cloud.height; y++)
    {
      for (size_t x = 0; x < cloud.width; x++)
      {
        if( minDepth_ > cloud[x].x || maxDepth_ < cloud[x].x )
          continue;

        if(collectCloudDataStats_)
        {
          maxX = std::max(maxX,cloud[x].x);
          maxY = std::max(maxY,cloud[x].y);
          maxZ = std::max(maxZ,cloud[x].z);
          if(0 < cloud[x].x)
            minX = std::min(minX,cloud[x].x);
          minY = std::min(minY,cloud[x].y);
          minZ = std::min(minZ,cloud[x].z);
        }

        if(autoScaleColorMap_)
        {
          maxX = std::max(maxX,cloud[x].x);

          if(0 < cloud[x].x)
            minX = std::min(minX,cloud[x].x);
        }

        if(colorizeLUT)
          theColor = colmap_->_lut.at<cv::Vec3b>(0,cloud[x].x * intensityScale);

        //if(0 < cloud[x].x && 0 < cloud[x].y)
        {
          auto pixel = &outImage.at<cv::Vec3b>((scale * -cloud[x].z) + width/2, 
            (scale * -cloud[x].y) + height/2);
          memcpy( pixel, &theColor, 3 * sizeof(uint8_t));
        }
      }
    }
  }

//*****************************************************************************
//*
//* Convert a LIVOX unorganized point cloud to a cv::Mat image. This differs from
//* toCvImage in that it creates a grayscale image and colorizes that instead
//* of colorizing the points from the cloud directly
//*
//******************************************************************************

void CloudOps::toCvImage2(const pcl::PointCloud<pcl::PointXYZI>& cloud, 
    cv::Mat& outImage, int width, int height, int scale)
  {  
    cv::Mat intensityImage;
    float intensityScale = 1;

    // allocate image
    intensityImage.create(height,width,CV_8U);
    // set all values to 0
    intensityImage = 0;

    if(autoScaleColorMap_)
    {
      if(0 < maxX)
        intensityScale = 255 / maxX;
    }

    for (size_t y = 0; y < cloud.height; y++)
    {
      for (size_t x = 0; x < cloud.width; x++)
      {
        if(collectCloudDataStats_)
        {
          maxX = std::max(maxX,cloud[x].x);
          maxY = std::max(maxY,cloud[x].y);
          maxZ = std::max(maxZ,cloud[x].z);
          if(0 < cloud[x].x)
            minX = std::min(minX,cloud[x].x);
          minY = std::min(minY,cloud[x].y);
          minZ = std::min(minZ,cloud[x].z);
        }

        if(autoScaleColorMap_)
        {
          maxX = std::max(maxX,cloud[x].x);

          if(0 < cloud[x].x)
            minX = std::min(minX,cloud[x].x);
        }

        //if(0 < cloud[x].x && 0 < cloud[x].y)
        {
          //X = depth forward, y = width left, z = height up
          intensityImage.at<uchar>((scale * -cloud[x].z) + width/2, 
            (scale * -cloud[x].y) + height/2) = cloud[x].x * intensityScale;
        }
      }
    }

    if(colorize_)
      cv::applyColorMap(intensityImage, outImage, cv::ColormapTypes::COLORMAP_RAINBOW);
    else
      intensityImage.convertTo(outImage,CV_8UC3);
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

//*****************************************************************************
//*
//* Add a LIVOX unorganized point cloud 
//*
//******************************************************************************

void CloudOps::add(const sensor_msgs::PointCloud2 &cloud)
  {
    pcl::PointCloud<pcl::PointXYZI> cloudOut;
    pcl::PointCloud<pcl::PointXYZI> cloudPopped;

    // convert ROS cloud to pcl cloud
    pcl::fromROSMsg(cloud, cloudOut);

    while(cloudQueueSizeCurrent_ > cloudQueueSizeMax_)
    {
      cloudPopped = cloudQueue_.front();
      cloudQueue_.pop_front();
      cloudQueueSizeCurrent_--;
    }

    cloudQueue_.push_back(cloudOut);
    cloudQueueSizeCurrent_++;

    //currentCloud_ += cloudOut;

  }

//*****************************************************************************
//*
//* Add a LIVOX unorganized point cloud 
//*
//******************************************************************************

void CloudOps::set(const sensor_msgs::PointCloud2 &cloud)
  {
    // set the current cloud
    pcl::fromROSMsg(cloud, currentCloud_);
  }

//*****************************************************************************
//*
//*  
//*
//******************************************************************************

pcl::PointCloud<pcl::PointXYZI> CloudOps::getCurrentCloud()
  {
    currentCloud_.clear();

    int counted = 0;

    for(auto index = cloudQueue_.begin(); index != cloudQueue_.end(); index++)
    {
      currentCloud_ += *index;
      if(counted++ > cloudQueueSizeMax_)
        break;
    }

    // return the current cloud
    return currentCloud_;
  }

//*****************************************************************************
//*
//*
//*
//******************************************************************************

int CloudOps::currentCloudloop()
{
  while(true)
  {
    getCurrentCloud();
    {
      boost::unique_lock<boost::shared_mutex> lockCloudProjection(projectionImageMutex_);
      toCvImage(currentCloud_, currentProjectionImage_, projectionImageWidth_, 
        projectionImageHeight_, projectionImageScale_);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(currentCloudloopSleepTimeMs_));
  }

  return 0;
}

//*****************************************************************************
//*
//*  
//*
//******************************************************************************

cv::Mat CloudOps::getCurrentProjectionImage(int width, int height, int scale)
  {
    // convert pcl cloud to cv image
    toCvImage(currentCloud_, currentProjectionImage_, width, height, scale);

    // return the current projection image
    return currentProjectionImage_;
  }

//*****************************************************************************
//*
//*  
//*
//******************************************************************************

cv::Mat CloudOps::getCurrentProjectionImage()
  {
    boost::shared_lock<boost::shared_mutex> lockCloudProjection(projectionImageMutex_);

    // return the current projection image
    return currentProjectionImage_;
  }

//*****************************************************************************
//
// @brief Conversion from a sensor_msgs::PointCLoud2 to octomap::Pointcloud, 
// used internally in OctoMap
// 
// @param cloud
// @param octomapCloud
//
//*****************************************************************************

  void pointCloud2ToOctomap(const sensor_msgs::PointCloud2& cloud, 
  octomap::Pointcloud& octomapCloud)
  {
    octomapCloud.reserve(cloud.data.size() / cloud.point_step);

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
    {
      // Check if the point is invalid
      if (!std::isnan (*iter_x) && !std::isnan (*iter_y) && !std::isnan (*iter_z))
        octomapCloud.push_back(*iter_x, *iter_y, *iter_z);
    }
  }

// https://github.com/ethz-asl/nbvplanner/issues/16

/*void thing()
{
  std::cout << "I am in OctomapWorld::insertPointcloudIntoMapImpl(), flag 1" << std::endl;
octomap::Pointcloud cloud_octo;
for (pcl::PointCloudpcl::PointXYZ::const_iterator it = cloud->begin();
it != cloud->end(); ++it) {
const octomap::point3d p_G_point(it->x, it->y, it->z);
cloud_octo.push_back( p_G_point );

//// First, check if we've already checked this.
//octomap::OcTreeKey key = octree_->coordToKey(p_G_point);
//if (occupied_cells.find(key) == occupied_cells.end()) {
//// Check if this is within the allowed sensor range.
//castRay(p_G_sensor, p_G_point, &free_cells, &occupied_cells);
//}

}
// Apply the new free cells and occupied cells from
//updateOccupancy(&free_cells, &occupied_cells);
std::cout << "I am in OctomapWorld::insertPointcloudIntoMapImpl(), flag 2" << std::endl;
octree_->insertPointCloud( cloud_octo, p_G_sensor);
octree_->updateInnerOccupancy();
}*/

} // namespace mfuse
