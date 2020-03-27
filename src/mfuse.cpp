//*****************************************************************************
//
// https://www.programcreek.com/python/example/89436/cv2.addWeighted
// https://www.learnopencv.com/homography-examples-using-opencv-python-c/
// https://stackoverflow.com/questions/17822585/copy-blend-images-of-different-sizes-using-opencv
//
//*****************************************************************************

#include <mfuse.hpp>



namespace mfuse
{

boost::interprocess::interprocess_semaphore imageReady_(0);
boost::interprocess::interprocess_semaphore fusedImageReady_(0);
boost::interprocess::interprocess_semaphore rgbImageReady_(0);
boost::interprocess::interprocess_semaphore irImageReady_(0);

//*****************************************************************************
//*
//*
//*
//******************************************************************************

CameraFuse::CameraFuse(ros::NodeHandle nh) :
      nodeHandle_(nh),
      imageTransport_(nodeHandle_)
{
	std::string logDirectory = "/Data/Shared/Logs/";
  warpFileName = "thewarp.json";
  warpType_ = cv::MOTION_HOMOGRAPHY;

	showCameraInStreams_ = false;
  showDebugImages_ = false;
  showFusedImage_ = true;
  showCloudInStreams_ = false;
  collectCloudDataStats_ = false;

	ROS_INFO("[CameraFuse] Node startedy.");

	logger_ = MfLogger_.CreateLogger(logDirectory, "MFuse");
	logger_->info("Node started");

	// Read parameters from config file.
	if (!readParameters()) 
	{
		ros::requestShutdown();
	}

	init();

	logger_->info("ok");
}

//*****************************************************************************
//*
//*
//*
//******************************************************************************

CameraFuse::~CameraFuse()
{
    //boost::unique_lock<boost::shared_mutex> lockNodeStatus(mutexNodeStatus_);
    //isNodeRunning_ = false;
}

//*****************************************************************************
//*
//*
//*
//******************************************************************************

bool CameraFuse::readParameters()
{
  // Load common parameters.
  //nodeHandle_.param("image_view/enable_opencv", viewImage_, true);
  //nodeHandle_.param("image_view/wait_key_delay", waitKeyDelay_, 3);
  //nodeHandle_.param("image_view/enable_console_output", enableConsoleOutput_, false);

  return true;
}

//*****************************************************************************
//*
//*
//*
//******************************************************************************

int CameraFuse::init()
{
	std::string rgbCameraTopicName;
	std::string irCameraTopicName;
	std::string pcInTopicName;

	int rgbCameraQueueSize; 
	int irCameraQueueSize; 
	int pcInQueueSize; 

	nodeHandle_.param("subscribers/rgb_camera_reading/topic", rgbCameraTopicName,
                    std::string("/gscam1/image_raw"));

	nodeHandle_.param("subscribers/rgb_camera_reading/queue_size", rgbCameraQueueSize, 5);  

	nodeHandle_.param("subscribers/ir_camera_reading/topic", irCameraTopicName,
                    std::string("/flir_boson/image_raw"));

	nodeHandle_.param("subscribers/ir_camera_reading/queue_size", irCameraQueueSize, 5);  

	nodeHandle_.param("subscribers/point_cloud_reading/topic", pcInTopicName,
                    std::string("/livox/lidar"));

	nodeHandle_.param("subscribers/point_cloud_reading/queue_size", pcInQueueSize, 5);  

  // create UI windows

  if(showCloudInStreams_)
  {
    cloudViewer_ = std::make_shared<pcl::visualization::PCLVisualizer>("cloudViewer");

    //cloudViewer_->createViewPort(0.0, 0.0, 0.5, 1.0, cloudViewPort_);
    cloudViewer_->setBackgroundColor(0, 0, 0, cloudViewPort_);

    //cloudViewer_->createViewPort(0.5, 0.0, 1.0, 1.0, downsampled_view);
    //cloudViewer_->setBackgroundColor(0, 0, 0, downsampled_view);

    cloudViewer_->addCoordinateSystem(1.0);
    cloudViewer_->initCameraParameters();
  }


  // create subscribers

	rgbSubscriber_ = imageTransport_.subscribe(rgbCameraTopicName, rgbCameraQueueSize,
                                               &CameraFuse::rgbCameraCallback, this);

	irSubscriber_ = imageTransport_.subscribe(irCameraTopicName, irCameraQueueSize,
                                               &CameraFuse::irCameraCallback, this);

  pcInSubscriber_ = nodeHandle_.subscribe( pcInTopicName, pcInQueueSize, &CameraFuse::pcInCallback, this);

  // create threads

  fusionThread_ = std::thread(&CameraFuse::fusionloop, this); 
  displayThread_ = std::thread(&CameraFuse::displayloop, this); 

  if(showCloudInStreams_)
    cloudViewerTimer_ = nodeHandle_.createTimer(ros::Duration(0.1), &CameraFuse::cloudViewerTimerCallback, this);

  // initialize or load the warp matrix
	if (warpType_ == cv::MOTION_HOMOGRAPHY)
		warpMatrix_ = cv::Mat::eye(3, 3, CV_32F);
	else
		warpMatrix_ = cv::Mat::eye(2, 3, CV_32F);

	gotWarp_ = FuseOps::readWarpFile(warpFileName, warpMatrix_);
}

//*****************************************************************************
//*
//*
//*
//******************************************************************************

int CameraFuse::fusionloop()
{
  double updateFrequency = 30;
	double thermalAlpha = .5;
	double colorAlpha = .5;
  bool gotMasks = false;
	ros::Rate sleepTime(updateFrequency);

  //TODO * Should we colorize here? Not sure.
  bool colorizeThermal = false;
	
	cv::Mat imWarped, imColorized, roiIncludeVisibleImage, roiExcludeVisibleImage, irImage, rgbImage;
  std::vector<cv::Point2f> warpedBBox;
  cv::Mat roiIncludeMask, roiExcludeMask;

  FuseOps fo(logger_);

  while(true)
  {     
    try
    {
      // wait for a new IR or RGB image to appear
      imageReady_.wait();

      if(!gotRgbImage_ | !gotIrImage_)
        continue;

      {
        // lock the images
        boost::shared_lock<boost::shared_mutex> lockRgb(mutexRgbCameraImage_);
        boost::shared_lock<boost::shared_mutex> lockIr(mutexIrCameraImage_);

        // make copies
        irImage = irImage_->image.clone();
        rgbImage = rgbImage_->image.clone();
      }

      if(!gotWarp_)
      {
      	cv::imshow(irInImageShowName_, irImage);
      	cv::imshow(rgbInImageShowName_, rgbImage);
      	cv::waitKey(3);
        continue;
      }

      fo.fuse(irImage, rgbImage, fusedImage_, 
        warpMatrix_, iThermalAlpha_, iColorAlpha_, colorizeThermal);

      fusedImageReady_.post();
    }
    catch(const std::exception& e)
    {
      ROS_ERROR("--- EXCEPTION --- CameraFuse::fusionloop: %s", e.what());
      logger_->error("- EXCEPTION --- CameraFuse::fusionloop: {}", e.what());
    }
    catch(...)
    {
      ROS_ERROR("--- EXCEPTION --- CameraFuse::fusionloop: -undefined-");
      logger_->error("--- EXCEPTION --- CameraFuse::fusionloop: -undefined-");
    }
  }

  return 0; 
}

//*****************************************************************************
//*
//*
//*
//******************************************************************************

int CameraFuse::displayloop()
{
  if(showCloudInStreams_)
  {
    cv::namedWindow(cloudProjectionDisplayName_, cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO | cv::WINDOW_GUI_EXPANDED);
  }

  if(showFusedImage_)
  {
    cv::namedWindow(fusedImageDisplayName_, cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO | cv::WINDOW_GUI_EXPANDED);
    cv::createTrackbar("Thermal / Vis", fusedImageDisplayName_, &iThermalAlpha_, 100);
	  cv::createTrackbar("Thermal Color", fusedImageDisplayName_, &iColorAlpha_, 100);
    
    // interesting potential
    // https://docs.opencv.org/2.4/modules/highgui/doc/qt_new_functions.html
    //cv::setOpenGlDrawCallback()
  }

  while(true)
  {     
    try
    {
      if(showCloudInStreams_)
      {
        cv::imshow(cloudProjectionDisplayName_, cloudProjectionImage_);
      }

      if(showFusedImage_)
      {
        // wait for a new fused image to appear
        fusedImageReady_.wait();

        // lock the images
        boost::shared_lock<boost::shared_mutex> lock(mutexFusedImage_);
       
        cv::imshow(fusedImageDisplayName_, fusedImage_); 
      }

      if(showCloudInStreams_ || showFusedImage_)
        cv::waitKey(1);
      else
        cv::waitKey(100);
    }
    catch(const std::exception& e)
    {
      ROS_ERROR("--- EXCEPTION --- CameraFuse::displayloop: %s", e.what());
      logger_->error("- EXCEPTION --- CameraFuse::displayloop: {}", e.what());
    }
    catch(...)
    {
      ROS_ERROR("--- EXCEPTION --- CameraFuse::displayloop: -undefined-");
      logger_->error("--- EXCEPTION --- CameraFuse::displayloop: -undefined-");
    }
  }

  return 0; 
}

//*****************************************************************************
//*
//*
//*
//******************************************************************************

void CameraFuse::cloudViewerTimerCallback(const ros::TimerEvent&)
{
    cloudViewer_->spinOnce();

    if (cloudViewer_->wasStopped())
    {
        //ros::shutdown();
    }
}

//*****************************************************************************
//*
//*
//*
//******************************************************************************

void CameraFuse::pcInCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  ROS_DEBUG("[CameraFuse] pc in received.");
  //logger_->info("[CameraFuse] pc in received");

  int width = 1000;
  int height = 1000;
  int scale = 10;
  
  pcl::PointCloud<pcl::PointXYZI> cloudOut;
  cv::Mat cloudProjectionImage;

  try 
  {
    // convert pcl cloud to cv image
    cloudOps_.fromROSMsg(*msg, cloudOut, cloudProjectionImage, width, height, scale);

    cloudProjectionImage_ = cloudProjectionImage.clone();

    if(showCloudInStreams_)
    {
      cloudViewer_->removeAllPointClouds(cloudViewPort_);
      cloudViewer_->addPointCloud<pcl::PointXYZI>(cloudOut.makeShared(), "downsampled", cloudViewPort_);
    }
  } 
  catch (cv_bridge::Exception& e) 
  {
    ROS_ERROR("-- EXCEPTION --- CameraFuse::pcInCallback: %s", e.what());
    logger_->error("- EXCEPTION --- CameraFuse::pcInCallback: {}", e.what());
    return;
  }
  catch(const std::exception& e)
  {
    ROS_ERROR("--- EXCEPTION --- CameraFuse::pcInCallback: %s", e.what());
    logger_->error("- EXCEPTION --- CameraFuse::pcInCallback: {}", e.what());
  }
  catch(...)
  {
    ROS_ERROR("--- EXCEPTION --- CameraFuse::pcInCallback: -undefined-");
    logger_->error("--- EXCEPTION --- CameraFuse::pcInCallback: -undefined-");
  }

  return;
}

//*****************************************************************************
//*
//*
//*
//******************************************************************************

void CameraFuse::rgbCameraCallback(const sensor_msgs::ImageConstPtr& msg)
{
  ROS_DEBUG("[CameraFuse] rgb image received.");
  //logger_->debug("image received.");

  try 
  {
    boost::shared_lock<boost::shared_mutex> lock(mutexRgbCameraImage_, boost::try_to_lock);

    // if we cant lock it then just drop this frame
    if(!lock)
      return;

    rgbImage_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    rgbImageHeader_ = msg->header;   

    if (rgbImage_) 
    {
      //rgbImageReady_.post();
      gotRgbImage_ = true;
      imageReady_.post();

      if(showCameraInStreams_)
	    {
      	cv::imshow(rgbInImageShowName_, rgbImage_->image);
      	cv::waitKey(3);
	    }

      //frameWidth_ = rgbImage_->image.size().width;
      //frameHeight_ = rgbImage_->image.size().height;
    }

  } 
  catch (cv_bridge::Exception& e) 
  {
    ROS_ERROR("-- EXCEPTION --- CameraFuse::rgbCameraCallback: %s", e.what());
    logger_->error("- EXCEPTION --- CameraFuse::rgbCameraCallback: {}", e.what());
    return;
  }
  catch(const std::exception& e)
  {
    ROS_ERROR("--- EXCEPTION --- CameraFuse::rgbCameraCallback: %s", e.what());
    logger_->error("- EXCEPTION --- CameraFuse::rgbCameraCallback: {}", e.what());
  }
  catch(...)
  {
    ROS_ERROR("--- EXCEPTION --- CameraFuse::rgbCameraCallback: -undefined-");
    logger_->error("--- EXCEPTION --- CameraFuse::rgbCameraCallback: -undefined-");
  }

  return;
}

//*****************************************************************************
//*
//*
//*
//******************************************************************************

void CameraFuse::irCameraCallback(const sensor_msgs::ImageConstPtr& msg)
{
  ROS_DEBUG("[CameraFuse] ir image received.");
  //logger_->debug("image received.");

  try 
  {
    boost::shared_lock<boost::shared_mutex> lock(mutexIrCameraImage_, boost::try_to_lock);

    // if we cant lock it then just drop this frame
    if(!lock)
      return;

    irImage_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    irImageHeader_ = msg->header;   

    if (irImage_) 
    {
      //irImageReady_.post();
      gotIrImage_ = true;
      imageReady_.post();

      if(showCameraInStreams_)
	    {
      	cv::imshow(irInImageShowName_, irImage_->image);
      	cv::waitKey(3);
	    }

      //frameWidth_ = irImage_->image.size().width;
      //frameHeight_ = irImage_->image.size().height;
    }

  } 
  catch (cv_bridge::Exception& e) 
  {
    ROS_ERROR("-- EXCEPTION --- CameraFuse::irCameraCallback: %s", e.what());
    logger_->error("- EXCEPTION --- CameraFuse::irCameraCallback: {}", e.what());
    return;
  }
  catch(const std::exception& e)
  {
    ROS_ERROR("--- EXCEPTION --- CameraFuse::irCameraCallback: %s", e.what());
    logger_->error("- EXCEPTION --- CameraFuse::irCameraCallback: {}", e.what());
  }
  catch(...)
  {
    ROS_ERROR("--- EXCEPTION --- CameraFuse::irCameraCallback: -undefined-");
    logger_->error("--- EXCEPTION --- CameraFuse::irCameraCallback: -undefined-");
  }

  return;
}
} // namespace mfuse