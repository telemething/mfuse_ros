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
  showFusedImage_ = false;
  showCloudInStreams_ = true;
  collectCloudDataStats_ = false;

	ROS_INFO("[CameraFuse] Node startedy.");

	CreateLogger(logDirectory);
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
//
//
//
//*****************************************************************************

int CameraFuse::readWarpFile(const std::string filename, cv::Mat& warp)
{
	cv::FileStorage fs2(filename, cv::FileStorage::READ);
	fs2["warpMatrix"] >> warp;
	fs2.release();

	if (nullptr == warp.data)
		return 0;

	return 1;
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

  cloudViewerTimer_ = nodeHandle_.createTimer(ros::Duration(0.1), &CameraFuse::cloudViewerTimerCallback, this);

  // initialize or load the warp matrix
	if (warpType_ == cv::MOTION_HOMOGRAPHY)
		warpMatrix = cv::Mat::eye(3, 3, CV_32F);
	else
		warpMatrix = cv::Mat::eye(2, 3, CV_32F);

	gotWarp_ = readWarpFile(warpFileName, warpMatrix);


}

//*****************************************************************************
//
//
//
//*****************************************************************************

int CameraFuse::DrawROI(cv::Mat image, std::vector<cv::Point2f> outline)
{
	cv::line(image, outline[0], outline[1], cv::Scalar(110, 220, 0), 1, 8);
	cv::line(image, outline[1], outline[2], cv::Scalar(110, 220, 0), 1, 8);
	cv::line(image, outline[2], outline[3], cv::Scalar(110, 220, 0), 1, 8);
	cv::line(image, outline[3], outline[0], cv::Scalar(110, 220, 0), 1, 8);

	return 0;
}

	//*****************************************************************************
	//
	//
	//
	//*****************************************************************************

	std::vector<cv::Point2f> CameraFuse::getTransposedBBox(const cv::Mat original, const cv::Mat warpMatrix)
	{
		std::vector<cv::Point2f> vIn;
		std::vector<cv::Point2f> vOut;

		vIn.push_back(cv::Point2f(0, 0));
		vIn.push_back(cv::Point2f(0, static_cast<float>(original.rows)));
		vIn.push_back(cv::Point2f(static_cast<float>(original.cols), static_cast<float>(original.rows)));
		vIn.push_back(cv::Point2f(static_cast<float>(original.cols), 0));

		cv::perspectiveTransform(vIn, vOut, warpMatrix);

		return vOut;
	}

	//*****************************************************************************
	//
	//
	//
	//*****************************************************************************

	cv::Mat CameraFuse::getMask(const cv::Mat original, const std::vector<cv::Point2f> area, const bool include) 
	{
		std::vector<cv::Point> hull;

		for (std::vector<int>::size_type i = 0; i != area.size(); i++) {
			hull.push_back(cv::Point2i(static_cast<int>(area[i].x), static_cast<int>(area[i].y)));
		}

		// draw black (or white) image 
		cv::Mat roiMask(original.rows, original.cols, CV_8U, include ? cv::Scalar(0) : cv::Scalar(255));

		// fill mask area with white (or black)
		fillConvexPoly(roiMask, hull, include ? cv::Scalar(255) : cv::Scalar(0));

		return roiMask;
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
	
	cv::Mat imWarped, imColorized, roiIncludeVisibleImage, roiExcludeVisibleImage, irImage, rgbImage;


  while(true)
  {     
    try
    {
      // wait for a new IR or RGB image to appear
      imageReady_.wait();
      //rgbImageReady_.wait();
      //irImageReady_.wait();

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

      if(!gotMasks)
      {
        warpedBBox = getTransposedBBox(irImage, warpMatrix);
        roiIncludeMask = getMask(rgbImage, warpedBBox, true);
        bitwise_not(roiIncludeMask, roiExcludeMask);
        gotMasks = true;
      }

      if(!gotWarp_)
      {
      	cv::imshow(irInImageShowName_, irImage);
      	cv::imshow(rgbInImageShowName_, rgbImage);
      	cv::waitKey(3);
        continue;
      }

      thermalAlpha = iThermalAlpha / 100.0;
      colorAlpha = iColorAlpha / 100.0;

      //ROS_INFO("mainLoop a");
      // invert pixel intensity so that colorizer works in correct direction
      cv::bitwise_not(irImage, irImage);

      //ROS_INFO("mainLoop b");
      // colorize 
      cv::applyColorMap(irImage, imColorized, cv::ColormapTypes::COLORMAP_RAINBOW);

      //ROS_INFO("mainLoop c");
      // show the image
      if (showDebugImages_)
        cv::imshow("imColorized", imColorized);

      //ROS_INFO("mainLoop d");
      // merge the color and gray thermal images
      cv::addWeighted(irImage, 1 - colorAlpha, imColorized, colorAlpha, 0.0, irImage);

      //ROS_INFO("mainLoop e");
      // warp the thermal image to the perspective of the visible image
      cv::warpPerspective(irImage, imWarped, warpMatrix, rgbImage.size());

      ///ROS_INFO("mainLoop f");
      // show the image
      if (showDebugImages_)
        cv::imshow("imWarped", imWarped);

      ///ROS_INFO("mainLoop g");
      // mask out the visible image outside of thermal viewport
      rgbImage.copyTo(roiIncludeVisibleImage, roiIncludeMask);

      {
        //lock out_image_, dont modify out_image_ before here
        boost::shared_lock<boost::shared_mutex> lock(mutexFusedImage_);

        //ROS_INFO("mainLoop h");
        // merge the visible and thermal images in the thermal viewport
        cv::addWeighted(imWarped, 1 - thermalAlpha, roiIncludeVisibleImage, thermalAlpha, 0.0, fusedImage_);

        ///ROS_INFO("mainLoop i");
        // show the image
        if (showDebugImages_)
          cv::imshow("imFusedSmall", fusedImage_);

        //ROS_INFO("mainLoop j");
        // mask out the visible area inside the thermal viewport
        rgbImage.copyTo(roiExcludeVisibleImage, roiExcludeMask);

        //ROS_INFO("mainLoop k");
        // merge the visible and thermal images in the visible viewport
        cv::addWeighted(fusedImage_, 1, roiExcludeVisibleImage, 1, 0.0, fusedImage_);

        fusedImageReady_.post();

        ///ROS_INFO("mainLoop l");
        if (showDebugImages_)
          DrawROI(fusedImage_, warpedBBox);
      }

      // This area is for the visible = camera0 thermal = camera1 scenario

      /*bitwise_not(frame2, frame2);
      warpPerspective(frame1, imWarped, warpMatrix, frame2.size());

      applyColorMap(frame2, imColorized, COLORMAP_RAINBOW);
      addWeighted(frame2, 1 - colorAlpha, imColorized, colorAlpha, 0.0, frame2);

      addWeighted(imWarped, 1 - thermalAlpha, frame2, thermalAlpha, 0.0, imFused);*/

      // show the image
      //if(showFusedImage_)
      //  cv::imshow("fused", fusedImage_);

      if(showDebugImages_)
        cv::waitKey(3);

      //ROS_INFO("mainLoop show");
      //imshow("zebra", imFused);
      //ROS_INFO("mainLoop showed");
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
    cv::createTrackbar("Thermal / Vis", fusedImageDisplayName_, &iThermalAlpha, 100);
	  cv::createTrackbar("Thermal Color", fusedImageDisplayName_, &iColorAlpha, 100);
    
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

/*inline
void toROSMsg(const sensor_msgs::PointCloud2 &cloud, sensor_msgs::Image &image)
{
  pcl::PCLPointCloud2 pcl_cloud;
  pcl_conversions::toPCL(cloud, pcl_cloud);
  pcl::PCLImage pcl_image;
  pcl::toPCLPointCloud2(pcl_cloud, pcl_image);
  pcl_conversions::moveFromPCL(pcl_image, image);
}*/

//*****************************************************************************
//*
//* Convert a LIVOX unorganized point cloud to a cv::Mat image
//*
//******************************************************************************

void CameraFuse::toCvImage(const pcl::PointCloud<pcl::PointXYZI>& cloud, 
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
  
  VPointCloud pcl_in_;

  sensor_msgs::Image cloudInRosImage_; 
  cv_bridge::CvImagePtr cloudInImage_;
  pcl::PCLImage pcl_image;
  cv::Mat theMat;

  try 
  {
    // convert ROS cloud to pcl cloud
    pcl::fromROSMsg(*msg, pcl_in_);

    // convert pcl cloud to cv image
    toCvImage(pcl_in_, theMat, width, height, scale);

    cloudProjectionImage_ = theMat.clone();

    //logger_->warn("[xy]: {}, {}, {} : {}, {}, {}", maxX, maxY, maxZ, minX, minY, minZ);

    // some dead ends that might be a good reference
    //pcl::toPCLPointCloud2(pcl_in_, pcl_image);
    //convert ROS cloud to ROS image
    //pcl::toROSMsg (*msg, cloudInRosImage_); 
    //cloudInImage_ = cv_bridge::toCvCopy(cloudInRosImage_, sensor_msgs::image_encodings::MONO16);

    if(showCloudInStreams_)
    {
    	//cv::imshow(cloudProjectionDisplayName_, theMat);
    	//cv::waitKey(1);

      cloudViewer_->removeAllPointClouds(cloudViewPort_);
      cloudViewer_->addPointCloud<pcl::PointXYZI>(pcl_in_.makeShared(), "downsampled", cloudViewPort_);
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

//*****************************************************************************
//*
//*
//*
//******************************************************************************

int CameraFuse::logloop()
{
  while(true)
  {
    if(NULL != logger_)
      logger_->flush();

    std::this_thread::sleep_for(std::chrono::milliseconds(logloopTimeoutMilliseconds_));
  }
}

//*****************************************************************************
//*
//* decsription: Create two log sinks.
//*   console : warnings and above
//*   file : trace (everything) and above
//* info: https://github.com/gabime/spdlog
//* examples:
//*   logger.set_level(spdlog::level::debug);
//*   spdlog::info("Welcome to spdlog!");
//*   spdlog::error("Some error message with arg: {}", 1);
//*   spdlog::warn("Easy padding in numbers like {:08d}", 12);
//*   spdlog::critical("Support for int: {0:d};  hex: {0:x};  oct: {0:o}; bin: {0:b}", 42);
//*   spdlog::info("Support for floats {:03.2f}", 1.23456);
//*   spdlog::info("Positional args are {1} {0}..", "too", "supported");
//*   spdlog::info("{:<30}", "left aligned");
//* Change log pattern
//*   spdlog::set_pattern("[%H:%M:%S %z] [%n] [%^---%L---%$] [thread %t] %v");
//* Compile time log levels, define SPDLOG_ACTIVE_LEVEL to desired level
//*   SPDLOG_TRACE("Some trace message with param {}", {});
//*   SPDLOG_DEBUG("Some debug message");
//*
//******************************************************************************

void CameraFuse::CreateLogger(std::string logDirectory)
{
    spdlog::info("Creating logs");

    try 
    {
      time_t rawtime;
      struct tm * timeinfo;
      char buffer[80];
      spdlog::level::level_enum logLevel_ = spdlog::level::debug;
      std::string logLevelName = "---";

      // we need to create a log flush thread, because the one built in to spdlog
      // doesn't work
      log_thread = std::thread(&CameraFuse::logloop, this); 

      // create time string
      time (&rawtime);
      timeinfo = localtime(&rawtime);
      strftime(buffer,80,"%Y-%m-%d-%H-%M-%S",timeinfo);
      auto fileName = logDirectory + "MFuse_Log_" + std::string(buffer) + ".txt";

      // create console logger
      auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
      console_sink->set_level(spdlog::level::warn);
      console_sink->set_pattern("[%H:%M:%S][%t][%^%L%$] %v");
      
      // create file logger
      auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(fileName, true);
      file_sink->set_level(spdlog::level::trace);
      file_sink->set_pattern("[%H:%M:%S][%t][%^%L%$] %v");

      // create multi logger
      spdlog::sinks_init_list sl = {console_sink, file_sink};
      logger_ = std::make_shared<spdlog::logger>("MFuse", sl);

      switch(logLevel_)
      {
        case spdlog::level::off : logLevelName = "off"; break;
        case spdlog::level::trace : logLevelName = "trace"; break;
        case spdlog::level::debug : logLevelName = "debug"; break;
        case spdlog::level::info : logLevelName = "info"; break;
        case spdlog::level::warn : logLevelName = "warn"; break;
        case spdlog::level::err : logLevelName = "err"; break;
        case spdlog::level::critical : logLevelName = "critical"; break;
      }

      // set level low to show info on startup
      logger_->set_level(spdlog::level::trace);
      spdlog::info("log file created: {0}, level: {1}", fileName, logLevelName);

      // set level to normal going forward
      logger_->set_level(logLevel_);
      logger_->warn("this should appear in both console and file");
      logger_->info("this message should not appear in the console, only in the file");

      // these dont seem to work, but maybe someday they will
      logger_->flush_on(spdlog::level::err);
      spdlog::flush_on(spdlog::level::err);
    }
    catch (const spdlog::spdlog_ex &ex)
    {
      std::cout << "Log init failed: " << ex.what() << std::endl;
            
      // create console logger
      auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
      console_sink->set_level(spdlog::level::warn);
      console_sink->set_pattern("[%H:%M:%S][%t][%^%L%$] %v");

      spdlog::sinks_init_list sl = {console_sink};
      logger_ = std::make_shared<spdlog::logger>("mFuse", sl);
	}     
}

} // namespace mfuse