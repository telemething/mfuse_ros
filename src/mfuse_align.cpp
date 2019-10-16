//*****************************************************************************
//
// https://www.programcreek.com/python/example/89436/cv2.addWeighted
// https://www.learnopencv.com/homography-examples-using-opencv-python-c/
// https://stackoverflow.com/questions/17822585/copy-blend-images-of-different-sizes-using-opencv
//
//*****************************************************************************

#include <mfuse_align.hpp>

namespace mfuse
{

boost::interprocess::interprocess_semaphore imageReady_(0);
boost::interprocess::interprocess_semaphore fusedImageReady_(0);
boost::interprocess::interprocess_semaphore rgbImageReady_(0);
boost::interprocess::interprocess_semaphore irImageReady_(0);

cv::Mat combinedImage;
CameraAlign::matchPointType matchPoint;
std::vector<CameraAlign::matchPointType> matchPoints;
std::string rectifyWindowsName = "rectify2";
int matchPointEndWidthOffset = 0;

//*****************************************************************************
//*
//*
//*
//******************************************************************************

CameraAlign::CameraAlign(ros::NodeHandle nh) :
      nodeHandle_(nh),
      imageTransport_(nodeHandle_)
{
	std::string logDirectory = "/Data/Shared/Logs/";
	showCameraInStreams_ = false;
	showCloudInStreams_ = true;

	ROS_INFO("[CameraAlign] Node started.");

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

CameraAlign::~CameraAlign()
{
    //boost::unique_lock<boost::shared_mutex> lockNodeStatus(mutexNodeStatus_);
    //isNodeRunning_ = false;
}

//*****************************************************************************
//*
//*
//*
//******************************************************************************

bool CameraAlign::readParameters()
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

int CameraAlign::init()
{
	std::string rgbCameraTopicName;
	std::string irCameraTopicName;
	std::string pcInTopicName;

	int rgbCameraQueueSize; 
	int irCameraQueueSize; 
	int pcInQueueSize; 

	nodeHandle_.param("subscribers/rgb_camera_reading/topic", rgbCameraTopicName,
                    std::string("/gscam1/image_raw"));

	nodeHandle_.param("subscribers/rgb_camera_reading/queue_size", rgbCameraQueueSize, 1);  

	nodeHandle_.param("subscribers/ir_camera_reading/topic", irCameraTopicName,
                    std::string("/flir_boson/image_raw"));

	nodeHandle_.param("subscribers/rir_camera_reading/queue_size", irCameraQueueSize, 1);  

	rgbSubscriber_ = imageTransport_.subscribe(rgbCameraTopicName, rgbCameraQueueSize,
                                               &CameraAlign::rgbCameraCallback, this);

	irSubscriber_ = imageTransport_.subscribe(irCameraTopicName, irCameraQueueSize,
                                               &CameraAlign::irCameraCallback, this);

	nodeHandle_.param("subscribers/point_cloud_reading/topic", pcInTopicName,
                    							std::string("/livox/lidar"));

	nodeHandle_.param("subscribers/point_cloud_reading/queue_size", pcInQueueSize, 5);  


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

  	pcInSubscriber_ = nodeHandle_.subscribe( pcInTopicName, pcInQueueSize, 
	  										 &CameraAlign::pcInCallback, this);


    //fusionThread_ = std::thread(&CameraFuse::fusionloop, this); 
    displayThread_ = std::thread(&CameraAlign::displayloop, this); 

	cloudViewerTimer_ = nodeHandle_.createTimer(ros::Duration(0.1), 
												&CameraAlign::cloudViewerTimerCallback, this);


    // initialize or load the warp matrix
	/*if (warpType_ == cv::MOTION_HOMOGRAPHY)
		warpMatrix = cv::Mat::eye(3, 3, CV_32F);
	else
		warpMatrix = cv::Mat::eye(2, 3, CV_32F);

	gotWarp_ = readWarpFile(warpFileName, warpMatrix);*/
}

	//*****************************************************************************
	//
	//
	//
	//*****************************************************************************

	int CameraAlign::readWarp(const std::string filename, cv::Mat& warp)
	{
		cv::FileStorage fs2(filename, cv::FileStorage::READ);
		fs2["warpMatrix"] >> warp;
		fs2.release();

		if (nullptr == warp.data)
			return 0;

		return 1;
	}

	//*****************************************************************************
	//
	//
	//
	//*****************************************************************************

	int CameraAlign::saveWarp(const std::string fileName, const cv::Mat& warp)
	{
		cv::FileStorage fs(fileName, cv::FileStorage::WRITE);
		fs << "warpMatrix" << warp;
		fs.release();
		return 1;
	}

	//*****************************************************************************
	//
	//
	//
	//*****************************************************************************

	static void manualRectifyMouseCallback(int event, int x, int y, int flags, void* userdata)
	{
		if (event == cv::MouseEventTypes::EVENT_LBUTTONDOWN)
		{
			matchPoint.begin.x = x;
			matchPoint.begin.y = y;
		}
		if (event == cv::MouseEventTypes::EVENT_LBUTTONUP)
		{
			if (matchPointEndWidthOffset > x)
				return;

			matchPoint.end.x = x;
			matchPoint.end.y = y;

			cv::line(combinedImage, matchPoint.begin,
				matchPoint.end, cv::Scalar(110, 220, 0), 1, 8);

			cv::imshow(rectifyWindowsName, combinedImage);

			matchPoint.end.x -= matchPointEndWidthOffset;
			matchPoints.push_back({ matchPoint });

			printf("Added matchPoint (%i,%i),(%i,%i)\r\n",
				matchPoint.begin.x, matchPoint.begin.y, matchPoint.end.x, matchPoint.end.y);
		}
	}

	static void clearButtonCallback(int state, void* userdata)
	{

	}

	//*****************************************************************************
	//
	//
	//
	//*****************************************************************************

	void CameraAlign::getSideBySideImage(const cv::Mat& im1, const cv::Mat& im2, 
		cv::Mat& imCombined, const std::string windowName)
	{
		cv::Mat combined( std::max(im1.size().height, im2.size().height),
			im1.size().width + im2.size().width, CV_8UC3);

		matchPointEndWidthOffset = im1.size().width;

		printf("combined frame c r : %i %i\r\n", combined.cols, combined.rows);

		cv::Mat left_roi(combined, cv::Rect(0, 0, im1.size().width, im1.size().height));
		im1.copyTo(left_roi);
		cv::Mat right_roi(combined, cv::Rect(im1.size().width, 0, im2.size().width, im2.size().height));
		im2.copyTo(right_roi);

		imCombined = combined;

		if (0 < windowName.length())
		{
			//auto winName = windowName.c_str();
			//auto winName = "aaa1";
			//namedWindow("aaa1", WINDOW_GUI_EXPANDED);
			cv::namedWindow(windowName.c_str());
			cv::createTrackbar("Thermal", windowName.c_str(), &iThermalAlpha, 100);
	  		cv::createTrackbar("Color", windowName.c_str(), &iColorAlpha, 100);

			cv::createButton("Clear", clearButtonCallback, NULL, cv::QT_PUSH_BUTTON, 0 );
			cv::createButton("Fuse", clearButtonCallback, NULL, cv::QT_PUSH_BUTTON, 0 );
			cv::createButton("Accept", clearButtonCallback, NULL, cv::QT_PUSH_BUTTON, 0 );
			cv::createButton("Quit", clearButtonCallback, NULL, cv::QT_PUSH_BUTTON, 0 );

			cv::setMouseCallback(windowName.c_str(), manualRectifyMouseCallback, this);
			cv::imshow(windowName.c_str(), combined);
		}
	}

	//*****************************************************************************
	//
	//
	//
	//*****************************************************************************

	/*void CameraAlign::combineImages(const cv::Mat& im1, const cv::Mat& im2, 
		const cv::Mat& im3, cv::Mat& imCombined)
	{
		cv::Mat combined( std::max(im1.size().height, im2.size().height),
			im1.size().width + im2.size().width, CV_8UC3);

		matchPointEndWidthOffset = im1.size().width;

		printf("combined frame c r : %i %i\r\n", combined.cols, combined.rows);

		cv::Mat left_roi(combined, cv::Rect(0, 0, im1.size().width, im1.size().height));
		im1.copyTo(left_roi);
		cv::Mat right_roi(combined, cv::Rect(im1.size().width, 0, im2.size().width, im2.size().height));
		im2.copyTo(right_roi);

		imCombined = combined;
	}*/

	/*void CameraAlign::combineImages(const cv::Mat& im1, const cv::Mat& im2, 
		const cv::Mat& im3, cv::Mat& imCombined)
	{
		int topHight = std::max(im1.size().height, im2.size().height);
		int width = std::max(im1.size().width + im2.size().width, im3.size().width);

		cv::Mat combined( topHight + im3.size().height,	width, CV_8UC3);

		matchPointEndWidthOffset = im1.size().width;

		printf("combined frame c r : %i %i\r\n", combined.cols, combined.rows);

		cv::Mat left_roi(combined, cv::Rect(0, 0, im1.size().width, im1.size().height));
		im1.copyTo(left_roi);
		cv::Mat right_roi(combined, cv::Rect(im1.size().width, 0, im2.size().width, im2.size().height));
		im2.copyTo(right_roi);
		cv::Mat bottom_roi(combined, cv::Rect(0, topHight, im3.size().width, im3.size().height));
		im3.copyTo(bottom_roi);

		imCombined = combined;
	}*/

	void CameraAlign::combineImages(const cv::Mat& im1, const cv::Mat& im2, 
		const cv::Mat& im3, cv::Mat& imCombined)
	{
		int topHight = im3.size().height;
		int width = im3.size().width;

		cv::Mat combined( topHight,	width, CV_8UC3);

		int i3chan = im3.channels();
		int comchan = combined.channels();

		//matchPointEndWidthOffset = im1.size().width;

		printf("combined frame c r : %i %i\r\n", combined.cols, combined.rows);

		//cv::Mat left_roi(combined, cv::Rect(0, 0, im1.size().width, im1.size().height));
		//im1.copyTo(left_roi);
		//cv::Mat right_roi(combined, cv::Rect(im1.size().width, 0, im2.size().width, im2.size().height));
		//im2.copyTo(right_roi);

		//imshowimshow("wut1", combined);

		cv::Mat bottom_roi = combined(cv::Rect(0, 0, im3.size().width, im3.size().height));
		im3.copyTo(bottom_roi);

		//im3.copyTo(combined);

		//imshow("wut2", combined);

		//colormap::Jet jt;

		combined.copyTo(imCombined);
	}

	//*****************************************************************************
	//
	//
	//
	//*****************************************************************************

	cv::Mat CameraAlign::calculateHomography(std::vector<matchPointType> matchPoints)
	{
		std::vector<cv::Point2f> begin, end;

		while (!matchPoints.empty())
		{
			begin.push_back(matchPoints.back().begin);
			end.push_back(matchPoints.back().end);
			matchPoints.pop_back();
		}

		return findHomography(begin, end);
	}

	//*****************************************************************************
	//
	//
	//
	//*****************************************************************************

	bool staticRectifyImages_ = false;

	void CameraAlign::rectifyManually(cv::Mat& im1, cv::Mat& im2)
	{
		cv::Mat imCombined, homography, imFitted, imBlended, ROI;

		double alpha = 0.5; double beta = 1 - alpha;

		getSideBySideImage(im1, im2, imCombined, rectifyWindowsName);

		combinedImage = imCombined.clone();

		std::cout << "Press: 'c' : clear, 'f' : fuse, 'a' : accept, 'q' : quit" << std::endl;

		while (true)
		{
			/* leave it for another time
			if(!staticRectifyImages_)
			{
				//{
				// lock the images
					boost::shared_lock<boost::shared_mutex> lockRgb(mutexRgbCameraImage_);
					boost::shared_lock<boost::shared_mutex> lockIr(mutexIrCameraImage_);

					//irImage_->image.copyTo(im1);
					//rgbImage_->image.copyTo(im2);
				//}

				cv::Mat combined( std::max(im1.size().height, im2.size().height),
					im1.size().width + im2.size().width, CV_8UC3);

				//matchPointEndWidthOffset = im1.size().width;

				//printf("combined frame c r : %i %i\r\n", combined.cols, combined.rows);

				cv::Mat left_roi(combined, cv::Rect(0, 0, im1.size().width, im1.size().height));
				im1.copyTo(left_roi);
				cv::Mat right_roi(combined, cv::Rect(im1.size().width, 0, im2.size().width, im2.size().height));
				im2.copyTo(right_roi);

				imCombined = combined;

				cv::imshow(rectifyWindowsName.c_str(), combined);
			}*/

			int keyPressed = cv::waitKey(30);

			if (keyPressed == -1)
				continue;

			std::cout << "Key '" << keyPressed << "' pressed" << std::endl;

			switch (keyPressed)
			{
			case 'c':
				matchPoints.clear();
				combinedImage = imCombined.clone();
				imshow(rectifyWindowsName, combinedImage);
				break;
			case 'f':
				// get homogarphy
				homography = calculateHomography(matchPoints);

				saveWarp(warpFileName, homography);

				// Warp source image to destination based on homography
				warpPerspective(im1, imFitted, homography, im2.size());

				cv::imshow("imFitted", imFitted);

				//ROI = im1(Rect(0, 1, imFitted.cols, imFitted.rows));

				//addWeighted(ROI, alpha, imFitted, beta, 0.0, imBlended);

				//imshow("blended", imBlended);

				break;
			case 'a':
				return;
			case 'q':
				return;
			default:
				break;
			}
		}
	}

//*****************************************************************************
//*
//*
//*
//******************************************************************************

int CameraAlign::displayloop()
{
  if(showCloudInStreams_)
  {
    cv::namedWindow(cloudProjectionDisplayName_, 
		cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO | cv::WINDOW_GUI_EXPANDED);
  }

  /*if(showFusedImage_)
  {
    cv::namedWindow(fusedImageDisplayName_);
    cv::createTrackbar("Thermal", fusedImageDisplayName_, &iThermalAlpha, 100);
	  cv::createTrackbar("Color", fusedImageDisplayName_, &iColorAlpha, 100);
  }*/

  cv::Mat imCombined;

  while(true)
  {     
    try
    {
	  imageReady_.wait();

      if(!gotRgbImage_ | !gotIrImage_)
        continue;

	  // temp
	  //rectifyManually(irImage_->image, rgbImage_->image);

	  //break;

	  if(showCloudInStreams_)
      {
        cv::imshow(cloudProjectionDisplayName_, cloudProjectionImage_);
      }

	  combineImages(irImage_->image, rgbImage_->image, cloudProjectionImage_, imCombined);

	  cv::imshow("TTTTemp", imCombined);

      // wait for a new fused image to appear
      /*fusedImageReady_.wait();

      if(showFusedImage_)
      {
        // lock the images
        boost::shared_lock<boost::shared_mutex> lock(mutexFusedImage_);
       
        cv::imshow(fusedImageDisplayName_, fusedImage_);
        cv::waitKey(3);
      }*/

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

void CameraAlign::cloudViewerTimerCallback(const ros::TimerEvent&)
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

void CameraAlign::pcInCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
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
    //cloudOps_.fromROSMsg(*msg, cloudOut, cloudProjectionImage, width, height, scale);

    //cloudProjectionImage_ = cloudProjectionImage.clone();

	cloudOps_.add(*msg);
	cloudOut = cloudOps_.getCurrentCloud();
	cloudProjectionImage_ = cloudOps_.getCurrentProjectionImage(width, height, scale);

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

void CameraAlign::rgbCameraCallback(const sensor_msgs::ImageConstPtr& msg)
{
  ROS_DEBUG("[CameraAlign] rgb image received.");
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

void CameraAlign::irCameraCallback(const sensor_msgs::ImageConstPtr& msg)
{
  ROS_DEBUG("[CameraAlign] ir image received.");
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
}

//*****************************************************************************
//*
//*
//*
//******************************************************************************

int CameraAlign::logloop()
{
  while(true)
  {
    if(NULL != logger_)
      logger_->flush();

    std::this_thread::sleep_for(std::chrono::milliseconds(logloopTimeoutMilliseconds_));
  }

  return 0;
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

void CameraAlign::CreateLogger(std::string logDirectory)
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
      log_thread = std::thread(&CameraAlign::logloop, this); 

      // create time string
      time (&rawtime);
      timeinfo = localtime(&rawtime);
      strftime(buffer,80,"%Y-%m-%d-%H-%M-%S",timeinfo);
      auto fileName = logDirectory + "Mfuse_Align_Log_" + std::string(buffer) + ".txt";

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