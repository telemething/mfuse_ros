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
	showCameraInStreams_ = true;

	ROS_INFO("[CameraAlign] Node started.");

	CreateLogger(logDirectory);
	logger_->info("Node started");

	// Read parameters from config file.
	if (!readParameters()) 
	{
		ros::requestShutdown();
	}

	init();

	logger_->info("oky");
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

	int rgbCameraQueueSize; 
	int irCameraQueueSize; 

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

    /*fusionThread_ = std::thread(&CameraFuse::fusionloop, this); 
    displayThread_ = std::thread(&CameraFuse::displayloop, this); 

    // initialize or load the warp matrix
	if (warpType_ == cv::MOTION_HOMOGRAPHY)
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

	//*****************************************************************************
	//
	//
	//
	//*****************************************************************************

	void CameraAlign::getSideBySideImage(const cv::Mat& im1, const cv::Mat& im2, cv::Mat& imCombined, const std::string windowName)
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
			cv::setMouseCallback(windowName.c_str(), manualRectifyMouseCallback, this);
			cv::imshow(windowName.c_str(), combined);
		}
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

	void CameraAlign::rectifyManually(cv::Mat& im1, cv::Mat& im2)
	{
		cv::Mat imCombined, homography, imFitted, imBlended, ROI;

		double alpha = 0.5; double beta = 1 - alpha;

		getSideBySideImage(im1, im2, imCombined, rectifyWindowsName);

		combinedImage = imCombined.clone();

		std::cout << "Press: 'c' : clear, 'f' : fuse, 'a' : accept, 'q' : quit" << std::endl;

		while (true)
		{
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