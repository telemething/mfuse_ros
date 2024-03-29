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
CameraAlign::matchPointType2 matchPoint2;
std::vector<CameraAlign::matchPointType2> matchPoints2;
std::string rectifyWindowsName = "rectify2";
int matchPointEndWidthOffset = 0;

cv::Rect irRoiRect;
cv::Rect visibleRoiRect;
cv::Rect cloudRoiRect;

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
	showCloudInStreams_ = false;

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

	if(showCloudInStreams_)
		cloudViewerTimer_ = nodeHandle_.createTimer(ros::Duration(0.1), 
													&CameraAlign::cloudViewerTimerCallback, this);

	projectionImageWidth_ = 500;
	projectionImageHeight_ = 500;
	projectionImageScale_ = 10;
	projectionImageDepthMin_ = 0;
	projectionImageDepthMax_ = 255;
	projectionImageDepthTrack_ = 127;

	readWarpFiles();
	
	cloudOps_.SetCloudToIrWarp(homographyIrOnCloud_);
	cloudOps_.SetCloudToVisWarp(homographyVisOnCloud_);
	cloudOps_.SetQueueSize(50);
	cloudOps_.RunCurrentCloudCreationThread(projectionImageWidth_, 
		projectionImageHeight_, projectionImageScale_, 100 );
	cloudOps_.SetDepthRange(projectionImageDepthMin_,projectionImageDepthMax_);

	readWarpFiles();

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

	int CameraAlign::readWarpFiles()
	{
		if( 1 == FuseOps::readWarpFile(warpFileNameIrOnVis, homographyIrOnVis_))
			haveHomographyIrOnVis_ = true;

		if( 1 == FuseOps::readWarpFile(warpFileNameIrOnCloud, homographyIrOnCloud_))
		{
			if(!warpToCloud_)
			{
				// invert to get cloud on ir
				homographyIrOnCloud_ = homographyIrOnCloud_.inv();
			}
			haveHomographyIrOnCloud_ = true;
		}

		if( 1 == FuseOps::readWarpFile(warpFileNameVisOnCloud, homographyVisOnCloud_))
		{
			if(!warpToCloud_)
			{
				// invert to get cloud on vis
				homographyVisOnCloud_ = homographyVisOnCloud_.inv();
			}
			haveHomographyVisOnCloud_ = true;
		}

		return 1;
	}

	//*****************************************************************************
	//
	//
	//
	//*****************************************************************************

	int circleRadius_ = 3;
	int circleThickness_ = 1;

	static inline void drawMatchPoint(const cv::Mat& img, cv::Point2i& pnt)
	{
		cv::circle(img, pnt, circleRadius_, 
			cv::Scalar(0, 255, 0), circleThickness_);
		cv::circle(img, pnt, circleRadius_+1, 
			cv::Scalar(0, 0, 255), circleThickness_);

	}

	static inline void drawUnderwayPoint(const cv::Mat& img, cv::Point2i& pnt)
	{
		cv::circle(img, pnt, circleRadius_+4, 
			cv::Scalar(0, 0, 255), circleThickness_ + 1);
		cv::circle(img, pnt, circleRadius_+8, 
			cv::Scalar(0, 0, 255), circleThickness_ + 1);
	}

  	//*****************************************************************************
	//
	//
	//
	//*****************************************************************************

	void CameraAlign::drawMatchPoints(const cv::Mat& img, 
		std::vector<CameraAlign::matchPointType2>& points, 
		CameraAlign::matchPointType2 pointUnderway)
	{
		cv::Point2i lineStart;
		cv::Point2i lineEnd;

		if(pointUnderway.ir.x > 0)
			drawUnderwayPoint(img, pointUnderway.ir);

		if(pointUnderway.visible.x > 0)
			drawUnderwayPoint(img, pointUnderway.visible);

		if(pointUnderway.cloud.x > 0)
			drawUnderwayPoint(img, pointUnderway.cloud);

		for (std::vector<CameraAlign::matchPointType2>::iterator it = points.begin(); 
			it != points.end(); ++it)
		{
			lineStart.x = 0;
			lineEnd.x = 0;

			if(it->cloud.x > 0)
			{
				drawMatchPoint(img, it->cloud);

				lineStart.x = it->cloud.x;
				lineStart.y = it->cloud.y;
			}

			if(it->ir.x > 0)
			{
				drawMatchPoint(img, it->ir);

				if(0 == lineStart.x)
				{
					lineStart.x = it->ir.x;
					lineStart.y = it->ir.y;
				}
				else
				{
					lineEnd.x = it->ir.x;
					lineEnd.y = it->ir.y;
				}
			}

			if(it->visible.x > 0)
			{
				drawMatchPoint(img, it->visible);

				lineEnd.x = it->visible.x;
				lineEnd.y = it->visible.y;
			}

			if(showLines)
			if( lineStart.x > 0)
			{
				cv::line(img, lineStart, lineEnd, cv::Scalar(110, 220, 0), 1, 8);
			}
		}
	}

	//*****************************************************************************
	//
	// V1 : Set is complete when three images have been clicked
	//
	//*****************************************************************************

	/*static void addMatchPoint(cv::Point2i hitPoint)
	{
		if( irRoiRect.contains(hitPoint) )
		{
			matchPoint2.ir = hitPoint;
		}
		else if( visibleRoiRect.contains(hitPoint) )
		{
			matchPoint2.visible = hitPoint;
		}
		else if( cloudRoiRect.contains(hitPoint) )
		{
			matchPoint2.cloud = hitPoint;
		}

		// weve got a trio, push them
		if(matchPoint2.ir.x > 0 & matchPoint2.visible.x > 0 & matchPoint2.cloud.x > 0)
		{
			matchPoints2.push_back(matchPoint2);

			matchPoint2.ir = cv::Point2i(0,0);
			matchPoint2.visible = matchPoint2.ir;
			matchPoint2.cloud = matchPoint2.ir;

			printf("--- matchpoint set complete ---\r\n");
		}
	}*/

	//*****************************************************************************
	//
	// V2: set complete when two points are clicked
	//
	//*****************************************************************************

	static void addMatchPoint(cv::Point2i hitPoint)
	{
		int pointCount = 0; 

		if( irRoiRect.contains(hitPoint) )
		{
			matchPoint2.ir = hitPoint;
		}
		else if( visibleRoiRect.contains(hitPoint) )
		{
			matchPoint2.visible = hitPoint;
		}
		else if( cloudRoiRect.contains(hitPoint) )
		{
			matchPoint2.cloud = hitPoint;
		}

		if(matchPoint2.ir.x > 0)
			pointCount++;
		if(matchPoint2.visible.x > 0)
			pointCount++;
		if(matchPoint2.cloud.x > 0)
			pointCount++;

		// if two points are set, push them
		if(pointCount > 1)
		{
			matchPoints2.push_back(matchPoint2);

			matchPoint2.ir = cv::Point2i(0,0);
			matchPoint2.visible = matchPoint2.ir;
			matchPoint2.cloud = matchPoint2.ir;

			printf("--- matchpoint set complete ---\r\n");
		}
	}

	//*****************************************************************************
	//
	//
	//
	//*****************************************************************************

	int lastMouseEvent_ = cv::MouseEventTypes::EVENT_MOUSEMOVE;

	static void manualRectifyMouseCallback(int event, int x, int y, int flags, void* userdata)
	{
		if (event == cv::MouseEventTypes::EVENT_LBUTTONUP)
		{
			cv::Point2i hitPoint(x,y);

			if(lastMouseEvent_ == cv::MouseEventTypes::EVENT_LBUTTONDOWN)
			printf("--- click (%i,%i)\r\n",x,y);
			cv::displayStatusBar(rectifyWindowsName, "--- click ---", 0);

			addMatchPoint(hitPoint);
		}

		lastMouseEvent_ = event;
	}

	//*****************************************************************************
	//
	//
	//
	//*****************************************************************************

	static void clearButtonCallback(int state, void* userdata)
	{
		matchPoints2.clear();
	}

	//*****************************************************************************
	//
	//
	//
	//*****************************************************************************

	static void fuseButtonCallback(int state, void* userdata)
	{
		auto ca = static_cast<CameraAlign*>(userdata);
		ca->DoFuse();
	}

	//*****************************************************************************
	//
	//
	//
	//*****************************************************************************

	static void acceptButtonCallback(int state, void* userdata)
	{
		auto ca = static_cast<CameraAlign*>(userdata);
		ca->DoAccept();
	}

	//*****************************************************************************
	//
	//
	//
	//*****************************************************************************

	static void quitButtonCallback(int state, void* userdata)
	{
		auto ca = static_cast<CameraAlign*>(userdata);
		ca->DoQuit();
	}

	//*****************************************************************************
	//
	//
	//
	//*****************************************************************************

	static void undoLastButtonCallback(int state, void* userdata)
	{
		matchPoints2.pop_back();
	}

	//*****************************************************************************
	//
	//
	//
	//*****************************************************************************

	static void showLinesCallback(int state, void* userdata)
	{
		auto ca = static_cast<CameraAlign*>(userdata);
		ca->showLines = !ca->showLines;
	}

	//*****************************************************************************
	//
	//
	//
	//*****************************************************************************

	void CameraAlign::DoAccept()
	{
		doOp_ = doOpEnum::DoOpAccept;
	}

	//*****************************************************************************
	//
	//
	//
	//*****************************************************************************

	void CameraAlign::DoQuit()
	{
		doOp_ = doOpEnum::DoOpQuit;
	}

	//*****************************************************************************
	//
	//
	//
	//*****************************************************************************

	void CameraAlign::DoFuse()
	{
		doOp_ = doOpEnum::DoOpFuse;
	}

	//*****************************************************************************
	//
	//
	//
	//*****************************************************************************

	int lastTrack_ = 0;
	int lastMin_ = 0;
	int lastMax_ = 0;

	void CameraAlign::setCloudProjectionDepthMin(int depth)
	{
		if(depth != lastMin_)
		{
			lastTrack_ = (depth + projectionImageDepthMax_) / 2;
			cv::setTrackbarPos("Both", rectifyWindowsName, lastTrack_);
		}

		cloudOps_.SetDepthRange(depth, projectionImageDepthMax_);
	}

	void CameraAlign::setCloudProjectionDepthMax(int depth)
	{
		if(depth != lastMax_)
		{
			lastTrack_ = (depth + projectionImageDepthMin_) / 2;
			cv::setTrackbarPos("Both", rectifyWindowsName, lastTrack_);
		}

		cloudOps_.SetDepthRange(projectionImageDepthMin_, depth);
	}

	void CameraAlign::setCloudProjectionDepthTrack(int depth)
	{
		if(depth != lastTrack_)
		{
			lastMin_ = projectionImageDepthMin_ + depth - lastTrack_;
			lastMax_ = projectionImageDepthMax_ + depth - lastTrack_;
			lastTrack_ = depth;

			cv::setTrackbarPos("Min", rectifyWindowsName, lastMin_);
			cv::setTrackbarPos("Max", rectifyWindowsName, lastMax_);
			
			printf("--- depth change : %i ---\r\n", depth - lastTrack_);
		}
	}

	//*****************************************************************************
	//
	//
	//
	//*****************************************************************************

	void minDepthChanged(int depth, void* parentP)
	{
		auto ca = static_cast<CameraAlign*>(parentP);
		ca->setCloudProjectionDepthMin(depth);
	}

	void maxDepthChanged(int depth, void* parentP)
	{
		auto ca = static_cast<CameraAlign*>(parentP);
		ca->setCloudProjectionDepthMax(depth);
	}

	void trackDepthChanged(int depth, void* parentP)
	{
		auto ca = static_cast<CameraAlign*>(parentP);
		ca->setCloudProjectionDepthTrack(depth);
	}

	//*****************************************************************************
	//
	//
	//
	//*****************************************************************************

	void CameraAlign::showAlignWindow(const cv::Mat& im1, const cv::Mat& im2, 
		const cv::Mat& im3, cv::Mat& imCombined, const std::string windowName)
	{
		combineImages(im1, im2, im3, imCombined);

		if(!alignWindowCreated_)
		{
			if (0 == windowName.length())
			{
				logger_->error("CameraAlign::showAlignWindow() : windowName.length = 0");
				return;
			}
			{
				//namedWindow("aaa1", WINDOW_GUI_EXPANDED);
				cv::namedWindow(windowName.c_str(), cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO | cv::WINDOW_GUI_EXPANDED);
				cv::createTrackbar("Min", windowName.c_str(), &projectionImageDepthMin_, 255, minDepthChanged, this);
				cv::createTrackbar("Max", windowName.c_str(), &projectionImageDepthMax_, 255, maxDepthChanged, this);
				cv::createTrackbar("Both", windowName.c_str(), &projectionImageDepthTrack_, 255,  trackDepthChanged, this);

				cv::createButton("Show Lines", showLinesCallback, this, cv::QT_PUSH_BUTTON, 0 );
				cv::createButton("Undo Last", undoLastButtonCallback, this, cv::QT_PUSH_BUTTON, 0 );
				cv::createButton("Clear", clearButtonCallback, this, cv::QT_PUSH_BUTTON, 0 );
				cv::createButton("Fuse", fuseButtonCallback, this, cv::QT_PUSH_BUTTON, 0 );
				cv::createButton("Accept", acceptButtonCallback, this, cv::QT_PUSH_BUTTON, 0 );
				cv::createButton("Quit", quitButtonCallback, this, cv::QT_PUSH_BUTTON, 0 );

				cv::setMouseCallback(windowName.c_str(), manualRectifyMouseCallback, this);	
				cv::displayStatusBar(windowName.c_str(), "--- select first point ---", 0);

				alignWindowCreated_ = true;
			}

		}
			
		drawMatchPoints(imCombined, matchPoints2, matchPoint2);
		cv::imshow(windowName.c_str(), imCombined);
	}

	//*****************************************************************************
	//
	//
	//
	//*****************************************************************************

	void CameraAlign::showIrOnVisWindow(const cv::Mat& im1, const std::string windowName)
	{
		if(!irOnVisWindowCreated_)
		{
			if (0 == windowName.length())
			{
				logger_->error("CameraAlign::showAlignWindow() : windowName.length = 0");
				return;
			}
			{
				//namedWindow("aaa1", WINDOW_GUI_EXPANDED);
				cv::namedWindow(windowName.c_str(), cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO | cv::WINDOW_GUI_EXPANDED);
				cv::createTrackbar("VisToCloud", windowName.c_str(), &irToVisBlend_, 100);
				irOnVisWindowCreated_ = true;
			}

		}
			
		cv::imshow(windowName.c_str(), im1);
	}

	//*****************************************************************************
	//
	//
	//
	//*****************************************************************************

	void CameraAlign::showIrOnCloudWindow(const cv::Mat& im1, const std::string windowName)
	{
		if(!irOnCloudWindowCreated_)
		{
			if (0 == windowName.length())
			{
				logger_->error("CameraAlign::showAlignWindow() : windowName.length = 0");
				return;
			}
			{
				//namedWindow("aaa1", WINDOW_GUI_EXPANDED);
				cv::namedWindow(windowName.c_str(), cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO | cv::WINDOW_GUI_EXPANDED);
				cv::createTrackbar("VisToCloud", windowName.c_str(), &irToCloudBlend_, 100);
				irOnCloudWindowCreated_ = true;
			}

		}
			
		cv::imshow(windowName.c_str(), im1);
	}

	//*****************************************************************************
	//
	//
	//
	//*****************************************************************************

	void CameraAlign::showVisOnCloudWindow(const cv::Mat& im1, const std::string windowName)
	{
		if(!visOnCloudWindowCreated_)
		{
			if (0 == windowName.length())
			{
				logger_->error("CameraAlign::showAlignWindow() : windowName.length = 0");
				return;
			}
			{
				//namedWindow("aaa1", WINDOW_GUI_EXPANDED);
				cv::namedWindow(windowName.c_str(), cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO | cv::WINDOW_GUI_EXPANDED);
				cv::createTrackbar("VisToCloud", windowName.c_str(), &visToCloudBlend_, 100);
				visOnCloudWindowCreated_ = true;
			}

		}
			
		cv::imshow(windowName.c_str(), im1);
	}

	//*****************************************************************************
	//
	//
	//
	//*****************************************************************************

	void CameraAlign::createFusedWindows()
	{
		cv::namedWindow(imFusedIrOnVisDisplayName_, 
			cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO | cv::WINDOW_GUI_EXPANDED);
		cv::namedWindow(imFusedIrOnCloudDisplayName_, 
			cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO | cv::WINDOW_GUI_EXPANDED);
		cv::namedWindow(imFusedVisOnCloudDisplayName_, 
			cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO | cv::WINDOW_GUI_EXPANDED);
	}

	//*****************************************************************************
	//
	//
	//
	//*****************************************************************************

	void CameraAlign::combineImages(const cv::Mat& irImg, const cv::Mat& visImg, 
		cv::Mat& imCombined)
	{
		cv::Mat combined( std::max(irImg.size().height, visImg.size().height),
			irImg.size().width + visImg.size().width, CV_8UC3);
		combined = 0;

		matchPointEndWidthOffset = irImg.size().width;

		//printf("combined frame c r : %i %i\r\n", combined.cols, combined.rows);

		cv::Mat left_roi(combined, cv::Rect(0, 0, irImg.size().width, irImg.size().height));
		irImg.copyTo(left_roi);
		cv::Mat right_roi(combined, cv::Rect(irImg.size().width, 0, visImg.size().width, visImg.size().height));
		visImg.copyTo(right_roi);

		irRoiRect.x = 0;
		irRoiRect.y = 0;
		irRoiRect.height = irImg.size().height;
		irRoiRect.width = irImg.size().width;

		visibleRoiRect.x = irRoiRect.width;
		visibleRoiRect.y = 0;
		visibleRoiRect.height = visImg.size().height;
		visibleRoiRect.width = visImg.size().width;

		cloudRoiRect.x = 0;
		cloudRoiRect.y = 0;
		cloudRoiRect.height = 0;
		cloudRoiRect.width = 0;

		imCombined = combined;
	}

	//*****************************************************************************
	//
	//
	//
	//*****************************************************************************

	void CameraAlign::combineImages(const cv::Mat& irImg, const cv::Mat& visImg, 
		const cv::Mat& cloudImg, cv::Mat& imCombined)
	{
		int topHight = std::max(irImg.size().height, visImg.size().height);
		int width = std::max(irImg.size().width + visImg.size().width, cloudImg.size().width);

		cv::Mat combined( topHight + cloudImg.size().height,	width, CV_8UC3);
		combined = 0;

		matchPointEndWidthOffset = irImg.size().width;

		//printf("combined frame c r : %i %i\r\n", combined.cols, combined.rows);

		cv::Mat irRoi(combined, cv::Rect(0, 0, irImg.size().width, irImg.size().height));
		irImg.copyTo(irRoi);
		cv::Mat visRoi(combined, cv::Rect(irImg.size().width, 0, visImg.size().width, visImg.size().height));
		visImg.copyTo(visRoi);
		cv::Mat cloudRoi(combined, cv::Rect(0, topHight, cloudImg.size().width, cloudImg.size().height));
		cloudImg.copyTo(cloudRoi);

		irRoiRect.x = 0;
		irRoiRect.y = 0;
		irRoiRect.height = irImg.size().height;
		irRoiRect.width = irImg.size().width;

		visibleRoiRect.x = irRoiRect.width;
		visibleRoiRect.y = 0;
		visibleRoiRect.height = visImg.size().height;
		visibleRoiRect.width = visImg.size().width;

		cloudRoiRect.x = 0;
		cloudRoiRect.y = topHight;
		cloudRoiRect.height = cloudImg.size().height;
		cloudRoiRect.width = cloudImg.size().width;

		imCombined = combined;
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

		return cv::findHomography(begin, end);
	}

	//*****************************************************************************
	//
	//
	//
	//*****************************************************************************

	bool CameraAlign::calculateHomographyIrOnVis(std::vector<matchPointType2> matchPoints, cv::Mat& warpOut)
	{
		std::vector<cv::Point2f> begin, end;

		while (!matchPoints.empty())
		{
			auto shiftedPoint = matchPoints.back();

			if( 0 < shiftedPoint.ir.x && 0 < shiftedPoint.visible.x)
			{
				// adjust for visible image right shift
				shiftedPoint.visible.x -=  visibleRoiRect.x;

				// should result in same as original
				begin.push_back(shiftedPoint.ir);
				end.push_back(shiftedPoint.visible);
			}

			matchPoints.pop_back();
		}

		if(0 == begin.size())
			return false;

		warpOut = findHomography(begin, end);

		return true;
	}

	//*****************************************************************************
	//
	//
	//
	//*****************************************************************************

	bool CameraAlign::calculateHomographyIrOnCloud(std::vector<matchPointType2> matchPoints, cv::Mat& warpOut)
	{
		std::vector<cv::Point2f> begin, end;

		while (!matchPoints.empty())
		{
			auto shiftedPoint = matchPoints.back();

			if( 0 < shiftedPoint.ir.x && 0 < shiftedPoint.cloud.x)
			{
				// adjust for cloud image down shift
				shiftedPoint.cloud.y -= cloudRoiRect.y;

				begin.push_back(shiftedPoint.ir);
				end.push_back(shiftedPoint.cloud);
			}

			matchPoints.pop_back();
		}

		if(0 == begin.size())
			return false;

		warpOut = findHomography(begin, end);

		return true;
	}

	//*****************************************************************************
	//
	//
	//
	//*****************************************************************************

	bool CameraAlign::calculateHomographyVisOnCloud(std::vector<matchPointType2> matchPoints, cv::Mat& warpOut)
	{
		std::vector<cv::Point2f> begin, end;

		while (!matchPoints.empty())
		{
			auto shiftedPoint = matchPoints.back();

			// adjust for visible image right shift
			shiftedPoint.visible.x -=  visibleRoiRect.x;

			if( 0 < shiftedPoint.cloud.x && 0 < shiftedPoint.visible.x)
			{
				// adjust for cloud image down shift
				shiftedPoint.cloud.y -= cloudRoiRect.y;

				// should result in same as original
				begin.push_back(shiftedPoint.visible);
				end.push_back(shiftedPoint.cloud);
			}

			matchPoints.pop_back();
		}

		if(0 == begin.size())
			return false;

		warpOut = findHomography(begin, end);

		return true;
	}

	//*****************************************************************************
	//
	//
	//
	//*****************************************************************************

	bool staticRectifyImages_ = false;

	void CameraAlign::rectifyManually(cv::Mat& irImgIn, cv::Mat& visImgIn, cv::Mat& cloudImgIn)
	{	
		cv::Mat imCombined, irImg, visImg, cloudProjectionImage;
		cv::Mat imFusedIrOnVis, imFusedIrOnCloud, imFusedVisOnCloud;
		int iThermalAlpha_ = 50, iColorAlpha_ = 50;
		int fuseRet = 0;

		double alpha = 0.5; double beta = 1 - alpha;

		showAlignWindow(irImgIn, visImgIn, cloudImgIn, imCombined, rectifyWindowsName);

		combinedImage = imCombined.clone();
 
		FuseOps fo(logger_);

		std::cout << "Press: 'c' : clear, 'f' : fuse, 'a' : accept, 'q' : quit" << std::endl;

		while (true)
		{
			if(!staticRectifyImages_)
			{
				// lock the images
				{
					boost::shared_lock<boost::shared_mutex> lockRgb(mutexRgbCameraImage_);
					irImage_->image.copyTo(irImg);
				}
				{
					boost::shared_lock<boost::shared_mutex> lockIr(mutexIrCameraImage_);
					rgbImage_->image.copyTo(visImg);
				}
				{
					boost::shared_lock<boost::shared_mutex> lockCloudProjection(mutexCloudProjectionImage_);
					cloudProjectionImage_.copyTo(cloudProjectionImage);
				}
				
				showAlignWindow(irImg, visImg, cloudProjectionImage, imCombined, rectifyWindowsName);
			}

			if(true)
			{
				/*if(haveHomographyIrOnVis_)
				{
					fo.fuse(irImage_->image, rgbImage_->image, imFusedIrOnVis, 
						homographyIrOnVis_, irToVisBlend_, iColorAlpha_, true);
					showIrOnVisWindow(imFusedIrOnVis, imFusedIrOnVisDisplayName_);
				}
				if(haveHomographyIrOnCloud_)
				{
					fo.fuse(irImage_->image, cloudProjectionImage, imFusedIrOnCloud, 
						homographyIrOnCloud_, irToCloudBlend_, iColorAlpha_, true);
					showIrOnCloudWindow(imFusedIrOnCloud, imFusedIrOnCloudDisplayName_);
				}*/
				if(haveHomographyVisOnCloud_)
				{
					if(warpToCloud_)
					{
						// forward: vis on cloud
						fuseRet = fo.fuse2(rgbImage_->image, cloudProjectionImage, imFusedVisOnCloud, 
							homographyVisOnCloud_, visToCloudBlend_, 0, false, true);
					}
					else
					{
						// Inverse: cloud on vis
						fuseRet = fo.fuse2(cloudProjectionImage, rgbImage_->image, imFusedVisOnCloud, 
							homographyVisOnCloud_, visToCloudBlend_, 0, false, true);
					}

					if(0 == fuseRet)
						showVisOnCloudWindow(imFusedVisOnCloud, imFusedVisOnCloudDisplayName_);
				}
			}

			int keyPressed = cv::waitKey(30);

			if (keyPressed != -1)
			{
				//std::cout << "Key '" << keyPressed << "' pressed" << std::endl;

				switch (keyPressed)
				{
					case 'c':
						clearButtonCallback(0,0);
						break;
					case 'u':
						undoLastButtonCallback(0,0);
						break;
					case 'f':
						doOp_ = doOpEnum::DoOpFuse;
						break;
					case 'a':
						doOp_ = doOpEnum::DoOpAccept;
						break;
					case 'q':
						doOp_ = doOpEnum::DoOpQuit;
						break;
					default:
						break;
				}
			}

			switch (doOp_)
			{
				case doOpEnum::DoOpFuse:

					doOp_ = doOpEnum::DoOpNothing;

					// get homogarphy
					haveHomographyIrOnVis_ = calculateHomographyIrOnVis(matchPoints2, homographyIrOnVis_);
					haveHomographyIrOnCloud_ = calculateHomographyIrOnCloud(matchPoints2, homographyIrOnCloud_);
					haveHomographyVisOnCloud_ = calculateHomographyVisOnCloud(matchPoints2, homographyVisOnCloud_);

					break;
				case doOpEnum::DoOpAccept:

					doOp_ = doOpEnum::DoOpNothing;

					if(haveHomographyIrOnVis_)
						FuseOps::writeWarpFile(warpFileNameIrOnVis, homographyIrOnVis_);
					if(haveHomographyIrOnCloud_)
						FuseOps::writeWarpFile(warpFileNameIrOnCloud, homographyIrOnCloud_);
					if(haveHomographyVisOnCloud_)
						FuseOps::writeWarpFile(warpFileNameVisOnCloud, homographyVisOnCloud_);

					break;
				case doOpEnum::DoOpQuit:
					doOp_ = doOpEnum::DoOpNothing;
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

      if(!gotRgbImage_ | !gotIrImage_ | !gotcloudProjectionImage_)
        continue;

	  //// temp
	  rectifyManually(irImage_->image, rgbImage_->image, cloudProjectionImage_);

	  //break;

	  if(showCloudInStreams_)
      {
        cv::imshow(cloudProjectionDisplayName_, cloudProjectionImage_);
      }

	  //combineImages(irImage_->image, rgbImage_->image, cloudProjectionImage_, imCombined);
	  //cv::imshow("Test Combined Image", imCombined);

	  showAlignWindow(irImage_->image, rgbImage_->image, 
	  	cloudProjectionImage_, imCombined, rectifyWindowsName);

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
	//cloudOut = cloudOps_.getCurrentCloud();
	

	{
		boost::unique_lock<boost::shared_mutex> lockCloudProjection(mutexCloudProjectionImage_);
		//cloudProjectionImage_ = cloudOps_.getCurrentProjectionImage(width, height, scale);
		cloudProjectionImage_ = cloudOps_.getCurrentProjectionImage();
		gotcloudProjectionImage_ = true;
	}

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