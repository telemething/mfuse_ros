//*****************************************************************************
//
// https://www.programcreek.com/python/example/89436/cv2.addWeighted
// https://www.learnopencv.com/homography-examples-using-opencv-python-c/
// https://stackoverflow.com/questions/17822585/copy-blend-images-of-different-sizes-using-opencv
//
//*****************************************************************************

#define USING_ROS
#define USING_ROS_VIDSOURCE
//#define USING_WINDOWS

#ifdef USING_WINDOWS
#include "pch.h"
#endif 

#ifdef USING_ROS
#include <ros/ros.h>
#endif

#ifdef USING_ROS_VIDSOURCE
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#endif

#include <iostream>
#include "opencv2/opencv.hpp"
#include <opencv2/highgui.hpp>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <boost/interprocess/sync/interprocess_semaphore.hpp>

#ifdef USING_WINDOWS
#include "DeviceEnumerator.h"
#else
#include "DeviceEnumerator.hpp"
#endif 

using namespace cv;

const int MAX_FEATURES = 500;
const float GOOD_MATCH_PERCENT = 0.15f;

bool use_orig = false;
Mat combinedImage;
struct matchPointType { cv::Point2i begin; cv::Point2i end; };
matchPointType matchPoint;
std::vector<matchPointType> matchPoints;
std::string rectifyWindowsName = "rectify2";
int matchPointEndWidthOffset = 0;
std::string warpFileName = "c:\\data\\thewarp.json";
int warpType = MOTION_HOMOGRAPHY;
Mat warpMatrix;

boost::interprocess::interprocess_semaphore out_image_ready_(0);

void alignImages(Mat& im1, Mat& im2, Mat& im1Reg, Mat& h)

{
	namedWindow("Matches", 1);
	namedWindow("gray1", 1);
	namedWindow("gray2", 1);


	// Convert images to grayscale
	Mat im1Gray, im2Gray;
	//cvtColor(im1, im1Gray, CV_BGR2GRAY);
	//cvtColor(im2, im2Gray, CV_BGR2GRAY);

	if (use_orig)
	{
		cvtColor(im1, im1Gray, COLOR_BGR2GRAY);
		cvtColor(im2, im2Gray, COLOR_BGR2GRAY);

		//im1Gray = im1;
		//im2Gray = im2;
	}
	else
	{
		//cvtColor(im1, im1Gray, CV_BGR2GRAY);
		//cvtColor(im2, im2Gray, CV_BGR2GRAY);

		cvtColor(im1, im1Gray, COLOR_BGR2GRAY);
		GaussianBlur(im1Gray, im1Gray, Size(7, 7), 1.5, 1.5);
		Canny(im1Gray, im1Gray, 0, 30, 3);

		cvtColor(im2, im2Gray, COLOR_BGR2GRAY);
		GaussianBlur(im2Gray, im2Gray, Size(7, 7), 1.5, 1.5);
		Canny(im2Gray, im2Gray, 0, 30, 3);
	}

	imshow("gray1", im1Gray);
	imshow("gray2", im2Gray);


	// Variables to store keypoints and descriptors
	std::vector<KeyPoint> keypoints1, keypoints2;
	Mat descriptors1, descriptors2;

	// Detect ORB features and compute descriptors.
	Ptr<Feature2D> orb = ORB::create(MAX_FEATURES);
	orb->detectAndCompute(im1Gray, Mat(), keypoints1, descriptors1);
	orb->detectAndCompute(im2Gray, Mat(), keypoints2, descriptors2);

	// Match features.
	std::vector<DMatch> matches;
	Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
	matcher->match(descriptors1, descriptors2, matches, Mat());

	// Sort matches by score
	std::sort(matches.begin(), matches.end());

	// Remove not so good matches
	const int numGoodMatches = matches.size() * GOOD_MATCH_PERCENT;
	matches.erase(matches.begin() + numGoodMatches, matches.end());


	// Draw top matches
	Mat imMatches;
	drawMatches(im1, keypoints1, im2, keypoints2, matches, imMatches);
	imwrite("matches.jpg", imMatches);

	imshow("Matches", imMatches);


	// Extract location of good matches
	std::vector<Point2f> points1, points2;

	for (size_t i = 0; i < matches.size(); i++)
	{
		points1.push_back(keypoints1[matches[i].queryIdx].pt);
		points2.push_back(keypoints2[matches[i].trainIdx].pt);
	}

	// Find homography
	h = findHomography(points1, points2, RANSAC);

	// Use homography to warp image
	warpPerspective(im1, im1Reg, h, im2.size());

}
Mat GetGradient(Mat src_gray)
{
	Mat grad_x, grad_y;
	Mat abs_grad_x, abs_grad_y;

	int scale = 1;
	int delta = 0;
	int ddepth = CV_32FC1; ;

	// Calculate the x and y gradients using Sobel operator
	Sobel(src_gray, grad_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT);
	convertScaleAbs(grad_x, abs_grad_x);

	Sobel(src_gray, grad_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT);
	convertScaleAbs(grad_y, abs_grad_y);

	// Combine the two gradients
	Mat grad;
	addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad);

	return grad;

}

void align2(Mat& im1, Mat& im2, Mat& im2_aligned, Mat& h)
{
	// Read the images to be aligned
	//Mat im1 = imread("images/image1.jpg");
	//Mat im2 = imread("images/image2.jpg");

	// Convert images to gray scale;
	Mat im1_gray, im2_gray;

	cvtColor(im1, im1_gray, COLOR_BGR2GRAY);
	cvtColor(im2, im2_gray, COLOR_BGR2GRAY);

	//im1_gray = GetGradient(im1_gray);
	//im2_gray = GetGradient(im2_gray);

	//namedWindow("im1_gray", 1);
	//namedWindow("im2_gray", 1);

	//imshow("im1_gray", im1_gray);
	//imshow("im2_gray", im2_gray);

	// Define the motion model
	const int warp_mode = MOTION_EUCLIDEAN;
	//const int warp_mode = MOTION_AFFINE;
	//const int warp_mode = MOTION_HOMOGRAPHY;

	// Set a 2x3 or 3x3 warp matrix depending on the motion model.
	Mat warp_matrix;

	// Initialize the matrix to identity
	if (warp_mode == MOTION_HOMOGRAPHY)
		warp_matrix = Mat::eye(3, 3, CV_32F);
	else
		warp_matrix = Mat::eye(2, 3, CV_32F);

	// Specify the number of iterations.
	int number_of_iterations = 500;

	// Specify the threshold of the increment
	// in the correlation coefficient between two iterations
	double termination_eps = 1e-10;

	TermCriteria criteria(TermCriteria::COUNT + TermCriteria::EPS, number_of_iterations, termination_eps);

	// Run the ECC algorithm. The results are stored in warp_matrix.
	findTransformECC(
		im1_gray,
		im2_gray,
		warp_matrix,
		warp_mode,
		criteria
	);

	// Storage for warped image.
	//Mat im2_aligned;

	if (warp_mode != MOTION_HOMOGRAPHY)
		// Use warpAffine for Translation, Euclidean and Affine
		warpAffine(im2, im2_aligned, warp_matrix, im1.size(), INTER_LINEAR + WARP_INVERSE_MAP);
	else
		// Use warpPerspective for Homography
		warpPerspective(im2, im2_aligned, warp_matrix, im1.size(), INTER_LINEAR + WARP_INVERSE_MAP);

	// Show final result
	imshow("Image 1", im1);
	imshow("Image 2", im2);
	imshow("Image 2 Aligned", im2_aligned);
	waitKey(0);
}

void callbackButton(int state, void* data)
{
	std::cout << "PUSH" << std::endl;
}

//*****************************************************************************
//*****************************************************************************
//*****************************************************************************
//*****************************************************************************
//*****************************************************************************
//*****************************************************************************
//*****************************************************************************
//*****************************************************************************
//*****************************************************************************
//*****************************************************************************
//*****************************************************************************
//*****************************************************************************

class MultispecView
{

private:

	bool showDebugImages = false;
	bool gotWarp = false;
	//VideoCapture capIR, capRGB;
	int iThermalAlpha = 50;
	int iColorAlpha = 50;
	cv::Mat roiIncludeMask, roiExcludeMask;
	std::vector<Point2f> warpedBBox;
	cv::Mat bufferFrameIR, bufferFrameRGB, frameIR, frameRGB;
	cv::Mat out_image_;
	std::chrono::duration<long long, std::milli> videoStreamSleepTime;
	int videoStreamWaitKeyTimeMs = 30;
	int rgbDeviceId_, irDeviceId_;

	std::mutex frameIrMmutex, frameRgbMmutex, outputImageMutex;
	std::thread* ReadStreamIrThread, * ReadStreamRgbThread;
	std::thread* mainLoopThread_, *displayLoopThread_;

#ifdef USING_ROS_VIDSOURCE

	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;

	image_transport::Subscriber image_sub_1;
	image_transport::Subscriber image_sub_2;

	cv_bridge::CvImagePtr cv_ptr1;
  	cv_bridge::CvImagePtr cv_ptr2;

	const std::string DEBUG_WINDOW_1 = "DEBUG_WINDOW_1";
	const std::string DEBUG_WINDOW_2 = "DEBUG_WINDOW_2";

#endif

public:

	//*****************************************************************************
	//
	//
	//
	//*****************************************************************************

#ifdef USING_ROS_VIDSOURCE
	MultispecView() : it_(nh_)
	{
		/*if( 0 != init() )
		{
			printf("Unable to initialize");
		}
		else
			mainLoop();*/
	}
#else
	MultispecView() : it_(nh_)
	{
		/*if( 0 != init() )
		{
			printf("Unable to initialize");
		}
		else
			mainLoop();*/
	}
#endif

	//*****************************************************************************
	//
	//
	//
	//*****************************************************************************

	~MultispecView()
	{
	}

	//*****************************************************************************
	//
	//
	//
	//*****************************************************************************

	int Run(int rgbDeviceId, int irDeviceId)
	{
		rgbDeviceId_ = rgbDeviceId;
		irDeviceId_ = irDeviceId;

		if (0 != init())
		{
			printf("Unable to initialize");
			return -1;
		}
		else
			mainLoop();

		return 0;
	}

	//*****************************************************************************
	//
	//
	//
	//*****************************************************************************

	int RunAsync(int rgbDeviceId, int irDeviceId)
	{
		rgbDeviceId_ = rgbDeviceId;
		irDeviceId_ = irDeviceId;

		//mainLoopThread_ = new std::thread( &MultispecView::mainLoop, this);

		if (0 != init())
		{
			printf("Unable to initialize");
			return -1;
		}

		printf("initialized OK");	
		mainLoopThread_ = new std::thread( &MultispecView::mainLoop, this);
		displayLoopThread_ = new std::thread( &MultispecView::displayLoop, this);
		
		return 0;
	}

	//*****************************************************************************
	//
	//
	//
	//*****************************************************************************

	static int readWarp(const std::string filename, Mat& warp)
	{
		FileStorage fs2(filename, FileStorage::READ);
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

	static int saveWarp(const std::string fileName, const Mat& warp)
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
		if (event == EVENT_LBUTTONDOWN)
		{
			matchPoint.begin.x = x;
			matchPoint.begin.y = y;
		}
		if (event == EVENT_LBUTTONUP)
		{
			if (matchPointEndWidthOffset > x)
				return;

			matchPoint.end.x = x;
			matchPoint.end.y = y;

			cv::line(combinedImage, matchPoint.begin,
				matchPoint.end, Scalar(110, 220, 0), 1, 8);

			imshow(rectifyWindowsName, combinedImage);

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

	void getSideBySideImage(const Mat& im1, const Mat& im2, Mat& imCombined, const std::string windowName)
	{
		Mat combined(max(im1.size().height, im2.size().height),
			im1.size().width + im2.size().width, CV_8UC3);

		matchPointEndWidthOffset = im1.size().width;

		printf("combined frame c r : %i %i\r\n", combined.cols, combined.rows);

		Mat left_roi(combined, Rect(0, 0, im1.size().width, im1.size().height));
		im1.copyTo(left_roi);
		Mat right_roi(combined, Rect(im1.size().width, 0, im2.size().width, im2.size().height));
		im2.copyTo(right_roi);

		imCombined = combined;

		if (0 < windowName.length())
		{
			//auto winName = windowName.c_str();
			//auto winName = "aaa1";
			//namedWindow("aaa1", WINDOW_GUI_EXPANDED);
			namedWindow(windowName.c_str());
			setMouseCallback(windowName.c_str(), manualRectifyMouseCallback, this);
			imshow(windowName.c_str(), combined);
		}
	}

	//*****************************************************************************
	//
	//
	//
	//*****************************************************************************

	Mat calculateHomography(std::vector<matchPointType> matchPoints)
	{
		std::vector<Point2f> begin, end;

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

	void rectifyManually(Mat& im1, Mat& im2)
	{
		Mat imCombined;
		Mat homography;
		Mat imFitted;
		Mat imBlended;

		Mat ROI;

		double alpha = 0.5; double beta = 1 - alpha;

		getSideBySideImage(im1, im2, imCombined, rectifyWindowsName);

		combinedImage = imCombined.clone();

		std::cout << "Press: 'c' : clear, 'f' : fuse, 'a' : accept, 'q' : quit" << std::endl;

		while (true)
		{
			int keyPressed = waitKey(30);

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

				imshow("imFitted", imFitted);

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
	//
	// This is a read loop for outside of ROS
	//
	//*****************************************************************************

	bool gotIrData = false;
	bool gotRgbData = false;

#ifdef USING_ROS_VIDSOURCE

#else
	void ReadStreamIrWorker()
	{
		cv::Mat tempBuff;

		cv::VideoCapture capIR;

		capIR.open(irDeviceId_);

		if (!capIR.isOpened())
		{

			printf("Unable to open IR camera (ID:%i)\r\n", irDeviceId_);
			return;
		}

		while (true)
		{
			{
				std::lock_guard<std::mutex> guard(frameIrMmutex);

				if(capIR.isOpened())
				{
					//capIR >> tempBuff;
					//tempBuff.copyTo(bufferFrameIR);

					capIR.read(tempBuff);

					//if(nullptr == bufferFrameIR)
						bufferFrameIR = tempBuff.clone();
					//else
					//	tempBuff.copyTo(*bufferFrameIR);

					gotIrData = true;
				}
			}

			//cv::waitKey(videoStreamWaitKeyTimeMs);
			std::this_thread::sleep_for(videoStreamSleepTime);
		}
	}

	void ReadStreamRgbWorker()
	{
		cv::Mat tempBuff;

		cv::VideoCapture capRGB;

		capRGB.open(rgbDeviceId_);

		if (!capRGB.isOpened())
		{
			printf("Unable to open RGB camera (ID:%i)\r\n", rgbDeviceId_);
			return;
		}		
		
		while (true)
		{
			{
				std::lock_guard<std::mutex> guard(frameRgbMmutex);

				if(capRGB.isOpened())
				{
					//capRGB >> bufferFrameRGB;
					capRGB.read(tempBuff);

					//if(nullptr == bufferFrameRGB)
						bufferFrameRGB = tempBuff.clone();
					//else
					//	tempBuff.copyTo(*bufferFrameRGB);

					gotRgbData = true;
				}
			}

			//cv::waitKey(videoStreamWaitKeyTimeMs);
			std::this_thread::sleep_for(videoStreamSleepTime);
		}
	}

#endif // USING_ROS_VIDSOURCE

	//*****************************************************************************
	//
	//
	//
	//*****************************************************************************

	void CopyFrameIR(Mat *copyToFrame)
	{
		while(!gotIrData)
		{
			std::this_thread::sleep_for(videoStreamSleepTime);
		}

		//ROS_INFO("--- get frameIrMmutex");
		std::lock_guard<std::mutex> guard(frameIrMmutex);
		//ROS_INFO("got frameIrMmutex ---");

	#ifdef USING_ROS_VIDSOURCE
		cv_ptr1->image.copyTo(*copyToFrame);
	#else
		bufferFrameIR.copyTo(*copyToFrame);
	#endif
	}

	//*****************************************************************************
	//
	//
	//
	//*****************************************************************************

	void CopyFrameRGB(Mat *copyToFrame)
	{
		while(!gotRgbData)
		{
			std::this_thread::sleep_for(videoStreamSleepTime);
		}

		//ROS_INFO("--- get frameRgbMmutex");
		std::lock_guard<std::mutex> guard(frameRgbMmutex);
		//ROS_INFO("got frameRgbMmutex ---");

	#ifdef USING_ROS_VIDSOURCE
		cv_ptr2->image.copyTo(*copyToFrame);
	#else
		bufferFrameRGB.copyTo(*copyToFrame);
	#endif
	}

	//***************************************************************************
	//*
	//*
	//*
	//***************************************************************************

	void imageCb1(const sensor_msgs::ImageConstPtr& msg)
	{
		//cv_bridge::CvImagePtr cv_ptr2;

		try
		{
			//std::lock_guard<std::mutex> guard(frameIrMmutex);

			std::unique_lock<std::mutex> lock(frameIrMmutex, std::try_to_lock);
			if(lock.owns_lock())
			{
				cv_ptr1 = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
				gotIrData = true;
			}
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}

		// Update GUI Window
		cv::imshow(DEBUG_WINDOW_1, cv_ptr1->image);
		cv::waitKey(3);

	}

	//***************************************************************************
	//*
	//*
	//*
	//***************************************************************************

	void imageCb2(const sensor_msgs::ImageConstPtr& msg)
	{
		//cv_bridge::CvImagePtr cv_ptr2;

		try
		{
			//std::lock_guard<std::mutex> guard(frameRgbMmutex);

			std::unique_lock<std::mutex> lock(frameRgbMmutex, std::try_to_lock);
			if(lock.owns_lock())
			{
				cv_ptr2 = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
				gotRgbData = true;
			}
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}

		// Update GUI Window
		cv::imshow(DEBUG_WINDOW_2, cv_ptr2->image);
		cv::waitKey(3);
	}

	//*****************************************************************************
	//
	//
	//
	//*****************************************************************************

	int init()
	{
		std::cout << "--- INIT ---";


		videoStreamSleepTime = std::chrono::milliseconds(100);
		videoStreamWaitKeyTimeMs = 30;

		//bufferFrameIR = new cv::Mat;
		//bufferFrameRGB = new cv::Mat;

		//if (0 > OpenVideoSources())
		//	return -1;

#ifdef USING_ROS_VIDSOURCE

		// Subscribe to input video feed and publish output video feed
		image_sub_1 = it_.subscribe("/flir_boson/image_raw", 1,
		&MultispecView::imageCb1, this);

		// Subscribe to input video feed and publish output video feed
		image_sub_2 = it_.subscribe("/gscam1/image_raw", 1,
		&MultispecView::imageCb2, this);

		//*** debug ***
		cv::namedWindow(DEBUG_WINDOW_1);
    	cv::namedWindow(DEBUG_WINDOW_2);

		ros::spinOnce();

#else
		ReadStreamIrThread = new std::thread(&MultispecView::ReadStreamIrWorker, this);
		ReadStreamRgbThread = new std::thread(&MultispecView::ReadStreamRgbWorker, this);
#endif

		// initialize or load the warp matrix
		if (warpType == MOTION_HOMOGRAPHY)
			warpMatrix = Mat::eye(3, 3, CV_32F);
		else
			warpMatrix = Mat::eye(2, 3, CV_32F);

		gotWarp = readWarp(warpFileName, warpMatrix);

		//capIR >> frameIR;
		//capRGB >> frameRGB;

		CopyFrameIR(&frameIR);
		CopyFrameRGB(&frameRGB);

		//printf("frame 1 c r : %i %i\r\n", frame1.cols, frame1.rows);
		//printf("frame 2 c r : %i %i\r\n", frame2.cols, frame2.rows);

		//imwrite("image1.png", frame1);
		//imwrite("image2.png", frame2);

		if (gotWarp)
		{
			namedWindow("fused", WINDOW_GUI_EXPANDED);
			createTrackbar("Thermal", "fused", &iThermalAlpha, 100);
			createTrackbar("Color", "fused", &iColorAlpha, 100);

			warpedBBox = getTransposedBBox(frameIR, warpMatrix);
			roiIncludeMask = getMask(frameRGB, warpedBBox, true);
			bitwise_not(roiIncludeMask, roiExcludeMask);


			// Debugging area

			/*thermalAlpha = iThermalAlpha / 100.0;
			colorAlpha = iColorAlpha / 100.0;

			bitwise_not(frame2, frame2);
			warpPerspective(frame1, imWarped, warpMatrix, frame2.size());

			imshow("warped", imWarped);

			applyColorMap(frame2, imColorized, COLORMAP_RAINBOW);
			addWeighted(frame2, 1 - colorAlpha, imColorized, colorAlpha, 0.0, frame2);

			imshow("imColorized", imColorized);

			addWeighted(imWarped, 1 - thermalAlpha, frame2, thermalAlpha, 0.0, imFused);
			imshow("fused", imFused);

			imshow("imFused", imFused);*/
		}
		else
		{
			rectifyManually(frameIR, frameRGB);
		}

		// looks odd, but we check again because rectifyManually() can change it
		if (!gotWarp)
		{
			namedWindow("frame1y", WINDOW_GUI_EXPANDED);
			namedWindow("frame2y", WINDOW_GUI_EXPANDED);
		}

		if (showDebugImages)
		{
			imshow("roiIncludeMask", roiIncludeMask);
			imshow("roiExcludeMask", roiExcludeMask);
		}
	}

	//*****************************************************************************
	//
	//
	//
	//*****************************************************************************

	static std::vector<Point2f> getTransposedBBox(const Mat original, const Mat warpMatrix)
	{
		std::vector<Point2f> vIn;
		std::vector<Point2f> vOut;

		vIn.push_back(Point2f(0, 0));
		vIn.push_back(Point2f(0, static_cast<float>(original.rows)));
		vIn.push_back(Point2f(static_cast<float>(original.cols), static_cast<float>(original.rows)));
		vIn.push_back(Point2f(static_cast<float>(original.cols), 0));

		cv::perspectiveTransform(vIn, vOut, warpMatrix);

		return vOut;
	}

	//*****************************************************************************
	//
	//
	//
	//*****************************************************************************

	Mat getMask(const Mat original, const std::vector<Point2f> area, const bool include) const
	{
		std::vector<Point> hull;

		for (std::vector<int>::size_type i = 0; i != area.size(); i++) {
			hull.push_back(Point2i(static_cast<int>(area[i].x), static_cast<int>(area[i].y)));
		}

		// draw black (or white) image 
		Mat roiMask(original.rows, original.cols, CV_8U, include ? Scalar(0) : Scalar(255));

		// fill mask area with white (or black)
		fillConvexPoly(roiMask, hull, include ? Scalar(255) : Scalar(0));

		return roiMask;
	}

	//*****************************************************************************
	//
	//
	//
	//*****************************************************************************

	/*int OpenVideoSources()
	{
		capIR.open(irDeviceId_);
		if (!capIR.isOpened())
		{

			printf("Unable to open IR camera (ID:%i)\r\n", irDeviceId_);
			return -1;
		}

		capRGB.open(rgbDeviceId_);
		if (!capRGB.isOpened())
		{
			printf("Unable to open RGB camera (ID:%i)\r\n", rgbDeviceId_);
			return -1;
		}

		return 1;
	}*/

	//*****************************************************************************
	//
	//
	//
	//*****************************************************************************

	static int DrawROI(Mat image, std::vector<Point2f> outline)
	{
		cv::line(image, outline[0], outline[1], Scalar(110, 220, 0), 1, 8);
		cv::line(image, outline[1], outline[2], Scalar(110, 220, 0), 1, 8);
		cv::line(image, outline[2], outline[3], Scalar(110, 220, 0), 1, 8);
		cv::line(image, outline[3], outline[0], Scalar(110, 220, 0), 1, 8);

		return 0;
	}

	//*****************************************************************************
	//
	//
	//
	//*****************************************************************************

	int mainLoop()
	{
		try
		{
			#ifdef USING_ROS_VIDSOURCE
			// update frequency in hertz
    		double updateFrequency = 30;
			ros::Rate sleepTime(updateFrequency);
			#endif

			Mat imWarped, imColorized;

			double thermalAlpha = .5;
			double colorAlpha = .5;

			Mat roiIncludeVisibleImage, roiExcludeVisibleImage;

			//createButton("TheButton", callbackButton);

			for (;;)
			{
				ROS_INFO("--- get frames");
				CopyFrameIR(&frameIR);
				CopyFrameRGB(&frameRGB);
				ROS_INFO("got frames ---");

				if (gotWarp)
				{				
					thermalAlpha = iThermalAlpha / 100.0;
					colorAlpha = iColorAlpha / 100.0;

					//ROS_INFO("mainLoop a");
					// invert pixel intensity so that colorizer works in correct direction
					bitwise_not(frameIR, frameIR);

					//ROS_INFO("mainLoop b");
					// colorize 
					applyColorMap(frameIR, imColorized, COLORMAP_RAINBOW);

					//ROS_INFO("mainLoop c");
					// show the image
					if (showDebugImages)
						imshow("imColorized", imColorized);

					//ROS_INFO("mainLoop d");
					// merge the color and gray thermal images
					addWeighted(frameIR, 1 - colorAlpha, imColorized, colorAlpha, 0.0, frameIR);

					//ROS_INFO("mainLoop e");
					// warp the thermal image to the perspective of the visible image
					warpPerspective(frameIR, imWarped, warpMatrix, frameRGB.size());

					///ROS_INFO("mainLoop f");
					// show the image
					if (showDebugImages)
						imshow("imWarped", imWarped);

					///ROS_INFO("mainLoop g");
					// mask out the visible image outside of thermal viewport
					frameRGB.copyTo(roiIncludeVisibleImage, roiIncludeMask);

					//lock out_image_, dont modify out_image_ before here
					std::lock_guard<std::mutex> guard(outputImageMutex);

					//ROS_INFO("mainLoop h");
					// merge the visible and thermal images in the thermal viewport
					addWeighted(imWarped, 1 - thermalAlpha, roiIncludeVisibleImage, thermalAlpha, 0.0, out_image_);

					///ROS_INFO("mainLoop i");
					// show the image
					if (showDebugImages)
						imshow("imFusedSmall", out_image_);

					//ROS_INFO("mainLoop j");
					// mask out the visible area inside the thermal viewport
					frameRGB.copyTo(roiExcludeVisibleImage, roiExcludeMask);

					//ROS_INFO("mainLoop k");
					// merge the visible and thermal images in the visible viewport
					addWeighted(out_image_, 1, roiExcludeVisibleImage, 1, 0.0, out_image_);

					//out_image_ready_.post();

					///ROS_INFO("mainLoop l");
					if (showDebugImages)
						DrawROI(out_image_, warpedBBox);

					// This area is for the visible = camera0 thermal = camera1 scenario

					/*bitwise_not(frame2, frame2);
					warpPerspective(frame1, imWarped, warpMatrix, frame2.size());

					applyColorMap(frame2, imColorized, COLORMAP_RAINBOW);
					addWeighted(frame2, 1 - colorAlpha, imColorized, colorAlpha, 0.0, frame2);

					addWeighted(imWarped, 1 - thermalAlpha, frame2, thermalAlpha, 0.0, imFused);*/

					// show the image
					//imshow("fused", imFused);

					//ROS_INFO("mainLoop show");
					//imshow("zebra", imFused);
					//ROS_INFO("mainLoop showed");
				}
				else
				{
					imshow("frame1y", frameIR);
					imshow("frame2y", frameRGB);
				}

			#ifdef USING_ROS_VIDSOURCE
			ROS_INFO("mainLoop sleep");
    		sleepTime.sleep();
			ROS_INFO("mainLoop slept");
			#endif
			//if (waitKey(60) == 'e')
			//	break;
			//waitKey(60);
			}
			// the camera will be deinitialized automatically in VideoCapture destructor
			return 0;
		}
		catch (const std::exception& e)
		{
			ROS_ERROR("mainLoop exception : %s", e.what());

			std::cout << "exception :" << e.what();
			return -1;
		}
		catch (...)
		{
			ROS_ERROR("mainLoop exception");

			std::cout << "exception unknown";
			return -1;
		}

		ROS_INFO("mainLoop return 0");
		return 0;
	}

	std::string DisplayWindowName_ = "Fused";

	//*****************************************************************************
	//
	//
	//
	//*****************************************************************************

	int displayLoop()
	{
    //cv::namedWindow(DisplayWindowName_);

    while(true)
    {     
      //wait for an image to appear
      //out_image_ready_.wait();

      //just loop (avoid locking) if we dont have work
      //if(trackerMode_ != startTracking & trackerMode_ != tracking)
      //  continue;

      //ROS_DEBUG("[tt_tracker] output image ready.");
    
      //lock the image so new images don't overwrite
	  std::lock_guard<std::mutex> guard(outputImageMutex);

      try
      {
        // Display frame.
        cv::imshow(DisplayWindowName_, out_image_);
      }
      catch(const std::exception& e)
      {
        //logger_->debug("--- EXCEPTION --- displayloop : {}",  e.what());	
        ROS_ERROR("--- EXCEPTION --- ccv::imshow: %s", e.what());
      }
      catch(...)
      {
        //logger_->debug("--- EXCEPTION --- displayloop : -undefined-");
        ROS_ERROR("--- EXCEPTION --- ccv::imshow: -undefined-");
      }

	  cv::waitKey(10);
    }

    return 0;
	}
};

int enumerateDevices()
{
	// The id field of the Device struct can be used with an OpenCV VideoCapture object

	DeviceEnumerator de;

	std::map<int, DeviceEnumerator::Device> devices;

	/*
	// Audio Devices
	devices = de.getAudioDevicesMap();

	// Print information about the devices
	for (auto const& device : devices)
	{
		std::cout << "== AUDIO DEVICE (id:" << device.first << ") ==" << std::endl;
		std::cout << "Name: " << device.second.deviceName << std::endl;
		std::cout << "Path: " << device.second.devicePath << std::endl;
	}
	*/

	// Video Devices
	devices = de.getVideoDevicesMap();

	// Print information about the devices
	for (auto const& device : devices)
	{
		std::cout << "== VIDEO DEVICE (id:" << device.first << ") ==" << std::endl;
		std::cout << "Name: " << device.second.deviceName << std::endl;
		std::cout << "Path: " << device.second.devicePath << std::endl;
	}

	return 0;
}

std::tuple<bool, int, int> findVideoDevices(std::string rgbSource, std::string irSource)
{
	int rgbDeviceId = -1;
	int irDeviceId = -1;
	bool foundBothDevices = false;
	DeviceEnumerator de;
	std::map<int, DeviceEnumerator::Device> devices;

	/*
	// Audio Devices
	devices = de.getAudioDevicesMap();

	// Print information about the devices
	for (auto const& device : devices)
	{
		std::cout << "== AUDIO DEVICE (id:" << device.first << ") ==" << std::endl;
		std::cout << "Name: " << device.second.deviceName << std::endl;
		std::cout << "Path: " << device.second.devicePath << std::endl;
	}
	*/

	// Video Devices
	devices = de.getVideoDevicesMap();

	// Print information about the devices
	for (auto const& device : devices)
	{
		if (std::string::npos != device.second.deviceName.find(rgbSource))
		{
			rgbDeviceId = device.first;
			std::cout << "RGB :";
		}
		else if (std::string::npos != device.second.deviceName.find(irSource))
		{
			irDeviceId = device.first;
			std::cout << "IR :";
		}
		else
		{
			std::cout << "other :";
		}

		std::cout << " id:" << device.first << " Name: '" << device.second.deviceName << "'" << std::endl;
	}

	if (-1 < rgbDeviceId & -1 < irDeviceId)
	{
		foundBothDevices = true;
		std::cout << "Located both video sources";
	}
	else
	{
		std::cout << "--- Did not locate both video sources ---";
	}

	return std::make_tuple(foundBothDevices, rgbDeviceId, irDeviceId);
}

int simpleVidint(int deviceID)
{
	Mat frame;
	//--- INITIALIZE VIDEOCAPTURE
	VideoCapture cap;
	// open the default camera using default API
	// cap.open(0);
	// OR advance usage: select any API backend
	int apiID = cv::CAP_ANY;      // 0 = autodetect default API

	// open selected camera using selected API
	//cap.open(deviceID + apiID);
	cap.open(deviceID);

	// check if we succeeded
	if (!cap.isOpened())
	{
		std::cerr << "ERROR! Unable to open camera\n";
		return -1;
	}
	//--- GRAB AND WRITE LOOP
	std::cout << "Start grabbing" << std::endl
		<< "Press any key to terminate" << std::endl;
	for (;;)
	{
		// wait for a new frame from camera and store it into 'frame'
		cap.read(frame);
		// check if we succeeded
		if (frame.empty()) {
			std::cerr << "ERROR! blank frame grabbed\n";
			break;
		}
		// show live and wait for a key with timeout long enough to show images
		imshow("Live", frame);
		if (waitKey(5) >= 0)
			break;
	}
	// the camera will be deinitialized automatically in VideoCapture destructor
	return 0;
}

int simpleVidint(int deviceID1, int deviceID2, int deviceID3)
{
	Mat frame1, frame2, frame3;
	//--- INITIALIZE VIDEOCAPTURE
	VideoCapture cap1, cap2, cap3;
	// open the default camera using default API
	// cap.open(0);
	// OR advance usage: select any API backend
	int apiID = cv::CAP_ANY;      // 0 = autodetect default API

	// open selected camera using selected API
	//cap.open(deviceID + apiID);
	cap1.open(deviceID1);
	cap2.open(deviceID2);
	cap3.open(deviceID3);

	// check if we succeeded
	if (!cap1.isOpened())
	{
		std::cerr << "ERROR! Unable to open camera 1\n";
		return -1;
	}
	// check if we succeeded
	if (!cap2.isOpened())
	{
		std::cerr << "ERROR! Unable to open camera 2\n";
		return -1;
	}
	// check if we succeeded
	if (!cap3.isOpened())
	{
		std::cerr << "ERROR! Unable to open camera 3\n";
		return -1;
	}

	//--- GRAB AND WRITE LOOP
	std::cout << "Start grabbing" << std::endl
		<< "Press any key to terminate" << std::endl;
	for (;;)
	{
		// wait for a new frame from camera and store it into 'frame'
		cap1.read(frame1);
		cap2.read(frame2);
		cap3.read(frame3);

		// check if we succeeded
		if (frame1.empty())
		{
			std::cerr << "ERROR! blank frame grabbed\n";
			break;
		}

		if (frame2.empty())
		{
			std::cerr << "ERROR! blank frame grabbed\n";
			break;
		}

		if (frame3.empty())
		{
			std::cerr << "ERROR! blank frame grabbed\n";
			break;
		}

		// show live and wait for a key with timeout long enough to show images
		imshow("Live1", frame1);
		imshow("Live2", frame2);
		imshow("Live3", frame3);

		if (waitKey(5) >= 0)
			break;
	}
	// the camera will be deinitialized automatically in VideoCapture destructor
	return 0;
}

int lidar_sample_cc_main(int argc, const char* argv[]);
int livox_lidar_main(int argc, char** argv);

int main_mfuse(int argc, char** argv)
{
	// //lidar_sample_cc_main(argc, argv);

	//livox_lidar_main(argc, argv);

	auto devices = findVideoDevices("USB", "FLIR");

	// //simpleVidint(0);
	// //simpleVidint(0,1,2);

	// //namedWindow("first", WINDOW_AUTOSIZE);

	auto vv = getBuildInformation();

	if (!std::get<0>(devices))
	{
		std::cout << "--- cannot continue, exiting now. ---";
		return -1;
	}

	MultispecView msv;
	//msv.Run(std::get<1>(devices), std::get<2>(devices));
	msv.Run(0, 1);
}

int mfuse_startAsync()
{
	//simpleVidint(0);

	auto devices = findVideoDevices("USB", "FLIR");

	auto vv = getBuildInformation();

	if (!std::get<0>(devices))
	{
		std::cout << "--- cannot continue, exiting now. ---";
		return -1;
	}

	MultispecView msv;
	//msv.Run(std::get<1>(devices), std::get<2>(devices));
	msv.RunAsync(1, 0);

	ros::spin();
}


