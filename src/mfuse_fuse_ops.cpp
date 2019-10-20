//*****************************************************************************
//
// https://www.programcreek.com/python/example/89436/cv2.addWeighted
// https://www.learnopencv.com/homography-examples-using-opencv-python-c/
// https://stackoverflow.com/questions/17822585/copy-blend-images-of-different-sizes-using-opencv
//
//*****************************************************************************

#include <mfuse_fuse_ops.hpp>

namespace mfuse
{

    //*****************************************************************************
//*
//*
//*
//******************************************************************************

FuseOps::FuseOps()
{
}

//*****************************************************************************
//*
//*
//*
//******************************************************************************

FuseOps::~FuseOps()
{
    //boost::unique_lock<boost::shared_mutex> lockNodeStatus(mutexNodeStatus_);
    //isNodeRunning_ = false;
}


//*****************************************************************************
//
//
//
//*****************************************************************************

int FuseOps::readWarpFile(const std::string filename, cv::Mat& warp)
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

int FuseOps::DrawROI(cv::Mat image, std::vector<cv::Point2f> outline)
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

std::vector<cv::Point2f> FuseOps::getTransposedBBox(const cv::Mat original, const cv::Mat warpMatrix)
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

cv::Mat FuseOps::getMask(const cv::Mat original, const std::vector<cv::Point2f> area, const bool include) 
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
} // namespace mfuse