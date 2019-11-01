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

FuseOps::FuseOps(std::shared_ptr<spdlog::logger> logger)
{
  logger_ = logger;
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

int FuseOps::writeWarpFile(const std::string fileName, const cv::Mat& warp)
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

//*****************************************************************************
//*
//*
//*
//******************************************************************************

int FuseOps::fuse(cv::Mat& irImage, cv::Mat& rgbImage, cv::Mat& fusedImage, 
    const cv::Mat& warpMatrix, const int iThermalAlpha, const int iColorAlpha, bool colorize)
{ 
    try
    {
      if(!gotMasks)
      {
        warpedBBox = FuseOps::getTransposedBBox(irImage, warpMatrix);
        roiIncludeMask = FuseOps::getMask(rgbImage, warpedBBox, true);
        cv::bitwise_not(roiIncludeMask, roiExcludeMask);
        gotMasks = true;
      }

      thermalAlpha = iThermalAlpha / 100.0;
      colorAlpha = iColorAlpha / 100.0;

      if(colorize)
      {
        // invert pixel intensity so that colorizer works in correct direction
        cv::bitwise_not(irImage, irImage);

        // colorize 
        cv::applyColorMap(irImage, imColorized, cv::ColormapTypes::COLORMAP_RAINBOW);

        // show the image
        if (showDebugImages_)
          cv::imshow("imColorized", imColorized);

        // merge the color and gray thermal images
        cv::addWeighted(irImage, 1 - colorAlpha, imColorized, colorAlpha, 0.0, irImage);
      }

      // warp the thermal image to the perspective of the visible image
      cv::warpPerspective(irImage, imWarped, warpMatrix, rgbImage.size());

      // show the image
      if (showDebugImages_)
        cv::imshow("imWarped", imWarped);

      // mask out the visible image outside of thermal viewport
      rgbImage.copyTo(roiIncludeVisibleImage, roiIncludeMask);

      {
        //lock out_image_, dont modify out_image_ before here
        //---- boost::shared_lock<boost::shared_mutex> lock(mutexFusedImage_);

        // merge the visible and thermal images in the thermal viewport
        cv::addWeighted(imWarped, 1 - thermalAlpha, roiIncludeVisibleImage, thermalAlpha, 0.0, fusedImage);

        // show the image
        if (showDebugImages_)
          cv::imshow("imFusedSmall", fusedImage);

        // mask out the visible area inside the thermal viewport
        rgbImage.copyTo(roiExcludeVisibleImage, roiExcludeMask);

        // merge the visible and thermal images in the visible viewport
        cv::addWeighted(fusedImage, 1, roiExcludeVisibleImage, 1, 0.0, fusedImage);

        //----fusedImageReady_.post();

        if (showDebugImages_)
          FuseOps::DrawROI(fusedImage, warpedBBox);
      }

      if(showDebugImages_)
        cv::waitKey(3);
    }
    catch(const std::exception& e)
    {
      //ROS_ERROR("--- EXCEPTION --- CameraFuse::fusionloop: %s", e.what());
      logger_->error("- EXCEPTION --- FuseOps::fuse(): {}", e.what());
    }
    catch(...)
    {
      //ROS_ERROR("--- EXCEPTION --- CameraFuse::fusionloop: -undefined-");
      logger_->error("--- EXCEPTION --- FuseOps::fuse)(): -undefined-");
    }

  return 0; 
}

//*****************************************************************************
//*
//*
//*
//******************************************************************************

int FuseOps::fuse2(cv::Mat& irImage, cv::Mat& rgbImage, cv::Mat& fusedImage, 
    const cv::Mat& warpMatrix, const int iThermalAlpha, const int iColorAlpha, 
    bool doColorize, bool doWarp)
{ 
    try
    {
      if(!gotMasks)
      {
        warpedBBox = FuseOps::getTransposedBBox(irImage, warpMatrix);
        roiIncludeMask = FuseOps::getMask(rgbImage, warpedBBox, true);
        cv::bitwise_not(roiIncludeMask, roiExcludeMask);
        gotMasks = true;
      }

      thermalAlpha = iThermalAlpha / 100.0;
      colorAlpha = iColorAlpha / 100.0;

      if(doColorize)
      {
        // invert pixel intensity so that colorizer works in correct direction
        cv::bitwise_not(irImage, irImage);

        // colorize 
        cv::applyColorMap(irImage, imColorized, cv::ColormapTypes::COLORMAP_RAINBOW);

        // show the image
        if (showDebugImages_)
          cv::imshow("imColorized", imColorized);

        // merge the color and gray thermal images
        cv::addWeighted(irImage, 1 - colorAlpha, imColorized, colorAlpha, 0.0, irImage);
      }

      if(doWarp)
        // warp the thermal image to the perspective of the visible image
        cv::warpPerspective(irImage, imWarped, warpMatrix, rgbImage.size());
      else
        imWarped = irImage;

      // show the image
      if (showDebugImages_)
        cv::imshow("imWarped", imWarped);

      // mask out the visible image outside of thermal viewport
      rgbImage.copyTo(roiIncludeVisibleImage, roiIncludeMask);

      {
        //lock out_image_, dont modify out_image_ before here
        //---- boost::shared_lock<boost::shared_mutex> lock(mutexFusedImage_);

        // merge the visible and thermal images in the thermal viewport
        cv::addWeighted(imWarped, 1 - thermalAlpha, roiIncludeVisibleImage, thermalAlpha, 0.0, fusedImage);

        // show the image
        if (showDebugImages_)
          cv::imshow("imFusedSmall", fusedImage);

        // mask out the visible area inside the thermal viewport
        rgbImage.copyTo(roiExcludeVisibleImage, roiExcludeMask);

        // merge the visible and thermal images in the visible viewport
        cv::addWeighted(fusedImage, 1, roiExcludeVisibleImage, 1, 0.0, fusedImage);

        //----fusedImageReady_.post();

        if (showDebugImages_)
          FuseOps::DrawROI(fusedImage, warpedBBox);
      }

      if(showDebugImages_)
        cv::waitKey(3);
    }
    catch(const std::exception& e)
    {
      //ROS_ERROR("--- EXCEPTION --- CameraFuse::fusionloop: %s", e.what());
      logger_->error("- EXCEPTION --- FuseOps::fuse(): {}", e.what());
    }
    catch(...)
    {
      //ROS_ERROR("--- EXCEPTION --- CameraFuse::fusionloop: -undefined-");
      logger_->error("--- EXCEPTION --- FuseOps::fuse)(): -undefined-");
    }

  return 0; 
}

} // namespace mfuse