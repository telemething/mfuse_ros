#pragma once

// OpenCV
#include "opencv2/opencv.hpp"

//#include <mfuse_cloud_ops.hpp>
#include <mfuse_logger.hpp>

namespace mfuse
{
class FuseOps 
{
private:

    cv::Mat imWarped, imColorized, roiIncludeVisibleImage, roiExcludeVisibleImage;
    std::vector<cv::Point2f> warpedBBox;
    cv::Mat roiIncludeMask, roiExcludeMask;
    bool showDebugImages_ = false;
    double updateFrequency = 30;
    double thermalAlpha = .5;
    double colorAlpha = .5;
    bool gotMasks = false;
    std::shared_ptr<spdlog::logger> logger_;

public:

    explicit FuseOps(std::shared_ptr<spdlog::logger> logger);
    ~FuseOps();

    static int readWarpFile(const std::string filename, cv::Mat& warp);
    static int writeWarpFile(const std::string fileName, const cv::Mat& warp);
    static int DrawROI(cv::Mat image, std::vector<cv::Point2f> outline);
    static std::vector<cv::Point2f> getTransposedBBox(const cv::Mat original, const cv::Mat warpMatrix);
    static cv::Mat getMask(const cv::Mat original, const std::vector<cv::Point2f> area, const bool include) ;
    int fuse(cv::Mat& irImage, cv::Mat& rgbImage, cv::Mat& fusedImage, 
        const cv::Mat& warpMatrix, const int iThermalAlpha, const int iColorAlpha, bool colorize);

}; // class FuseOps   
} // namespace mfuse



