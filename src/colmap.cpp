#include <colmap.hpp>

namespace colmap
{
    void applyColorMap(cv::InputArray src, cv::OutputArray dst, int colormap)
    {
        colmap::ColMap* cm =
            colormap == colmap::ColMap::ColormapTypes::COLORMAP_AUTUMN ? (colmap::ColMap*)(new colmap::Autumn) :
            colormap == colmap::ColMap::ColormapTypes::COLORMAP_BONE ? (colmap::ColMap*)(new colmap::Bone) :
            colormap == colmap::ColMap::ColormapTypes::COLORMAP_COOL ? (colmap::ColMap*)(new colmap::Cool) :
            colormap == colmap::ColMap::ColormapTypes::COLORMAP_HOT ? (colmap::ColMap*)(new colmap::Hot) :
            colormap == colmap::ColMap::ColormapTypes::COLORMAP_HSV ? (colmap::ColMap*)(new colmap::HSV) :
            colormap == colmap::ColMap::ColormapTypes::COLORMAP_JET ? (colmap::ColMap*)(new colmap::Jet) :
            colormap == colmap::ColMap::ColormapTypes::COLORMAP_OCEAN ? (colmap::ColMap*)(new colmap::Ocean) :
            colormap == colmap::ColMap::ColormapTypes::COLORMAP_PARULA ? (colmap::ColMap*)(new colmap::Parula) :
            colormap == colmap::ColMap::ColormapTypes::COLORMAP_PINK ? (colmap::ColMap*)(new colmap::Pink) :
            colormap == colmap::ColMap::ColormapTypes::COLORMAP_RAINBOW ? (colmap::ColMap*)(new colmap::Rainbow) :
            colormap == colmap::ColMap::ColormapTypes::COLORMAP_SPRING ? (colmap::ColMap*)(new colmap::Spring) :
            colormap == colmap::ColMap::ColormapTypes::COLORMAP_SUMMER ? (colmap::ColMap*)(new colmap::Summer) :
            colormap == colmap::ColMap::ColormapTypes::COLORMAP_WINTER ? (colmap::ColMap*)(new colmap::Winter) : 0;

        if( !cm )
            CV_Error( cv::Error::StsBadArg, "Unknown colormap id; use one of COLORMAP_*");

        (*cm)(src, dst);

        delete cm;
    }

    void applyColorMap(cv::InputArray src, cv::OutputArray dst, cv::InputArray userColor)
    {
        if (userColor.size() != cv::Size(1,256))
            CV_Error(cv::Error::StsAssert, "cv::LUT only supports tables of size 256.");
        if (userColor.type() != CV_8UC1 && userColor.type() != CV_8UC3)
            CV_Error(cv::Error::StsAssert, "cv::LUT only supports tables CV_8UC1 or CV_8UC3.");
        colmap::UserColorMap cm(userColor.getMat());

        (cm)(src, dst);
    }
}
