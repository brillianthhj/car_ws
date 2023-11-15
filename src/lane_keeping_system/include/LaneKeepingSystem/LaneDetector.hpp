#ifndef LANE_DETECTOR_HPP_
#define LANE_DETECTOR_HPP_

#include "opencv2/opencv.hpp"
#include <cmath>
#include <cstdlib>
#include <yaml-cpp/yaml.h>

/// create your lane detecter
/// Class naming.. it's up to you.
namespace Xycar {

enum class Direction : uint8_t
{
	// lane direction
	LEFT = 0,
	RIGHT = 1
};

template <typename PREC>
class LaneDetector final
{
public:
    using Ptr = LaneDetector*; /// < Pointer type of the class(it's up to u)

    static inline const cv::Scalar kRed = {0, 0, 255}; /// Scalar values of Red
    static inline const cv::Scalar kGreen = {0, 255, 0}; /// Scalar values of Green
    static inline const cv::Scalar kBlue = {255, 0, 0}; /// Scalar values of Blue

    LaneDetector(const YAML::Node& config) {setConfiguration(config);}
    std::vector<int32_t> getLposRpos(const cv::Mat& frame);
    void visualizeLposRpos(cv::Mat& mFrame);

private:
    int32_t mImageWidth;
    int32_t mImageHeight;
    int32_t mOffset;
    int32_t mRectHeight;
    int32_t mRectWidth;
    int32_t mMargin;

    int32_t mLpos;
    int32_t mRpos;

    // left, right rectangles
    cv::Rect mRectLeft;
    cv::Rect mRectRight;

    // kalman filter configs
    cv::KalmanFilter mKF;
    cv::Mat mMeasurement;
    cv::Mat mImproved;

    // Debug Image and flag
    cv::Mat mDebugFrame; /// < The frame for debugging
    void setConfiguration(const YAML::Node& config);
    bool mDebugging;

    int32_t findEdges(const cv::Mat& mFrame, Direction direction);
    void drawCross(cv::Mat& mFrame, cv::Point pt, cv::Scalar color);
};
}

#endif // LANE_DETECTOR_HPP_