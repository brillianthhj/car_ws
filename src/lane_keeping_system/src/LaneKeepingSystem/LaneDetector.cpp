// Copyright (C) 2023 Grepp CO.
// All rights reserved.

/**
 * @file LaneDetector.cpp
 * @author Jeongmin Kim
 * @author Jeongbin Yim
 * @brief lane detector class source file
 * @version 2.1
 * @date 2023-10-13
 */

#include <numeric>
#include "LaneKeepingSystem/LaneDetector.hpp"

namespace Xycar {

template <typename PREC>
void LaneDetector<PREC>::setConfiguration(const YAML::Node& config)
{
    mImageWidth = config["IMAGE"]["WIDTH"].as<int32_t>();
    mImageHeight = config["IMAGE"]["HEIGHT"].as<int32_t>();
    mOffset = config["IMAGE"]["OFFSET"].as<int32_t>();
    mRectWidth = config["IMAGE"]["RECTWIDTH"].as<int32_t>();
    mRectHeight = config["IMAGE"]["RECTHEIGHT"].as<int32_t>();
    mMargin = config["IMAGE"]["MARGIN"].as<int32_t>();

    mRectLeft = cv::Rect(0, mOffset-10, mRectWidth, mRectHeight);
    mRectRight = cv::Rect(mRectWidth, mOffset-10, mRectWidth, mRectHeight);

    mKF = cv::KalmanFilter(4, 2, 0);
    mMeasurement = cv::Mat::zeros(2, 1, CV_32F);
    mImproved = cv::Mat(4, 1, CV_32F);

    mKF.transitionMatrix = (cv::Mat_<float>(4, 4) << 1,0,1,0, 0,1,0,1, 0,0,1,0, 0,0,0,1);
    setIdentity(mKF.measurementMatrix);
    setIdentity(mKF.processNoiseCov, cv::Scalar::all(1e-4));
    setIdentity(mKF.measurementNoiseCov, cv::Scalar::all(1e-1));
    setIdentity(mKF.errorCovPost, cv::Scalar::all(1));
    randn(mKF.statePost, cv::Scalar::all(0), cv::Scalar::all(0.1));

    mDebugging = config["DEBUG"].as<bool>();
}

template <typename PREC>
std::vector<int32_t> LaneDetector<PREC>::getLposRpos(const cv::Mat& mFrame)
{
    std::vector<int32_t> answer;

    mLpos = findEdges(mFrame(mRectLeft), Direction::LEFT) - mMargin;
	mRpos = findEdges(mFrame(mRectRight), Direction::RIGHT) + mRectWidth + mMargin;

    // // predict
	// if (mLpos < 0)
	// 	mLpos = mImproved.at<float>(0);

	// if (mRpos > 640)
	// 	mRpos = mImproved.at<float>(1);

	// mMeasurement.at<float>(0) = mLpos;
	// mMeasurement.at<float>(1) = mRpos;

	// mKF.correct(mMeasurement);
	// mImproved = mKF.predict();

    answer.push_back(mLpos);
    answer.push_back(mRpos);

    return answer;
}

template <typename PREC>
int32_t LaneDetector<PREC>::findEdges(const cv::Mat& mFrame, Direction direction)
{
	cv::Mat gray, fimg, blr, dy;
    cv::cvtColor(mFrame, gray, cv::COLOR_BGR2GRAY);
	gray.convertTo(fimg, CV_32F);
	GaussianBlur(fimg, blr, cv::Size(), 1.);
	Sobel(blr, dy, CV_32F, 0, 1);
	cv::Mat kernel = getStructuringElement(cv::MORPH_RECT, cv::Size(3,3));
	morphologyEx(dy, dy, cv::MORPH_CLOSE, kernel);

	double minValue, maxValue;
	cv::Point minLoc, maxLoc;

	int32_t halfY = fimg.rows/2;
	cv::Mat roi = dy.row(halfY);
	minMaxLoc(roi, &minValue, &maxValue, &minLoc, &maxLoc);

	int32_t threshold = 90;
	// int32_t xCoord = (maxValue > threshold) ? maxLoc.x : (direction == Direction::LEFT) ? 0 : 320;
    int32_t interval = 40;
	int32_t xCoord = (maxValue > threshold && std::abs(minLoc.x - maxLoc.x) < interval) ? maxLoc.x : (direction == Direction::LEFT) ? 0 : 320;
    std::cout << maxValue << maxLoc << minValue << minLoc << std::endl;

	return xCoord;
}

template <typename PREC>
void LaneDetector<PREC>::visualizeLposRpos(cv::Mat& mFrame)
{
    // draw left cross
    int32_t xLeft = (mLpos > 0) ? mLpos : 0;
    drawCross(mFrame, cv::Point(xLeft, mOffset), cv::Scalar(0, 0, 255));
    putText(mFrame, cv::format("(%d, %d)", xLeft, mOffset),
        cv::Point(xLeft - 50, mOffset - 20),
        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);

    // draw right cross
    int32_t xRight = (mRpos < 640) ? mRpos : 640;
    drawCross(mFrame, cv::Point(xRight, mOffset), cv::Scalar(0, 0, 255));
    putText(mFrame, cv::format("(%d, %d)", xRight, mOffset),
        cv::Point(xRight - 50, mOffset - 20),
        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);

    // draw ractangle
    rectangle(mFrame, cv::Rect(0, mOffset-10, mRectWidth * 2, mRectHeight), cv::Scalar(0, 0, 255), 2);

    cv::imshow("ROI", mFrame);
    cv::waitKey(10);
}

template <typename PREC>
void LaneDetector<PREC>::drawCross(cv::Mat& mFrame, cv::Point pt, cv::Scalar color)
{
	int32_t span = 5;
	line(mFrame, pt + cv::Point(-span, -span), pt + cv::Point(span, span), color, 1, cv::LINE_AA);
	line(mFrame, pt + cv::Point(-span, span), pt + cv::Point(span, -span), color, 1, cv::LINE_AA);
}

template class LaneDetector<float>;
template class LaneDetector<double>;
} // namespace Xycar
