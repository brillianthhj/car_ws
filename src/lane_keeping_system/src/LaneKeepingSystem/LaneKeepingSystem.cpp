#include "LaneKeepingSystem/LaneKeepingSystem.hpp"

namespace Xycar {

template <typename PREC>
LaneKeepingSystem<PREC>::LaneKeepingSystem()
{
    std::cout << "Lane keeping system initializing..." << std::endl;

    std::string configPath;
    mNodeHandler.getParam("config_path", configPath);
    YAML::Node config = YAML::LoadFile(configPath);

    mPID = new PIDController<PREC>(config["PID"]["P_GAIN"].as<PREC>(), config["PID"]["I_GAIN"].as<PREC>(), config["PID"]["D_GAIN"].as<PREC>());
    mMovingAverage = new MovingAverageFilter<PREC>(config["MOVING_AVERAGE_FILTER"]["SAMPLE_SIZE"].as<uint32_t>());
    mLaneDetector = new LaneDetector<PREC>(config);
    /*
        create your lane detector.
    */
    setParams(config);

    mPublisher = mNodeHandler.advertise<xycar_msgs::xycar_motor>(mPublishingTopicName, mQueueSize);
    mSubscriber = mNodeHandler.subscribe(mSubscribedTopicName, mQueueSize, &LaneKeepingSystem::imageCallback, this);

    mMsgPublisher = mNodeHandler.advertise<perception_msgs::LaneVisionMsg>(mMsgPublishingTopicName, mQueueSize);
}

template <typename PREC>
void LaneKeepingSystem<PREC>::setParams(const YAML::Node& config)
{
    mPublishingTopicName = config["TOPIC"]["PUB_NAME"].as<std::string>();
    mSubscribedTopicName = config["TOPIC"]["SUB_NAME"].as<std::string>();
    mMsgPublishingTopicName = config["TOPIC"]["PUB_MSG_NAME"].as<std::string>();
    mQueueSize = config["TOPIC"]["QUEUE_SIZE"].as<uint32_t>();
    mXycarSpeed = config["XYCAR"]["START_SPEED"].as<PREC>();
    mXycarMaxSpeed = config["XYCAR"]["MAX_SPEED"].as<PREC>();
    mXycarMinSpeed = config["XYCAR"]["MIN_SPEED"].as<PREC>();
    mXycarSpeedControlThreshold = config["XYCAR"]["SPEED_CONTROL_THRESHOLD"].as<PREC>();
    mAccelerationStep = config["XYCAR"]["ACCELERATION_STEP"].as<PREC>();
    mDecelerationStep = config["XYCAR"]["DECELERATION_STEP"].as<PREC>();
    mDebugging = config["DEBUG"].as<bool>();
}

template <typename PREC>
LaneKeepingSystem<PREC>::~LaneKeepingSystem()
{
    delete mPID;
    delete mMovingAverage;
    // delete your LaneDetector if you add your LaneDetector.
}

template <typename PREC>
void LaneKeepingSystem<PREC>::run()
{
    std::cout << "Now `run()` method running..." << std::endl;
    ros::Rate rate(kFrameRate);

    //PREC speed = 3.;
    //PREC angle = 3.;

    while (ros::ok())
    {
        ros::spinOnce();

        if (mFrame.empty())
            continue;
        // Step1) Get ptsLeft, ptsRight
        std::vector<int32_t> pts = mLaneDetector->getLposRpos(mFrame);

        perception_msgs::LaneVisionMsg laneMessage;
        laneMessage.lpos = pts[0];
        laneMessage.rpos = pts[1];

        mMsgPublisher.publish(laneMessage);

        // Step2) Draw lanes
        mLaneDetector->visualizeLposRpos(mFrame);

        // Step3) MovingAverageFilter.cpp & PIDController.cpp steering
        int32_t error = static_cast<int32_t>((pts[0] + pts[1]) * 0.5 - 320);
        PREC angle = mPID->getControlOutput(error);

        // Step4) Run drive()
        drive(angle);

    }
}

template <typename PREC>
void LaneKeepingSystem<PREC>::imageCallback(const sensor_msgs::Image& message)
{
    cv::Mat src = cv::Mat(message.height, message.width, CV_8UC3, const_cast<uint8_t*>(&message.data[0]), message.step);
    cv::cvtColor(src, mFrame, cv::COLOR_RGB2BGR);
}

template <typename PREC>
void LaneKeepingSystem<PREC>::speedControl(PREC steeringAngle)
{
    if (std::abs(steeringAngle) > mXycarSpeedControlThreshold)
    {
        mXycarSpeed -= mDecelerationStep;
        mXycarSpeed = std::max(mXycarSpeed, mXycarMinSpeed);
        return;
    }

    mXycarSpeed += mAccelerationStep;
    mXycarSpeed = std::min(mXycarSpeed, mXycarMaxSpeed);
}

template <typename PREC>
void LaneKeepingSystem<PREC>::drive(PREC steeringAngle)
{
    xycar_msgs::xycar_motor motorMessage;
    motorMessage.angle = std::round(steeringAngle);
    motorMessage.speed = std::round(mXycarSpeed);

    mPublisher.publish(motorMessage);
}

template class LaneKeepingSystem<float>;
template class LaneKeepingSystem<double>;
} // namespace Xycar
