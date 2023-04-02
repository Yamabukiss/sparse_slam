#pragma once
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
#include "sparse_slam/extractor.h"
#include "sparse_slam/track.h"
class System
{
public:
    void initialize();
    void receiveFromCam(const sensor_msgs::ImageConstPtr &msg_ptr);
    void imageTest();
    cv_bridge::CvImagePtr image_ptr_{};
    ros::NodeHandle nh_;
    ros::Subscriber img_subscriber_;

};