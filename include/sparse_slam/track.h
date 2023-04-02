#pragma once
#include "system.h"

class Track
{
public:
     Track();
     void GrabImage(cv::Mat &image);
     bool initialized_;
     int width_;
     int height_;
     float fx_;
     float fy_;
     float cx_;
     float cy_;
     cv::Mat k_;
     cv::Mat distcoef_;
};