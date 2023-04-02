#include "sparse_slam/track.h"
Track::Track()
{
    initialized_ = false;
    cv::FileStorage file_setting("/home/yamabuki/10GDisk/slam_ws/src/sparse_slam/EuRoC.yaml",cv::FileStorage::READ);
    width_ = file_setting["Camera.width"];
    height_ = file_setting["Camera.height"];
    fy_ = file_setting["Camera.fy"];
    fx_ = file_setting["Camera.fx"];
    fy_ = file_setting["Camera.fy"];
    cx_ = file_setting["Camera.cx"];
    cy_ = file_setting["Camera.cy"];
    k_ = cv::Mat::eye(3,3,CV_32F);
    k_.at<float>(0,0) = fx_;
    k_.at<float>(1,1) = fy_;
    k_.at<float>(0,2) = cx_;
    k_.at<float>(1,2) = cy_;
    distcoef_ = cv::Mat::zeros(4,1,CV_32F);
    distcoef_.at<float>(0) = file_setting["Camera.k1"];
    distcoef_.at<float>(1) = file_setting["Camera.k2"];
    distcoef_.at<float>(2) = file_setting["Camera.p1"];
    distcoef_.at<float>(3) = file_setting["Camera.p2"];
    const float k3 = file_setting["Camera.k3"];
    if(k3!=0)
    {
        distcoef_.resize(5);
        distcoef_.at<float>(4) = k3;
    }

}

void Track::GrabImage(cv::Mat &image)
{
    Extractor extractor;
    if (image.cols !=width_ && image.rows!=height_)
    {
        cv::resize(image,image,cv::Size(width_,height_),0,0,cv::INTER_LINEAR);
    }
    std::vector<std::vector<cv::KeyPoint>> layers_keypoints_vec;
    cv::Mat descriptor;
    extractor.extract(image,layers_keypoints_vec,descriptor);
}