#include "sparse_slam/system.h"
void System::initialize()
{
    img_subscriber_ = nh_.subscribe("/cam0/image_raw", 1, &System::receiveFromCam, this);

}

void System::imageTest()
{
    cv::Mat image = cv::imread("/home/yamabuki/10GDisk/slam_ws/src/sparse_slam/5.jpg",cv::IMREAD_GRAYSCALE);
    Track tracker ;
    tracker.GrabImage(image);
}

void System::receiveFromCam(const sensor_msgs::ImageConstPtr& msg_ptr)
{
    image_ptr_ = boost::make_shared<cv_bridge::CvImage>(*cv_bridge::toCvShare(msg_ptr, msg_ptr->encoding));
    cv::Mat image = image_ptr_->image.clone();
    if (image.channels() == 3)
    {
        cv::cvtColor(image,image,CV_BGR2GRAY);
    }
    if (image.empty() && image.type()!=CV_8UC1)
        return;

    Track tracker ;
    tracker.GrabImage(image);
}
