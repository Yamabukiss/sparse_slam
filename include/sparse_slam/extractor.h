#pragma once
#include "sparse_slam/system.h"

class ExtractorNode
{
public:
    ExtractorNode():no_more_(false){}

    void divideNode(ExtractorNode &node1, ExtractorNode &node2, ExtractorNode &node3, ExtractorNode &node4);

    std::vector<cv::KeyPoint> keypoints_vec_;
    cv::Point2i tl_,tr_,br_,bl_;
    std::list<ExtractorNode>::iterator it_;
    bool no_more_;
};


class Extractor
{
public:
    Extractor();
    void extract(const cv::Mat &image,std::vector<std::vector<cv::KeyPoint>> &output_keypoints_vec,cv::Mat &output_descriptors);
    void computePyramid(const cv::Mat& image);
    void computeKeyPointsOctTree(std::vector<std::vector<cv::KeyPoint>> & layers_keypoints_vec);
    std::vector<cv::KeyPoint> distributeOctTree(std::vector<cv::KeyPoint> to_distribute_keypoints_vec,
                           const int min_border_x,const int min_border_y,const int max_border_x,
                           const int max_border_y,const int num_needed_features,const int layer);
    void checkNode(const ExtractorNode &node,int &num_to_expanded_node,std::vector<std::pair<int,ExtractorNode *>> &keypoints_num_node_ptr_vec,std::list<ExtractorNode> &nodes_list);
    void checkNode(const ExtractorNode &node,std::vector<std::pair<int,ExtractorNode *>> &keypoints_num_node_ptr_vec,std::list<ExtractorNode> &nodes_list);
    void computeOrientation(const cv::Mat &image, std::vector<cv::KeyPoint> &keypoints,const std::vector<int> &umax);
    float icAngle(const cv::Mat &image , cv::Point2f keypoint,const std::vector<int> &u_max );
    void computeDescriptors(const cv::Mat &gauss_image, std::vector<cv::KeyPoint> &keypoints,
                            cv::Mat &descriptors,const std::vector<cv::Point>& pattern);
    void computeOrbDescriptor(const cv::KeyPoint& keypoint, const cv::Mat &gauss_image,const cv::Point* pattern,uchar* desc);

    int num_feature_;
    int patch_size_;
    int half_patch_size_;
    int num_layer_;
    float scale_factor_;
    int edge_threshold_;
    int init_fast_thresh_;
    int min_fast_thresh_;
    std::vector<float> scale_factor_vec_;
    std::vector<float> scale_sigma2_vec_;
    std::vector<float> inv_scale_factor_vec_;
    std::vector<float> inv_scale_sigma2_vec_;
    std::vector<cv::Mat> image_pyramid_vec_;
    std::vector<int> layer_needed_feature_vec_;
    std::vector<cv::Point> pattern_;
    std::vector<int> u_max_;
};