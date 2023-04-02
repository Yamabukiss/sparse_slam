#include "sparse_slam/system.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "exchanger_node");
    System system;
    system.initialize();
    system.imageTest();
//    while (ros::ok())
//    {
//        ros::spinOnce();
//    }

}
