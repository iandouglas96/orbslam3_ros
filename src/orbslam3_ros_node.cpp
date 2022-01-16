#include <ros/ros.h>
#include "orbslam3_ros/orbslam3_ros.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "orbslam3_ros");
  ros::NodeHandle nh("~");

  try {
    ORBSLAM3Ros node(nh);
    node.initialize();
    ros::spin();
  } catch (const std::exception& e) {
    ROS_ERROR("%s: %s", nh.getNamespace().c_str(), e.what());
  }

  return 0;
}
