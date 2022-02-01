#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <System.h>

class ORBSLAM3Ros {
  public:
    ORBSLAM3Ros(const ros::NodeHandle& nh);
    ~ORBSLAM3Ros();

    void initialize();

  private:
    //! Helper function to create SLAM object
    ORB_SLAM3::System createSLAM(const ros::NodeHandle& nh);

    //! Image callback, main loop
    void imageCallback(const sensor_msgs::Image::ConstPtr& img_msg);

    ros::NodeHandle nh_;
    ros::Subscriber image_sub_;
    ros::Publisher pose_pub_;

    ORB_SLAM3::System SLAM_;

    bool initialized_ = false;
    bool tracking_lost_ = false;
};
