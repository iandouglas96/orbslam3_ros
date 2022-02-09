#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
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
    static ORB_SLAM3::System createSLAM(const ros::NodeHandle& nh);

    //! Image callback, main loop
    void imageCallback(const sensor_msgs::Image::ConstPtr& img_msg);

    //! IMU Callback, main loop if running in inertial mode
    void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg);

    void procBuffers();

    void publishPose(const Sophus::SE3f& pose, const ros::Time& time);

    ros::NodeHandle nh_;
    ros::Subscriber image_sub_, imu_sub_;
    ros::Publisher pose_pub_;

    ORB_SLAM3::System SLAM_;

    //! Recieve buffers
    std::list<cv_bridge::CvImageConstPtr> img_buf_;
    std::list<ORB_SLAM3::IMU::Point> imu_buf_;

    bool initialized_ = false;
    bool tracking_lost_ = false;
    bool use_imu_ = false;
};
