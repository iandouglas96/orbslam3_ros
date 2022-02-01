#include "orbslam3_ros/orbslam3_ros.h"

ORBSLAM3Ros::ORBSLAM3Ros(const ros::NodeHandle& nh) : nh_(nh), SLAM_(createSLAM(nh)) {}

ORBSLAM3Ros::~ORBSLAM3Ros() {
  SLAM_.Shutdown();
}

ORB_SLAM3::System ORBSLAM3Ros::createSLAM(const ros::NodeHandle& nh) {
  std::string vocab_path, settings_path;
  bool show_viz;

  nh.param<std::string>("vocab_path", vocab_path, "");
  nh.param<std::string>("settings_path", settings_path, "");
  nh.param<bool>("show_viz", show_viz, false);

  return ORB_SLAM3::System(vocab_path, settings_path, ORB_SLAM3::System::MONOCULAR, show_viz);
}

void ORBSLAM3Ros::initialize() {
  image_sub_ = nh_.subscribe<sensor_msgs::Image>("image", 30, &ORBSLAM3Ros::imageCallback, this);
  pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("pose", 1);
}

void ORBSLAM3Ros::imageCallback(const sensor_msgs::Image::ConstPtr& img_msg) {
  cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(img_msg);
  Sophus::SE3f pose = SLAM_.TrackMonocular(cv_ptr->image, cv_ptr->header.stamp.toSec());

  if (pose.translation().norm() > 0.0001 && !tracking_lost_) {
    initialized_ = true;
    Sophus::SE3f pose_inv = pose.inverse();

    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = img_msg->header.stamp;
    pose_msg.header.frame_id = "world";

    pose_msg.pose.position.x = pose_inv.translation()[0];
    pose_msg.pose.position.y = pose_inv.translation()[1];
    pose_msg.pose.position.z = pose_inv.translation()[2];
    pose_msg.pose.orientation.x = pose_inv.unit_quaternion().x();
    pose_msg.pose.orientation.y = pose_inv.unit_quaternion().y();
    pose_msg.pose.orientation.z = pose_inv.unit_quaternion().z();
    pose_msg.pose.orientation.w = pose_inv.unit_quaternion().w();

    pose_pub_.publish(pose_msg);

    ROS_INFO_STREAM("Tracking Latency: " << 
        (pose_msg.header.stamp - ros::Time::now()).toSec() << " sec");
  } else if (!initialized_) {
    ROS_WARN("Initializing...");
  } else {
    ROS_ERROR("Tracking Lost");
    // Blocks from ever publishing again and trashing the map
    tracking_lost_ = true;
  }
}
