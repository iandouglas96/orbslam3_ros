#include "orbslam3_ros/orbslam3_ros.h"

ORBSLAM3Ros::ORBSLAM3Ros(const ros::NodeHandle& nh) : nh_(nh), SLAM_(createSLAM(nh)) {
  nh.param<bool>("use_imu", use_imu_, false);
}

ORBSLAM3Ros::~ORBSLAM3Ros() {
  SLAM_.Shutdown();
}

ORB_SLAM3::System ORBSLAM3Ros::createSLAM(const ros::NodeHandle& nh) {
  std::string vocab_path, settings_path;
  bool show_viz;
  bool use_imu;

  nh.param<std::string>("vocab_path", vocab_path, "");
  nh.param<std::string>("settings_path", settings_path, "");
  nh.param<bool>("show_viz", show_viz, false);
  nh.param<bool>("use_imu", use_imu, false);

  auto mode = use_imu ? ORB_SLAM3::System::IMU_MONOCULAR : ORB_SLAM3::System::MONOCULAR;
  
  return ORB_SLAM3::System(vocab_path, settings_path, mode, show_viz);
}

void ORBSLAM3Ros::initialize() {
  image_sub_ = nh_.subscribe<sensor_msgs::Image>("image", 30, &ORBSLAM3Ros::imageCallback, this);
  if (use_imu_) {
    imu_sub_ = nh_.subscribe<sensor_msgs::Imu>("imu", 30, &ORBSLAM3Ros::imuCallback, this);
  }
  pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("pose", 1);
}

void ORBSLAM3Ros::imageCallback(const sensor_msgs::Image::ConstPtr& img_msg) {
  cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(img_msg);
  if (use_imu_) {
    img_buf_.push_back(cv_ptr);
  } else {
    Sophus::SE3f pose = SLAM_.TrackMonocular(cv_ptr->image, cv_ptr->header.stamp.toSec());
    publishPose(pose, img_msg->header.stamp);
  }
}

void ORBSLAM3Ros::imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg) {
  cv::Point3f acc(imu_msg->linear_acceleration.x,
                  imu_msg->linear_acceleration.y,
                  imu_msg->linear_acceleration.z);
  cv::Point3f gyr(imu_msg->angular_velocity.x,
                  imu_msg->angular_velocity.y,
                  imu_msg->angular_velocity.z);
  ORB_SLAM3::IMU::Point imu_pt(acc, gyr, imu_msg->header.stamp.toSec());
  imu_buf_.push_back(imu_pt);
  procBuffers();
}

void ORBSLAM3Ros::procBuffers() {
  if (imu_buf_.size() < 1 || img_buf_.size() < 1) return;

  auto img_it = img_buf_.begin();
  while (img_it != img_buf_.end()) {
    auto img_t = (*img_it)->header.stamp.toSec();
    if (imu_buf_.back().t >= img_t) {
      // IMU meas are in the future of this image, so not going to get any more
      std::vector<ORB_SLAM3::IMU::Point> imu_for_img;

      auto imu_it = imu_buf_.begin();
      while (imu_it != imu_buf_.end()) {
        if (imu_it->t <= img_t) {
          // IMU predates image, add to buffer
          imu_for_img.push_back(*imu_it);
          imu_it = imu_buf_.erase(imu_it);
        } else {
          break;
        }
      }

      ROS_INFO_STREAM(imu_for_img.size());
      Sophus::SE3f pose = SLAM_.TrackMonocular((*img_it)->image, img_t, imu_for_img);
      publishPose(pose, (*img_it)->header.stamp);
      img_it = img_buf_.erase(img_it);
    } else {
      break;
    }
  }
}

void ORBSLAM3Ros::publishPose(const Sophus::SE3f& pose, const ros::Time& time) {
  if (pose.translation().norm() > 0.0001 && !tracking_lost_) {
    initialized_ = true;
    Sophus::SE3f pose_inv = pose.inverse();

    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = time;
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
        (ros::Time::now() - pose_msg.header.stamp).toSec() << " sec");
  } else if (!initialized_) {
    ROS_WARN("Initializing...");
  } else {
    ROS_ERROR("Tracking Lost");
    // Blocks from ever publishing again and trashing the map
    tracking_lost_ = true;
  }
}
