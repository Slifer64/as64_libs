#ifndef APRILTAG_DESCRIPTION_H
#define APRILTAG_DESCRIPTION_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include <cstring>
#include <memory>
#include <mutex>

namespace apriltags_ros
{

class AprilTagDetector; // forward decleration

class AprilTagDescription
{

  friend AprilTagDetector;

public:

  AprilTagDescription(int id, double size, std::string &frame_name);

  // copy constructor
  AprilTagDescription(const AprilTagDescription &obj);

  ~AprilTagDescription();

  double size() const;

  int getId() const;

  std::string frame_name() const;

  std::string getRefFrame() const;

  void setPose(const geometry_msgs::PoseStamped &pose);

  geometry_msgs::PoseStamped getPose() const;

  bool isGood() const;

  int hammingDis() const;

private:

  int id_;

  std::mutex *pose_mtx;

  geometry_msgs::PoseStamped pose;

  bool is_good;

  int hamming_dist;

  double size_;

  std::string frame_name_;
  std::string ref_frame;

  int missed_frames_num;
};

} // namespace apriltags_ros

#endif // APRILTAG_DESCRIPTION_H
