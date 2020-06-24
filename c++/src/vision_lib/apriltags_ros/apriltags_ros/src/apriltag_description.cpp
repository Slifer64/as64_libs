#include <apriltags_ros/apriltag_description.h>

namespace apriltags_ros
{

AprilTagDescription::AprilTagDescription(int id, double size, std::string &frame_name):
id_(id), size_(size), frame_name_(frame_name),
is_good(false), pose_mtx(new std::mutex), missed_frames_num(0) {}

// copy constructor
AprilTagDescription::AprilTagDescription(const AprilTagDescription &obj)
{
//    std::memcpy((void *)this, (void *)(&obj), sizeof(AprilTagDescription));
  this->id_ = obj.id_;
  this->pose = obj.pose;
  this->is_good = obj.is_good;
  this->hamming_dist = obj.hamming_dist;
  this->size_ = obj.size_;
  this->frame_name_ = obj.frame_name_;
  this->ref_frame = obj.ref_frame;
  this->missed_frames_num = obj.missed_frames_num;

  this->pose_mtx = new std::mutex;
}

AprilTagDescription::~AprilTagDescription()
{
  delete pose_mtx;
}

double AprilTagDescription::size() const
{
  return size_;
}

int AprilTagDescription::getId() const
{
  return id_;
}

std::string AprilTagDescription::frame_name() const { return frame_name_; }

std::string AprilTagDescription::getRefFrame() const
{
  return pose.header.frame_id;
}

void AprilTagDescription::setPose(const geometry_msgs::PoseStamped &pose)
{
  std::unique_lock<std::mutex> lck(*pose_mtx);
  this->pose = pose;
}

geometry_msgs::PoseStamped AprilTagDescription::getPose() const
{
  std::unique_lock<std::mutex> lck(*const_cast<std::mutex *>(pose_mtx));
  return this->pose;
}

bool AprilTagDescription::isGood() const
{
  return is_good;
}

int AprilTagDescription::hammingDis() const
{
  return hamming_dist;
}


} // namespace apriltags_ros

