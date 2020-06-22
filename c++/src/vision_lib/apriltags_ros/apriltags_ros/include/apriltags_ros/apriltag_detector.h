#ifndef APRILTAG_DETECTOR_H
#define APRILTAG_DETECTOR_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include <AprilTags/TagDetector.h>
#include <tf/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>

#include <memory>
#include <mutex>
#include <condition_variable>
#include <algorithm>
#include <thread>
#include <armadillo>

#include <apriltags_ros/gui/main_window.h>
#include <apriltags_ros/utils/utils.h>

namespace apriltags_ros
{

class AprilTagDetector; // forward decleration

class AprilTagDescription
{

  friend AprilTagDetector;

public:

  AprilTagDescription(int id, double size, std::string &frame_name):
  id_(id), size_(size), frame_name_(frame_name),
  is_good(false), pose_mtx(new std::mutex), missed_frames_num(0) {}

  // copy constructor
  AprilTagDescription(const AprilTagDescription &obj)
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

  ~AprilTagDescription()
  { delete pose_mtx; }

  double size() const { return size_; }

  int getId() const { return id_; }

  std::string frame_name() const { return frame_name_; }

  std::string getRefFrame() const { return pose.header.frame_id; }

  void setPose(const geometry_msgs::PoseStamped &pose)
  {
    std::unique_lock<std::mutex> lck(*pose_mtx);
    this->pose = pose;
  }

  geometry_msgs::PoseStamped getPose() const
  {
    std::unique_lock<std::mutex> lck(*const_cast<std::mutex *>(pose_mtx));
    return this->pose;
  }

  bool isGood() const { return is_good; }

  int hammingDis() const { return hamming_dist; }

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


class AprilTagDetector
{
  friend MainWindow;

public:

  AprilTagDetector(ros::NodeHandle &nh);
  ~AprilTagDetector();

  void setFilter(bool enable) { this->apply_filter = enable; }
  void setPosFilter(double ap) { this->a_p.set(ap); }
  void setOrientFilter(double aq) { this->a_q.set(aq); }

  void launchPublishThread();
  void stopPublishThread();
  void setPublishRate(double pub_rate_sec);

  void launchGUI();

private:

  void imageCb(const sensor_msgs::ImageConstPtr& msg,const sensor_msgs::CameraInfoConstPtr& cam_info);
  std::map<int, AprilTagDescription> parse_tag_descriptions(XmlRpc::XmlRpcValue& april_tag_descriptions);

  geometry_msgs::PoseStamped filterPose(const geometry_msgs::PoseStamped &pose, const AprilTagDescription &description);

  void setImage(cv_bridge::CvImagePtr im_ptr)
  {
    std::unique_lock<std::mutex> lck(im_mtx);
    this->cv_im = im_ptr;
  }

  cv_bridge::CvImagePtr getImage() const
  {
    std::unique_lock<std::mutex> lck(*const_cast<std::mutex *>(&im_mtx));
    return this->cv_im;
  }

private:

  bool use_gui; //
  MainWindow *gui;
  Semaphore gui_finished_sem;

  bool publish_;
  bool publish_tag_tf;
  bool publish_tag_im;
  MtxVar<unsigned long long> pub_rate_nsec;

  bool apply_filter; // apply a 1st order low pass filter to the detected pose
  MtxVar<double> a_p; // filtering coeff for position (a_p = 1 : no filtering)
  MtxVar<double> a_q; // filtering coeff for orientation (a_q = 1 : no filtering)

  MtxVar<int> miss_frames_tol; // if a specific is not detected for more than "tol" then consider it "undetected"

  std::string tag_detections_image_topic; // topic where the detected tags image is published
  std::string tag_detections_topic; // topic where the detected tags are published


  cv_bridge::CvImagePtr cv_im;
  std::mutex im_mtx;
  Semaphore new_msg_sem;


  std::map<int, AprilTagDescription> descriptions_;
  std::string sensor_frame_id_;
  std::shared_ptr<image_transport::ImageTransport> it_;
  image_transport::CameraSubscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Publisher detections_pub_;
  ros::Publisher pose_pub_;
  tf::TransformBroadcaster tf_pub_;
  boost::shared_ptr<AprilTags::TagDetector> tag_detector_;
  bool projected_optics_;

};

} // namespace apriltags_ros

#endif // APRILTAG_DETECTOR_H
