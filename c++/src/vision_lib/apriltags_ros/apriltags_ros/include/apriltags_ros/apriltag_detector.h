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
#include <apriltags_ros/apriltag_description.h>

namespace apriltags_ros
{

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
