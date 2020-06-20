#include <apriltags_ros/apriltag_detector.h>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>



namespace apriltags_ros{

class AprilTagDetectorNodelet : public nodelet::Nodelet
{

public:
  AprilTagDetectorNodelet(){}

private:
  void onInit(){
    detector_.reset(new apriltags_ros::AprilTagDetector(getNodeHandle()));
  }
  boost::shared_ptr<apriltags_ros::AprilTagDetector> detector_;

};

}

PLUGINLIB_EXPORT_CLASS(apriltags_ros::AprilTagDetectorNodelet, nodelet::Nodelet)
