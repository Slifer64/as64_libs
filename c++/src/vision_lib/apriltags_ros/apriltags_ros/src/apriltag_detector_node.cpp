#include <apriltags_ros/apriltag_detector.h>
#include <ros/ros.h>

int main(int argc, char **argv){
  ros::init(argc, argv, "apriltag_detector");
  ros::NodeHandle nh;

  apriltags_ros::AprilTagDetector detector(nh);

  // detector.setFilter(true, 0.01, 0.01);

  while (ros::ok())
  {
    ros::spinOnce();
    // do other stuff...
  }
}
