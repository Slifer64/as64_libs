#include <memory>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <chrono>
#include <thread>

#include <armadillo>

#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <apriltags_ros/apriltags_ros.h>

int counter = 0;

void tagDetectionsCb(const apriltags_ros::AprilTagDetectionArrayPtr& tags_msg)
{

  std::cerr << "===================================\n";
  std::cerr << "----  Tag detections callback  ----\n";
  std::cerr << "*** counter = " << counter++ << "\n";
  for (int i=0; i<tags_msg->detections.size(); i++)
  {
    apriltags_ros::AprilTagDetection &tag = tags_msg->detections[i];
    geometry_msgs::Point pos = tag.pose.pose.position;
    geometry_msgs::Quaternion quat = tag.pose.pose.orientation;
    std::cerr << "position: (" << pos.x << ", " << pos.y << ", " << pos.z << ")\n";
    std::cerr << "quaternion: (" << quat.w << "," << quat.x << ", " << quat.y << ", " << quat.z << ")\n";
    std::cerr << "ID: " << tag.id << "\n";
    std::cerr << "is_good: " << tag.is_good << "\n";
    std::cerr << "------------------------------------\n";
  }
  std::cerr << "===================================\n";

}

int main(int argc, char** argv)
{
  // ===========  Initialize the ROS node  ==================
  ros::init(argc, argv, "apriltag_detect_test_node");
  std::string path = ros::package::getPath("vision_test") + "/";

  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("tag_detections", 1, tagDetectionsCb);

  while (ros::ok())
  {
    ros::spinOnce();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  // ===========  Shutdown ROS node  ==================
  ros::shutdown();

  return 0;
}
