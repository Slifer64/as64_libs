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

struct Tag
{
  Tag(int id=-1)
  {
    pos = arma::vec().zeros(3);
    quat = {1, 0, 0, 0};
    this->id = id;
    is_detected = false;
  }

  arma::vec pos;
  arma::vec quat;
  int id;

  bool is_detected;

  std_msgs::Header header;
};

class TagDetectWorker
{
public:

  double a_p;
  double a_q;

  std::map<int, Tag> tags_map;

  ros::NodeHandle nh;
  ros::Subscriber apriltag_sub;
  tf::TransformBroadcaster tf_pub_;

  TagDetectWorker()
  {
    stop_tag_tf_pub = true;

    tags_map[0] = Tag(0);
    tags_map[1] = Tag(1);

  }

  ~TagDetectWorker()
  {
    stop_tag_tf_pub = true;
  }

  void subscribe(const std::string &topic)
  {
    apriltag_sub = nh.subscribe(topic, 1, &TagDetectWorker::tagDetectionsCallback, this);
  }

  void tagDetectionsCallback(const apriltags_ros::AprilTagDetectionArrayConstPtr& tag_detections_ptr)
  {
    apriltags_ros::AprilTagDetectionArray tag_detection_array;

    for (int i=0; i<tag_detections_ptr->detections.size(); i++)
    {
      const apriltags_ros::AprilTagDetection &detection = tag_detections_ptr->detections[i];

      auto it = tags_map.find(detection.id);
      if (it != tags_map.end()) // if the identified tag exist in the map
      {
        Tag &tag = it->second;
        tag.id = detection.id;
        geometry_msgs::Point pos = detection.pose.pose.position;
        geometry_msgs::Quaternion quat = detection.pose.pose.orientation;
        tag.pos = arma::vec({pos.x, pos.y, pos.z});
        tag.quat = arma::vec({quat.w, quat.x, quat.y, quat.z});
        tag.is_detected = true;
        tag.header = detection.pose.header;

//        std::string frame_name = "tag_" + std::to_string(tag.id);
//        tf::Stamped<tf::Transform> tag_transform;
//        tf::poseStampedMsgToTF(detection.pose, tag_transform);
//        tf_pub_.sendTransform(tf::StampedTransform(tag_transform, tag_transform.stamp_, tag_transform.frame_id_, frame_name));
      }
    }
  }

  bool stop_tag_tf_pub;

  void publishTagsTf(double pub_cycle)
  {
    stop_tag_tf_pub = false;

    unsigned long long sleep_time = pub_cycle * 1e9;

    std::thread([this,sleep_time]()
    {
      while (!stop_tag_tf_pub)
      {
        for (auto it = tags_map.begin(); it!=tags_map.end(); it++)
        {
          const Tag &tag = it->second;

          if (!tag.is_detected) continue;

          geometry_msgs::PoseStamped tag_pose;
          tag_pose.pose.position.x = tag.pos(0);
          tag_pose.pose.position.y = tag.pos(1);
          tag_pose.pose.position.z = tag.pos(2);
          tag_pose.pose.orientation.w = tag.quat(0);
          tag_pose.pose.orientation.x = tag.quat(1);
          tag_pose.pose.orientation.y = tag.quat(2);
          tag_pose.pose.orientation.z = tag.quat(3);
          tag_pose.header = tag.header;

          std::string frame_name = "tag_" + std::to_string(tag.id);
          tf::Stamped<tf::Transform> tag_transform;
          tf::poseStampedMsgToTF(tag_pose, tag_transform);
          tf_pub_.sendTransform(tf::StampedTransform(tag_transform, tag_transform.stamp_, tag_transform.frame_id_, frame_name));
        }

        std::this_thread::sleep_for(std::chrono::nanoseconds(sleep_time));
      }
    }).detach();
  }

  void stopTagTfPublish() { stop_tag_tf_pub = true; }

};


void loadParams()
{
  ros::NodeHandle nh("~");
//  if (!nh.getParam("T",T)) throw std::ios_base::failure("Failed to load param \"T\"...\n");
}

int main(int argc, char** argv)
{
  // ===========  Initialize the ROS node  ==================
  ros::init(argc, argv, "apriltag_detect_test_node");
  std::string path = ros::package::getPath("vision_test") + "/";

  loadParams();

  TagDetectWorker tag_detect_worker;
  tag_detect_worker.subscribe("tag_detections");
  tag_detect_worker.publishTagsTf(0.1);

  while (ros::ok())
  {
    ros::spinOnce();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

//  std::string temp;
//  std::cerr << "Program paused. Press enter to continue...\n";
//  std::getline(std::cin, temp);

  // ===========  Shutdown ROS node  ==================
  ros::shutdown();

  return 0;
}
