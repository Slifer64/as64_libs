#include <apriltags_ros/apriltag_detector.h>
#include <sensor_msgs/image_encodings.h>
#include <boost/foreach.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <apriltags_ros/AprilTagDetection.h>
#include <apriltags_ros/AprilTagDetectionArray.h>
#include <AprilTags/Tag16h5.h>
#include <AprilTags/Tag25h7.h>
#include <AprilTags/Tag25h9.h>
#include <AprilTags/Tag36h9.h>
#include <AprilTags/Tag36h11.h>
#include <XmlRpcException.h>

#include <QApplication>
#include <QThread>

namespace apriltags_ros
{

AprilTagDetector::AprilTagDetector(ros::NodeHandle &)
{
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  it_.reset(new image_transport::ImageTransport(nh));

  XmlRpc::XmlRpcValue april_tag_descriptions;
  if(!pnh.getParam("tag_descriptions", april_tag_descriptions))
  {
    ROS_WARN("No april tags specified");
  }
  else{
    try{
      descriptions_ = parse_tag_descriptions(april_tag_descriptions);
    } catch(XmlRpc::XmlRpcException e){
      ROS_ERROR_STREAM( "Error loading tag descriptions: " << e.getMessage() );
    }
  }

  if(!pnh.getParam("sensor_frame_id", sensor_frame_id_)) sensor_frame_id_ = "";

  std::string tag_family;
  pnh.param<std::string>("tag_family", tag_family, "36h11");

  pnh.param<bool>("projected_optics", projected_optics_, false);

  const AprilTags::TagCodes* tag_codes;
  if(tag_family == "16h5") tag_codes = &AprilTags::tagCodes16h5;
  else if(tag_family == "25h7") tag_codes = &AprilTags::tagCodes25h7;
  else if(tag_family == "25h9") tag_codes = &AprilTags::tagCodes25h9;
  else if(tag_family == "36h9") tag_codes = &AprilTags::tagCodes36h9;
  else if(tag_family == "36h11") tag_codes = &AprilTags::tagCodes36h11;
  else
  {
    ROS_WARN("Invalid tag family specified; defaulting to 36h11");
    tag_codes = &AprilTags::tagCodes36h11;
  }

  if (!pnh.getParam("tag_detections_image_topic", tag_detections_image_topic)) tag_detections_image_topic = "tag_detections_image";

  if (!pnh.getParam("tag_detections_topic", tag_detections_topic))
    throw std::runtime_error("AprilTagDetector::AprilTagDetector: failed to read param \"tag_detections_topic\"..." );

  if (!pnh.getParam("use_gui", use_gui)) use_gui = false;
  if (!pnh.getParam("apply_filter", apply_filter)) apply_filter = false;
  double ap, aq;
  if (!pnh.getParam("a_p", ap)) ap = 1.0;
  if (!pnh.getParam("a_q", aq)) aq = 1.0;
  a_p.set(ap);
  a_q.set(aq);
  if (!pnh.getParam("miss_frames_tol", miss_frames_tol)) miss_frames_tol = 5;
  if (!pnh.getParam("publish_", publish_)) publish_ = false;
  if (!pnh.getParam("publish_tag_tf", publish_tag_tf)) publish_tag_tf = false;
  if (!pnh.getParam("publish_tag_im", publish_tag_im)) publish_tag_im = false;
  double pub_rate_sec;
  if (!pnh.getParam("pub_rate_sec", pub_rate_sec)) pub_rate_sec = 0.033;
  this->setPublishRate(pub_rate_sec);

  tag_detector_= boost::shared_ptr<AprilTags::TagDetector>(new AprilTags::TagDetector(*tag_codes));
  image_sub_ = it_->subscribeCamera("image_rect", 1, &AprilTagDetector::imageCb, this);
  image_pub_ = it_->advertise(tag_detections_image_topic, 1);
  detections_pub_ = nh.advertise<AprilTagDetectionArray>(tag_detections_topic, 1);
  // pose_pub_ = nh.advertise<geometry_msgs::PoseArray>("tag_detections_pose", 1);

   if (publish_) launchPublishThread();

  if (use_gui) launchGUI();
}

AprilTagDetector::~AprilTagDetector()
{
  emit gui->closeSignal();
  new_msg_sem.notify();
  image_sub_.shutdown();

  gui_finished_sem.wait();
}

void AprilTagDetector::stopPublishThread()
{
  this->publish_ = false;
}

void AprilTagDetector::setPublishRate(double pub_rate_sec)
{
  this->pub_rate_nsec = pub_rate_sec*1e9;
}

void AprilTagDetector::launchPublishThread()
{
  std::thread([this]()
  {
    while (publish_)
    {
      new_msg_sem.wait();

      if (publish_tag_im)
      {
        // std::cerr << "this->cv_ptr: " << ((void *)this->getImage().get()) << "\n";
        image_pub_.publish(this->getImage()->toImageMsg());
      }

      if (publish_tag_tf)
      {
        geometry_msgs::PoseStamped tag_pose;
        for (auto it = descriptions_.begin(); it!=descriptions_.end(); it++)
        {
          const AprilTagDescription &tag_descr = it->second;
          if (!tag_descr.isGood()) continue;
          tag_pose = tag_descr.getPose();
          tf::Stamped<tf::Transform> tag_transform;
          tf::poseStampedMsgToTF(tag_pose, tag_transform);
          tf_pub_.sendTransform(tf::StampedTransform(tag_transform, tag_transform.stamp_, tag_transform.frame_id_, tag_descr.frame_name()));
        }
      }

      std::this_thread::sleep_for(std::chrono::nanoseconds(pub_rate_nsec));
    }
  }).detach();
}

void AprilTagDetector::launchGUI()
{
  Semaphore start_sem;

  std::thread( [this, &start_sem]()
  {
    int argc = 0;
    char **argv = 0;
    QApplication app(argc, argv);

    QThread::currentThread()->setPriority(QThread::LowestPriority);

    gui = new MainWindow(this);

    gui->show();
    start_sem.notify();
    app.exec();

    delete gui;

    gui_finished_sem.notify();
  }).detach();

  start_sem.wait();
}

void AprilTagDetector::imageCb(const sensor_msgs::ImageConstPtr& im_msg, const sensor_msgs::CameraInfoConstPtr& cam_info)
{
  cv_bridge::CvImagePtr cv_ptr;

  // std::cerr << "========> msg->header.frame_id: " << msg->header.frame_id << "\n";

  try
  {
    cv_ptr = cv_bridge::toCvCopy(im_msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

//  std::cerr << "cv_ptr: " << ((void *)cv_ptr.get()) << "\n";

  cv::Mat gray;
  cv::cvtColor(cv_ptr->image, gray, CV_BGR2GRAY);
  std::vector<AprilTags::TagDetection>	detections = tag_detector_->extractTags(gray);
  ROS_DEBUG("%d tag detected", (int)detections.size());

  double fx;
  double fy;
  double px;
  double py;
  if (projected_optics_) {
    // use projected focal length and principal point
    // these are the correct values
    fx = cam_info->P[0];
    fy = cam_info->P[5];
    px = cam_info->P[2];
    py = cam_info->P[6];
  } else {
    // use camera intrinsic focal length and principal point
    // for backwards compatibility
    fx = cam_info->K[0];
    fy = cam_info->K[4];
    px = cam_info->K[2];
    py = cam_info->K[5];
  }

//  std::cout << "================================\n";
//  std::cout << "--------- cam_info->P ----------\n ";
//  std::cout << "fx: " << cam_info->P[0] << "\n";
//  std::cout << "fy: " << cam_info->P[5] << "\n";
//  std::cout << "px: " << cam_info->P[2] << "\n";
//  std::cout << "py: " << cam_info->P[6] << "\n";
//  std::cout << "================================\n";
//  std::cout << "================================\n";
//  std::cout << "--------- cam_info->K ----------\n ";
//  std::cout << "fx: " << cam_info->K[0] << "\n";
//  std::cout << "fy: " << cam_info->K[4] << "\n";
//  std::cout << "px: " << cam_info->K[2] << "\n";
//  std::cout << "py: " << cam_info->K[5] << "\n";
//  std::cout << "================================\n";


  if(!sensor_frame_id_.empty()) cv_ptr->header.frame_id = sensor_frame_id_;

  AprilTagDetectionArray tag_detection_array;
  geometry_msgs::PoseArray tag_pose_array;
  tag_pose_array.header = cv_ptr->header;

  for (auto descr_it = descriptions_.begin(); descr_it!=descriptions_.end(); descr_it++)
  {
    descr_it->second.missed_frames_num++;
    if (descr_it->second.missed_frames_num > miss_frames_tol)
    {
      descr_it->second.missed_frames_num = 0;
      descr_it->second.is_good = false;
    }
  }

  BOOST_FOREACH(AprilTags::TagDetection detection, detections)
  {
    std::map<int, AprilTagDescription>::iterator description_itr = descriptions_.find(detection.id);
    if(description_itr == descriptions_.end())
    {
      ROS_WARN_THROTTLE(10.0, "Found tag: %d, but no description was found for it", detection.id);
      continue;
    }
    AprilTagDescription &description = description_itr->second;
    double tag_size = description.size();

    description.missed_frames_num = 0;

    if (publish_tag_im) detection.draw(cv_ptr->image);

    Eigen::Matrix4d transform = detection.getRelativeTransform(tag_size, fx, fy, px, py);
    Eigen::Matrix3d rot = transform.block(0, 0, 3, 3);
    Eigen::Quaternion<double> rot_quaternion = Eigen::Quaternion<double>(rot);

    geometry_msgs::PoseStamped tag_pose;
    tag_pose.pose.position.x = transform(0, 3);
    tag_pose.pose.position.y = transform(1, 3);
    tag_pose.pose.position.z = transform(2, 3);
    tag_pose.pose.orientation.x = rot_quaternion.x();
    tag_pose.pose.orientation.y = rot_quaternion.y();
    tag_pose.pose.orientation.z = rot_quaternion.z();
    tag_pose.pose.orientation.w = rot_quaternion.w();
    tag_pose.header = cv_ptr->header;

    if (apply_filter) tag_pose = filterPose(tag_pose, description);

    description.setPose(tag_pose);
    description.is_good = detection.good;
    description.hamming_dist = detection.hammingDistance;

    AprilTagDetection tag_detection;
    tag_detection.pose = tag_pose;
    tag_detection.id = detection.id;
    tag_detection.size = tag_size;
    tag_detection.is_good = detection.good;
    tag_detection.hamming_dist = detection.hammingDistance;
    tag_detection_array.detections.push_back(tag_detection);
    // tag_pose_array.poses.push_back(tag_pose.pose);
  }

  // important to do here and not inside the above loop, because in case of no detections
  // the above loop in not entered and setImage won't be called leading initially to an
  // empty image which will be attempted to be published by the the publishThread
  // this->cv_im will be null so the program would crush.
  this->setImage(cv_ptr);

  new_msg_sem.notify();

  detections_pub_.publish(tag_detection_array);

  // pose_pub_.publish(tag_pose_array);
//  if (publish_tag_im) image_pub_.publish(cv_ptr->toImageMsg());
}

geometry_msgs::PoseStamped AprilTagDetector::filterPose(const geometry_msgs::PoseStamped &pose, const AprilTagDescription &description)
{
  if (!description.isGood()) return pose;

  geometry_msgs::PoseStamped filt_pose;
  filt_pose.header = pose.header;
  geometry_msgs::PoseStamped prev_pose = description.getPose();

  double cp = std::min(1.0, a_p.get() * (description.missed_frames_num + 1) );

  filt_pose.pose.position.x = (1-cp)*prev_pose.pose.position.x + cp*pose.pose.position.x;
  filt_pose.pose.position.y = (1-cp)*prev_pose.pose.position.y + cp*pose.pose.position.y;
  filt_pose.pose.position.z = (1-cp)*prev_pose.pose.position.z + cp*pose.pose.position.z;

  double cq = std::min(1.0, a_q.get() * (description.missed_frames_num + 1) );

  double w = (1-cq)*prev_pose.pose.orientation.w + cq*pose.pose.orientation.w;
  double x = (1-cq)*prev_pose.pose.orientation.x + cq*pose.pose.orientation.x;
  double y = (1-cq)*prev_pose.pose.orientation.y + cq*pose.pose.orientation.y;
  double z = (1-cq)*prev_pose.pose.orientation.z + cq*pose.pose.orientation.z;

  double norm = std::sqrt( w*w + x*x + y*y + z*z );

  filt_pose.pose.orientation.w = w/norm;
  filt_pose.pose.orientation.x = x/norm;
  filt_pose.pose.orientation.y = y/norm;
  filt_pose.pose.orientation.z = z/norm;

  return filt_pose;
}

std::map<int, AprilTagDescription> AprilTagDetector::parse_tag_descriptions(XmlRpc::XmlRpcValue& tag_descriptions)
{
  std::map<int, AprilTagDescription> descriptions;
  ROS_ASSERT(tag_descriptions.getType() == XmlRpc::XmlRpcValue::TypeArray);
  for (int32_t i = 0; i < tag_descriptions.size(); ++i)
  {
    XmlRpc::XmlRpcValue &tag_description = tag_descriptions[i];
    ROS_ASSERT(tag_description.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    ROS_ASSERT(tag_description["id"].getType() == XmlRpc::XmlRpcValue::TypeInt);
    ROS_ASSERT(tag_description["size"].getType() == XmlRpc::XmlRpcValue::TypeDouble);

    int id = (int)tag_description["id"];
    double size = (double)tag_description["size"];

    std::string frame_name;
    if(tag_description.hasMember("frame_id")){
      ROS_ASSERT(tag_description["frame_id"].getType() == XmlRpc::XmlRpcValue::TypeString);
      frame_name = (std::string)tag_description["frame_id"];
    }
    else{
      std::stringstream frame_name_stream;
      frame_name_stream << "tag_" << id;
      frame_name = frame_name_stream.str();
    }
    AprilTagDescription description(id, size, frame_name);
    ROS_INFO_STREAM("Loaded tag config: "<<id<<", size: "<<size<<", frame_name: "<<frame_name);
    descriptions.insert(std::make_pair(id, description));
  }
  return descriptions;
}


}
