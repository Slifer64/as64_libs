#include <misc_lib/joint_state_publisher.h>

namespace as64_
{

namespace misc_
{

JointStatePublisher::JointStatePublisher()
{
  run = false;
}

JointStatePublisher::~JointStatePublisher()
{
  stop();
}

void JointStatePublisher::start()
{
  if (run == true) return;

  joint_state_pub = nh.advertise<sensor_msgs::JointState>(pub_topic, 1);
  run = true;
  run_thread = std::thread(&JointStatePublisher::publishLoop, this);
}

void JointStatePublisher::stop()
{
  run = false;
  if (run_thread.joinable()) run_thread.join();
  joint_state_pub.shutdown();
}

void JointStatePublisher::addFun(void (*fun)(sensor_msgs::JointState &))
{
  std::unique_lock<std::mutex> lck(mtx);
  add_joint_state_funs.push_back( std::bind(fun, std::placeholders::_1) );
}


void JointStatePublisher::setPublishCycle(double Ts)
{
  this->Ts = Ts*1e9;
}

void JointStatePublisher::setPublishTopic(const std::string &pub_topic)
{
  this->pub_topic = pub_topic;
}


void JointStatePublisher::publishLoop()
{
  while (run)
  {
    std::unique_lock<std::mutex> lck(mtx);

    sensor_msgs::JointState joint_state_msg;
    joint_state_msg.header.stamp = ros::Time::now();
    for (int i=0; i<add_joint_state_funs.size(); i++) (add_joint_state_funs[i])(joint_state_msg);

    joint_state_pub.publish(joint_state_msg);

    lck.unlock();

    std::this_thread::sleep_for(std::chrono::nanoseconds(Ts));
  }
}

} // namespace misc_

} // namespace as64_
