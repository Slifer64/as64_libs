#include <misc_lib/wrench_reader.h>

namespace as64_
{

namespace misc_
{

WrenchReader::WrenchReader(const std::string &read_wrench_topic)
{
  sub = nh.subscribe(read_wrench_topic.c_str(), 1, &WrenchReader::readWrenchCallback, this);

  wrench = arma::vec().zeros(6);
}

arma::vec WrenchReader::getWrench()
{
  ros::spinOnce(); // update wrench from "readWrenchCallback"
  return wrench;
}

void WrenchReader::readWrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr& wrench_ptr)
{
  wrench(0) = wrench_ptr->wrench.force.x;
  wrench(1) = wrench_ptr->wrench.force.y;
  wrench(2) = wrench_ptr->wrench.force.z;
  wrench(3) = wrench_ptr->wrench.torque.x;
  wrench(4) = wrench_ptr->wrench.torque.y;
  wrench(5) = wrench_ptr->wrench.torque.z;
}

} // namespace misc_

} // namespace as64_
