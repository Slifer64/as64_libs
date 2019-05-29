#ifndef AS64_MISC_LIB_WRENCH_READER
#define AS64_MISC_LIB_WRENCH_READER

#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <armadillo>

namespace as64_
{

namespace misc_
{

class WrenchReader
{
public:
  WrenchReader(const std::string &read_wrench_topic);
  arma::vec getWrench();

private:
  arma::vec wrench;
  ros::Subscriber sub;
  ros::NodeHandle nh;

  void readWrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr& wrench_ptr);
};

} // namespace misc_

} // namespace as64_

#endif // AS64_MISC_LIB_WRENCH_READER
