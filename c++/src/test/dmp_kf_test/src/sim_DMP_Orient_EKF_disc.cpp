#include <ros/ros.h>
#include <ros/package.h>
#include <memory>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <exception>

#include <io_lib/io_utils.h>
#include <dmp_lib/DMP/DMP_eo.h>
#include <dmp_lib/GatingFunction/LinGatingFunction.h>
#include <dmp_lib/GatingFunction/ExpGatingFunction.h>
#include <dmp_lib/GatingFunction/SigmoidGatingFunction.h>

#include <dmp_kf_test/utils.h>

using namespace as64_;

int main(int argc, char** argv)
{
  // ===========  Initialize the ROS node  ===============
  ros::init(argc, argv, "dmp_kf_orient_test_node");
  ros::NodeHandle nh_("~");

  // ===========  Read params  ===============


  // ===========  Load training data  ===============


  // ===========  initialize DMP  ===============


  // ===========  DMP simulation  ===============


  // ===========  write results  ===============


  // ===========  Shutdown ROS node  ==================
  ros::shutdown();

  return 0;
}
