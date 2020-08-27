/**
 * Copyright (C) 2017 as64_
 */

#include <memory>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <exception>
#include <thread>
#include <mutex>
#include <chrono>

#include <armadillo>

#include <ros/ros.h>
#include <ros/package.h>

#include <io_lib/io_lib.h>
#include <ur_robot/ur_robot.h>


int main(int argc, char** argv)
{
  // ===========  Initialize the ROS node  ==================
  ros::init(argc, argv, "run_urscript");

  std::string urScript_file = "movej_example1.script";
  if (argc == 2) urScript_file = argv[1];

  // ===========  Create robot  ==================
  std::shared_ptr<ur_::Robot> robot;
  robot.reset(new ur_::Robot());

  // ===========  Launch thread for printing  ==================
  // optional...
  // robot->launch_printRobotStateThread(print_robotState_rate);

  // ===========  Load and run urscript  ==================
  std::cout << io_::bold << io_::blue << "Loading URscript file...\n" << io_::reset;
  robot->load_URScript(urScript_file);
  std::cout << io_::bold << io_::green << "Executing URscript file...\n" << io_::reset;
  robot->execute_URScript();

  // robot->stop_printRobotStateThread();

  // ===========  Shutdown ROS node  ==================
  ros::shutdown();

  return 0;
}
