#ifndef LWR4P_TEST_UTILS_H
#define LWR4P_TEST_UTILS_H

#include <iostream>
#include <cstdlib>
#include <chrono>
#include <memory>
#include <thread>
#include <vector>
#include <ros/ros.h>
#include <ros/package.h>
#include <armadillo>

#include <lwr4p/robot_arm.h>
#include <lwr4p/sim_robot.h>
#include <lwr4p/robot.h>
#include <misc_lib/joint_state_publisher.h>

using namespace as64_;

struct ExecArgs
{
  ExecArgs() {}
  ExecArgs(const arma::vec &q1, const arma::vec &q2, double total_time, lwr4p_::RobotArm *robot)
  {
    this->q1 = q1;
    this->q2 = q2;
    this->total_time = total_time;
    this->robot = robot;
  }
  arma::vec q1;
  arma::vec q2;
  double total_time;
  lwr4p_::RobotArm *robot;
};

void PRINT_INFO_MSG(const std::string &msg);
void PRINT_ERR_MSG(const std::string &msg);

void parseArgs(std::string &robot_descr_name, std::vector<std::string> &base_link,
  std::vector<std::string> &tool_link, std::vector<double> &ctrl_cycle,
  std::vector<arma::vec>  &q1, std::vector<arma::vec>  &q2,
  std::vector<double> &time_duration, std::vector<bool> &use_sim);

void jointsTrajectory(const arma::vec &qT, double total_time, lwr4p_::RobotArm *robot);
void jointPositionControl(const arma::vec &qT, double total_time, lwr4p_::RobotArm *robot);
void jointVelocityControl(const arma::vec &qT, double total_time, lwr4p_::RobotArm *robot);
void cartesianVelocityControl(const arma::vec &qT, double total_time, lwr4p_::RobotArm *robot);
void taskTrajectory(const arma::vec &qT, double total_time, lwr4p_::RobotArm *robot);
void freedrive(lwr4p_::RobotArm *robot);

void robotRun(ExecArgs *args);

#endif // LWR4P_TEST_UTILS_H
