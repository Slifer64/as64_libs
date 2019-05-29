#ifndef BHAND_TEST_UTILS_H
#define BHAND_TEST_UTILS_H

#include <iostream>
#include <cstdlib>
#include <chrono>
#include <memory>
#include <thread>
#include <vector>
#include <ros/ros.h>
#include <ros/package.h>
#include <armadillo>

#include <bhand/robot_hand.h>
#include <bhand/bh282_sim_robot.h>
#include <bhand/bh282_robot.h>
#include <misc_lib/joint_state_publisher.h>

using namespace as64_;

struct ExecArgs
{
  ExecArgs() {}
  ExecArgs(const arma::vec &q1, const arma::vec &q2, double total_time, bhand_::RobotHand *robot)
  {
    this->q1 = q1;
    this->q2 = q2;
    this->total_time = total_time;
    this->robot = robot;
  }
  arma::vec q1;
  arma::vec q2;
  double total_time;
  bhand_::RobotHand *robot;
};

void PRINT_INFO_MSG(const std::string &msg);
void PRINT_ERR_MSG(const std::string &msg);

void parseArgs(std::string &robot_descr_name, std::vector<std::string> &base_link,
  std::vector<std::vector<std::string>> &tool_link, std::vector<double> &ctrl_cycle,
  std::vector<arma::vec>  &q1, std::vector<arma::vec>  &q2,
  std::vector<double> &time_duration, std::vector<bool> &use_sim);

void jointsTrajectory(const arma::vec &qT, double total_time, bhand_::RobotHand *robot);
void jointPositionControl(const arma::vec &qT, double total_time, bhand_::RobotHand *robot);
void jointVelocityControl(const arma::vec &qT, double total_time, bhand_::RobotHand *robot);
void freedrive(bhand_::RobotHand *robot);

void robotRun(ExecArgs *args);

#endif // BHAND_TEST_UTILS_H
