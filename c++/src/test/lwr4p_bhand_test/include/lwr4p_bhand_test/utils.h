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
#include <bhand/robot_hand.h>
#include <bhand/bh282_sim_robot.h>
#include <bhand/bh282_robot.h>
#include <misc_lib/joint_state_publisher.h>

using namespace as64_;

struct LwrExecArgs
{
  LwrExecArgs() {}
  LwrExecArgs(const arma::vec &q1, const arma::vec &q2, double total_time, lwr4p_::RobotArm *robot, const bool *run)
  {
    this->q1 = q1;
    this->q2 = q2;
    this->total_time = total_time;
    this->robot = robot;
    this->run = run;
  }
  arma::vec q1;
  arma::vec q2;
  double total_time;
  lwr4p_::RobotArm *robot;
  const bool *run;
};

struct BhExecArgs
{
  BhExecArgs() {}
  BhExecArgs(const arma::vec &q1, const arma::vec &q2, double total_time, bhand_::RobotHand *robot, const bool *run)
  {
    this->q1 = q1;
    this->q2 = q2;
    this->total_time = total_time;
    this->robot = robot;
    this->run = run;
  }
  arma::vec q1;
  arma::vec q2;
  double total_time;
  bhand_::RobotHand *robot;
  const bool *run;
};

void PRINT_INFO_MSG(const std::string &msg);
void PRINT_ERR_MSG(const std::string &msg);

void lwrRobotRun(LwrExecArgs *args);

void bhRobotRun(BhExecArgs *args);

#endif // LWR4P_TEST_UTILS_H
