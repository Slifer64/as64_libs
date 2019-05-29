#ifndef BHAND_ROBOT_HAND_H
#define BHAND_ROBOT_HAND_H

#include <iostream>
#include <cstdlib>
#include <fstream>
#include <vector>
#include <memory>
#include <functional>
#include <map>
#include <string>
#include <thread>
#include <mutex>
#include <chrono>
#include <condition_variable>

#include <urdf/model.h>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>

#include <armadillo>
#include <bhand/utils.h>
#include <bhand/kinematic_chain.h>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

namespace as64_
{

namespace bhand_
{

class RobotHand
{
public:
  RobotHand();
  RobotHand(urdf::Model &urdf_model, const std::string &base_link, const std::vector<std::string> &tool_link, double ctrl_cycle);
  RobotHand(const std::string &robot_desc_param, const std::string &base_link, const std::vector<std::string> &tool_link, double ctrl_cycle);
  ~RobotHand();

  virtual bool isOk() const;
  virtual void enable();
  std::string getErrMsg() const;
  bhand_::Mode getMode() const;
  double getCtrlCycle() const;
  int getNumJoints() const;
  bool setJointsTrajectory(const arma::vec &j_targ, double duration);

  virtual void update() = 0;
  virtual void setMode(const bhand_::Mode &m) = 0;
  virtual void setJointsPosition(const arma::vec &j_pos) = 0;
  virtual void setJointsVelocity(const arma::vec &j_vel) = 0;
  virtual void setTaskVelocity(bhand_::ChainName &chain_name, const arma::vec &task_vel) = 0;

  virtual arma::vec getJointsPosition() const;
  virtual arma::vec getJointsVelocity() const;
  virtual arma::mat getTaskPose(bhand_::ChainName &chain_name) const;
  virtual arma::vec getTaskPosition(bhand_::ChainName &chain_name) const;
  virtual arma::vec getTaskOrientation(bhand_::ChainName &chain_name) const;
  virtual arma::mat getJacobian(bhand_::ChainName &chain_name) const;
  virtual arma::mat getEEJacobian(bhand_::ChainName &chain_name) const;
  virtual arma::vec getJointsTorque() const = 0;

  arma::mat getTaskPose(bhand_::ChainName &chain_name, const arma::vec &j_pos) const;
  arma::mat getJacobian(bhand_::ChainName &chain_name, const arma::vec &j_pos) const;

  void setJointLimitCheck(bool check);

  void addJointState(sensor_msgs::JointState &joint_state_msg);

protected:

  virtual void stop() = 0;
  virtual void protectiveStop() = 0;

  void setJointsPositionHelper(const arma::vec &j_pos);
  void setJointsVelocityHelper(const arma::vec &j_vel);
  void setTaskVelocityHelper(bhand_::ChainName &chain_name, const arma::vec &task_vel);

  void init();

  Mode mode;
  std::map<bhand_::Mode, std::string> mode_name;

  int N_JOINTS;

  std::mutex robot_state_mtx;

  urdf::Model urdf_model;

  ros::NodeHandle node;

  std::vector<std::string> joint_names;
  std::vector<double> joint_pos_lower_lim;
  std::vector<double> joint_pos_upper_lim;
  std::vector<double> joint_vel_lim;
  std::vector<double> effort_lim;

  int N_fingers;
  std::string base_link_name;
  std::vector<std::string> tool_link_name;
  std::vector<std::shared_ptr<KinematicChain>> fingers;

  double ctrl_cycle;
  arma::vec prev_joint_pos;
  arma::vec joint_pos;

  bool check_limits;
  bool check_singularity;

  std::string err_msg;
  bool checkJointPosLimits(const arma::vec &j_pos);
  bool checkJointVelLimits(const arma::vec &dj_pos);
  bool checkSingularity();

  std::string getModeName(Mode mode) const;
};

}; // namespace bhand_

}; // namespace as64_

#endif // BHAND_ROBOT_HAND_H
