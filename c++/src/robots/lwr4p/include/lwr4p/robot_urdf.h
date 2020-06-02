#ifndef ROBOT_URDF_ARM_H
#define ROBOT_URDF_ARM_H

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
#include <lwr4p/utils.h>

namespace as64_
{

namespace lwr4p_
{

class RobotUrdf
{
public:
  RobotUrdf(urdf::Model &urdf_model, const std::string &base_link, const std::string &tool_link);
  RobotUrdf(const std::string &robot_desc_param, const std::string &base_link, const std::string &tool_link);
  ~RobotUrdf();

  int getNumJoints() const;

  // return using the urdf model and kdl forward-inverse kinematics solvers
  arma::vec getJointsPosition(const arma::mat &pose, const arma::vec &q0, bool *found_solution=NULL) const;
  arma::mat getTaskPose(const arma::vec &j_pos) const;
  arma::mat getJacobian(const arma::vec &j_pos) const;
  arma::mat getEEJacobian(const arma::vec &j_pos) const;

  arma::vec getLowerJointLimits() const { return arma::vec(joint_pos_lower_lim); }
  arma::vec getUpperJointLimits() const { return arma::vec(joint_pos_upper_lim); }

protected:

  void init();

  urdf::Model urdf_model;
  std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver;
  std::shared_ptr<KDL::ChainIkSolverVel_pinv> ik_vel_solver;//Inverse velocity solver
  std::shared_ptr<KDL::ChainIkSolverPos_NR> ik_solver;
  std::shared_ptr<KDL::ChainJntToJacSolver> jac_solver;
  KDL::Chain chain;

  ros::NodeHandle node;

  std::vector<std::string> joint_names;
  std::vector<double> joint_pos_lower_lim;
  std::vector<double> joint_pos_upper_lim;
  std::vector<double> joint_vel_lim;
  std::vector<double> effort_lim;

  std::string base_link_name;
  std::string tool_link_name;
};

}; // namespace lwr4p_

}; // namespace as64_

#endif // ROBOT_URDF_ARM_H
