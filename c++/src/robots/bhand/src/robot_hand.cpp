#include <bhand/robot_hand.h>

#include <stdexcept>
#include <sstream>
#include <chrono>
#include <map>
#include <stack>

#include <ros/package.h>
#include <kdl_parser/kdl_parser.hpp>

namespace as64_
{

namespace bhand_
{

RobotHand::RobotHand()
{
  std::string robot_description_name;
  if (!node.getParam("/bhand_robot/robot_description_name",robot_description_name))
  {
    throw std::runtime_error("[RobotHand Error]: Failed to load parameter \"/bhand_robot/robot_description_name\" ...\n");
  }

  if (!node.getParam("/bhand_robot/base_frame",base_link_name))
  {
    throw std::runtime_error("[RobotHand Error]: Failed to load parameter \"/bhand_robot/base_frame\" ...\n");
  }

  if (!node.getParam("/bhand_robot/tool_frame",tool_link_name))
  {
    throw std::runtime_error("[RobotHand Error]: Failed to load parameter \"/bhand_robot/tool_frame\" ...\n");
  }

  if (!node.getParam("/bhand_robot/ctrl_cycle",ctrl_cycle))
  {
    ctrl_cycle = 0.01;
  }

  if (!node.getParam("/bhand_robot/check_limits",check_limits))
  {
    check_limits = false;
  }

  if (!node.getParam("/bhand_robot/check_singularity",check_singularity))
  {
    check_singularity = false;
  }

  if (!urdf_model.initParam(robot_description_name.c_str()))
  {
    throw std::ios_base::failure("[RobotHand Error]: Couldn't load urdf model from \"" + robot_description_name + "\"...\n");
  }

  init();
}

RobotHand::RobotHand(urdf::Model &urdf_model, const std::string &base_link, const std::vector<std::string> &tool_link, double ctrl_cycle)
{
  this->urdf_model = urdf_model;
  this->base_link_name = base_link;
  this->tool_link_name = tool_link;
  this->ctrl_cycle = ctrl_cycle;
  this->check_limits = false;
  this->check_singularity = false;

  init();
}

RobotHand::RobotHand(const std::string &robot_desc_param, const std::string &base_link, const std::vector<std::string> &tool_link, double ctrl_cycle)
{
  if (!urdf_model.initParam(robot_desc_param.c_str()))
  {
    throw std::ios_base::failure("Couldn't load urdf model from \"" + robot_desc_param + "\"...\n");
  }

  this->base_link_name = base_link;
  this->tool_link_name = tool_link;
  this->ctrl_cycle = ctrl_cycle;
  this->check_limits = false;
  this->check_singularity = false;

  init();
}

RobotHand::~RobotHand() {}

void RobotHand::init()
{
  mode_name[bhand_::IDLE] = "IDLE";
  mode_name[bhand_::FREEDRIVE] = "FREEDRIVE";
  mode_name[bhand_::JOINT_POS_CONTROL] = "JOINT_POS_CONTROL";
  mode_name[bhand_::JOINT_VEL_CONTROL] = "JOINT_VEL_CONTROL";
  mode_name[bhand_::CART_VEL_CONTROL] = "CART_VEL_CONTROL";
  mode_name[bhand_::PROTECTIVE_STOP] = "PROTECTIVE_STOP";

  mode = bhand_::Mode::IDLE;

  N_fingers = tool_link_name.size();

  fingers.resize(N_fingers);
  for (int i=0; i<N_fingers; i++)
    fingers[i].reset(new KinematicChain(urdf_model, base_link_name, tool_link_name[i]));

  for (int i=0; i<N_fingers; i++)
  {
    for (int j=0; j<fingers[i]->joint_names.size(); j++) joint_names.push_back(fingers[i]->joint_names[j]);
    for (int j=0; j<fingers[i]->joint_pos_lower_lim.size(); j++) joint_pos_lower_lim.push_back(fingers[i]->joint_pos_lower_lim[j]);
    for (int j=0; j<fingers[i]->joint_pos_upper_lim.size(); j++) joint_pos_upper_lim.push_back(fingers[i]->joint_pos_upper_lim[j]);
    for (int j=0; j<fingers[i]->joint_vel_lim.size(); j++) joint_vel_lim.push_back(fingers[i]->joint_vel_lim[j]);
    for (int j=0; j<fingers[i]->effort_lim.size(); j++) effort_lim.push_back(fingers[i]->effort_lim[j]);
  }

  // preallocate space for all states
  N_JOINTS = joint_names.size();
  joint_pos.zeros(N_JOINTS);
  prev_joint_pos.zeros(N_JOINTS);

  N_fingers = tool_link_name.size();
  fingers.resize(N_fingers);
}

bool RobotHand::isOk() const
{
  return getMode()!=bhand_::Mode::PROTECTIVE_STOP;
}

void RobotHand::setJointLimitCheck(bool check)
{
  check_limits = check;
}

void RobotHand::enable()
{
  setMode(bhand_::Mode::IDLE);
  joint_pos = getJointsPosition();
  prev_joint_pos = joint_pos;
  update();
}

std::string RobotHand::getErrMsg() const
{
  return err_msg;
}

void RobotHand::addJointState(sensor_msgs::JointState &joint_state_msg)
{
  std::unique_lock<std::mutex> lck(robot_state_mtx);

  arma::vec j_pos = getJointsPosition();
  arma::vec j_vel = getJointsVelocity();

  for (int i=0;i<N_JOINTS;i++)
  {
    joint_state_msg.name.push_back(joint_names[i]);
    joint_state_msg.position.push_back(j_pos(i));
    joint_state_msg.velocity.push_back(j_vel(i));
    joint_state_msg.effort.push_back(0.0);
  }
}

double RobotHand::getCtrlCycle() const
{
  return ctrl_cycle;
}

bhand_::Mode RobotHand::getMode() const
{
  return mode;
}

std::string RobotHand::getModeName(Mode mode) const
{
  return (mode_name.find(mode))->second;
}

int RobotHand::getNumJoints() const
{
  return N_JOINTS;
}

bool RobotHand::setJointsTrajectory(const arma::vec &j_targ, double duration)
{
  // keep last known robot mode
  bhand_::Mode prev_mode = getMode();

  // initialize position
  arma::vec q0 = getJointsPosition();
  arma::vec qref = q0;

  // initalize time
  double t = 0.0;
  double Ts = getCtrlCycle();

  // start conttroller
  setMode(bhand_::Mode::JOINT_POS_CONTROL);
  while (isOk() && getMode()!=bhand_::Mode::IDLE && t<duration)
  {
    update();
    t += Ts;
    qref = get5thOrder(t, q0, j_targ, duration)[0];
    setJointsPosition(qref);
  }

  bool reached_target = t>=duration;

  if (isOk() && getMode()==bhand_::Mode::JOINT_POS_CONTROL) setMode(prev_mode);

  return reached_target;
}

void RobotHand::setJointsPositionHelper(const arma::vec &j_pos)
{
  arma::vec current_j_pos = joint_pos; // getJointsPosition();
  arma::vec dj_pos = (j_pos - current_j_pos) / ctrl_cycle;

  if (check_limits)
  {
    if (!checkJointPosLimits(j_pos)) return;
    if (!checkJointVelLimits(dj_pos)) return;
  }

  std::unique_lock<std::mutex> lck(robot_state_mtx);

  prev_joint_pos = current_j_pos;
  joint_pos = j_pos;
}

void RobotHand::setJointsVelocityHelper(const arma::vec &j_vel)
{
  setJointsPositionHelper(getJointsPosition() + j_vel*ctrl_cycle);
}

void RobotHand::setTaskVelocityHelper(bhand_::ChainName &chain_name, const arma::vec &task_vel)
{
  setJointsVelocityHelper(arma::solve(getJacobian(chain_name),task_vel));
}

arma::vec RobotHand::getJointsPosition() const
{
  // std::unique_lock<std::mutex> lck(robot_state_mtx);
  return joint_pos;
}

arma::vec RobotHand::getJointsVelocity() const
{
  // std::unique_lock<std::mutex> lck(robot_state_mtx);
  return (joint_pos - prev_joint_pos)/getCtrlCycle();
}

arma::mat RobotHand::getTaskPose(bhand_::ChainName &chain_name) const
{
  // std::unique_lock<std::mutex> lck(robot_state_mtx);
  return getTaskPose(chain_name, getJointsPosition());
}

arma::mat RobotHand::getTaskPose(bhand_::ChainName &chain_name, const arma::vec &j_pos) const
{
  return fingers[(int)chain_name]->getTaskPose(j_pos);
}

arma::vec RobotHand::getTaskPosition(bhand_::ChainName &chain_name) const
{
  // std::unique_lock<std::mutex> lck(robot_state_mtx);
  return getTaskPose(chain_name).submat(0,3,2,3);
}

arma::vec RobotHand::getTaskOrientation(bhand_::ChainName &chain_name) const
{
  // std::unique_lock<std::mutex> lck(robot_state_mtx);
  arma::mat R = getTaskPose(chain_name).submat(0,0,2,2);
  arma::vec quat = rotm2quat(R);
  return quat;
}

arma::mat RobotHand::getJacobian(bhand_::ChainName &chain_name) const
{
  // std::unique_lock<std::mutex> lck(robot_state_mtx);
  return getJacobian(chain_name, getJointsPosition());
}

arma::mat RobotHand::getJacobian(bhand_::ChainName &chain_name, const arma::vec &j_pos) const
{
  return fingers[(int)chain_name]->getJacobian(j_pos);
}

arma::mat RobotHand::getEEJacobian(bhand_::ChainName &chain_name) const
{
  // std::unique_lock<std::mutex> lck(robot_state_mtx);
  arma::mat R = getTaskPose(chain_name).submat(0,0,2,2);
  arma::mat Jrobot = getJacobian(chain_name);
  int n_joints = fingers[(int)chain_name]->getNumJoints();
  arma::mat Jee(6, n_joints);
  Jee.submat(0,0,2,n_joints-1) = R * Jrobot.submat(0,0,2,n_joints-1);
  Jee.submat(3,0,5,n_joints-1) = R * Jrobot.submat(3,0,5,n_joints-1);

  return Jee;
}

arma::vec RobotHand::getJointsTorque() const
{
  // std::unique_lock<std::mutex> lck(robot_state_mtx);
  return arma::vec().zeros(N_JOINTS);
}

bool RobotHand::checkJointPosLimits(const arma::vec &j_pos)
{
  for (int i=0;i<N_JOINTS;i++)
  {
    if (j_pos(i)>joint_pos_upper_lim[i] || j_pos(i)<joint_pos_lower_lim[i])
    {
      std::ostringstream out;
      out << joint_names[i] << ": position limit reached: " << j_pos(i) << " rad";
      err_msg = out.str();
      // print_err_msg(err_msg);
      setMode(bhand_::Mode::PROTECTIVE_STOP);
      return false;
    }
  }

  return true;
}

bool RobotHand::checkJointVelLimits(const arma::vec &dj_pos)
{
  for (int i=0;i<N_JOINTS;i++)
  {
    if (std::fabs(dj_pos(i))>joint_vel_lim[i])
    {
      std::ostringstream out;
      out << joint_names[i] << ": velocity limit reached: " << dj_pos(i) << " rad/s";
      err_msg = out.str();
      // print_err_msg(err_msg);
      setMode(bhand_::Mode::PROTECTIVE_STOP);
      return false;
    }
  }

  return true;
}

}; // namespace bhand_

}; // namespace as64_
