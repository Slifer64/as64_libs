#include <lwr4p/sim_robot.h>

namespace as64_
{

namespace lwr4p_
{

SimRobot::SimRobot()
{
  initSimRobot();
}

SimRobot::SimRobot(urdf::Model &urdf_model, const std::string &base_link, const std::string &tool_link,
                   double ctrl_cycle): RobotArm(urdf_model, base_link, tool_link, ctrl_cycle)
{
  initSimRobot();
}

SimRobot::SimRobot(const std::string &robot_desc_param, const std::string &base_link, const std::string &tool_link,
                   double ctrl_cycle): RobotArm(robot_desc_param, base_link, tool_link, ctrl_cycle)
{
  initSimRobot();
}

void SimRobot::initSimRobot()
{
  // jState_pub = node.advertise<sensor_msgs::JointState>(pub_state_topic, 1);
  jState_sub = node.subscribe("/joint_states", 1, &SimRobot::jStateSubCallback, this);

  setGetExternalWrenchFun(&SimRobot::getExternalWrenchImplementation, this);

  update_time = (unsigned long)(getCtrlCycle()*1e9);
  timer.start();
}

void SimRobot::setMode(const lwr4p_::Mode &m)
{
  if (getMode() == m) return;

  if (m == PROTECTIVE_STOP)
  {
    protectiveStop();
    return;
  }

  stop();
  if (!isOk()) return;

  switch (m) {
    case lwr4p_::Mode::IDLE:
    case lwr4p_::Mode::FREEDRIVE:
    case lwr4p_::Mode::JOINT_POS_CONTROL:
    case lwr4p_::Mode::JOINT_VEL_CONTROL:
    case lwr4p_::Mode::CART_VEL_CONTROL:
      break;
    case lwr4p_::Mode::JOINT_TORQUE_CONTROL:
    case lwr4p_::Mode::CART_IMPEDANCE_CONTROL:
      throw std::runtime_error("[lwr4+::SimRobot::setMode]: \"CART_IMPEDANCE_CONTROL\" is not supported.");
  }

  mode = m;
}

void SimRobot::update()
{
  // std::unique_lock<std::mutex> lck(robot_state_mtx);

  ros::spinOnce();

  unsigned long elaps_time = timer.elapsedNanoSec();
  if (elaps_time < update_time)
  {
    std::this_thread::sleep_for(std::chrono::nanoseconds(update_time-elaps_time));
  }
  timer.start(); // restart time cycle
}

void SimRobot::setJointsPosition(const arma::vec &j_pos)
{
  if (getMode() != lwr4p_::Mode::JOINT_POS_CONTROL)
  {
    print_warn_msg("[lwr4+::SimRobot::setJointsPosition]: Cannot set joints position. Current mode is \"" + getModeName(getMode()) + "\"\n");
    return;
  }

  err_msg = ""; // clear the error msg
  setJointsPositionHelper(j_pos);
}

void SimRobot::setJointsVelocity(const arma::vec &j_vel)
{

  if (getMode() != lwr4p_::Mode::JOINT_VEL_CONTROL)
  {
    print_warn_msg("[lwr4+::SimRobot::setJointsVelocity]: Cannot set joints velocity. Current mode is \"" + getModeName(getMode()) + "\"\n");
    return;
  }

  err_msg = ""; // clear the error msg
  setJointsVelocityHelper(j_vel);
}

void SimRobot::setTaskVelocity(const arma::vec &task_vel)
{
  err_msg = ""; // clear the error msg
  if (getMode() != lwr4p_::Mode::CART_VEL_CONTROL)
  {
    print_warn_msg("[lwr4+::SimRobot::setTaskVelocity]: Cannot set task velocity. Current mode is \"" + getModeName(getMode()) + "\"\n");
    return;
  }
  setTaskVelocityHelper(task_vel);
}

void SimRobot::setJointsTorque(const arma::vec &j_torques)
{
  throw std::runtime_error("[ERROR]: SimRobot::setJointsTorque is not supported.");
}

void SimRobot::setTaskPose(const arma::mat &task_pose)
{
  throw std::runtime_error("[ERROR]: SimRobot::setTaskPose is not supported.");
}

void SimRobot::setWrench(const arma::vec &wrench)
{
  throw std::runtime_error("[ERROR]: SimRobot::setWrench is not supported.");
}

void SimRobot::setCartStiffness(const arma::vec &cart_stiff)
{
  this->cart_stiff = cart_stiff;
}

void SimRobot::setCartDamping(const arma::vec &cart_damp)
{
  this->cart_damp = cart_damp;
}

void SimRobot::stop()
{
  if (getMode() == lwr4p_::IDLE) return;

  update();
  arma::vec q_current = getJointsPosition();
  setJointsPositionHelper(q_current);
  prev_joint_pos = getJointsPosition();
  if (isOk()) mode = lwr4p_::IDLE;
}

void SimRobot::protectiveStop()
{
  if (getMode() == lwr4p_::PROTECTIVE_STOP) return;

  joint_pos = getJointsPosition();
  prev_joint_pos = joint_pos;
  mode = lwr4p_::Mode::PROTECTIVE_STOP;
  print_warn_msg("Mode changed to \"" + getModeName(getMode()) + "\"\n");
}

void SimRobot::jStateSubCallback(const sensor_msgs::JointState::ConstPtr& j_state)
{
  if (getMode() != lwr4p_::Mode::FREEDRIVE) return;

  std::map<std::string, double> j_map;

  for (int i=0;i<j_state->name.size();i++)
    j_map.insert( std::pair<std::string,double>(j_state->name[i], j_state->position[i]) );

  for (int i=0;i<joint_names.size();i++)
    joint_pos[i] = j_map[joint_names[i]];
  // joint_vel = j_state->velocity;
  // joint_torques = j_state->effort;
}

arma::vec SimRobot::getExternalWrenchImplementation()
{
  return arma::vec().zeros(6);
}

}; // namespace lwr4p_

}; // namespace as64_
