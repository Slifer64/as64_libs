#include <bhand/bh282_sim_robot.h>

namespace as64_
{

namespace bhand_
{

Bh282SimRobot::Bh282SimRobot(urdf::Model &urdf_model, const std::string &base_link,
                      const std::vector<std::string> &tool_link, double ctrl_cycle)
                      :RobotHand(urdf_model, base_link, tool_link, ctrl_cycle)
{
  initSimRobot();
}

Bh282SimRobot::Bh282SimRobot(const std::string &robot_desc_param, const std::string &base_link,
                      const std::vector<std::string> &tool_link, double ctrl_cycle)
                      :RobotHand(robot_desc_param, base_link, tool_link, ctrl_cycle)
{
  initSimRobot();
}

Bh282SimRobot::Bh282SimRobot()
{
  initSimRobot();
}

Bh282SimRobot::~Bh282SimRobot()
{
  initSimRobot();
}

void Bh282SimRobot::initSimRobot()
{
  // jState_pub = node.advertise<sensor_msgs::JointState>(pub_state_topic, 1);
  jState_sub = node.subscribe("/joint_states", 1, &Bh282SimRobot::jStateSubCallback, this);

  update_time = (unsigned long)(getCtrlCycle()*1e9);
  timer.start();
}

void Bh282SimRobot::update()
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

void Bh282SimRobot::setMode(const bhand_::Mode &m)
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
    case bhand_::Mode::IDLE:
    case bhand_::Mode::FREEDRIVE:
    case bhand_::Mode::JOINT_POS_CONTROL:
    case bhand_::Mode::JOINT_VEL_CONTROL:
    case bhand_::Mode::CART_VEL_CONTROL:
      break;
  }

  mode = m;
}

void Bh282SimRobot::stop()
{
  if (getMode() == bhand_::IDLE) return;

  update();
  arma::vec q_current = getJointsPosition();
  setJointsPositionHelper(q_current);
  prev_joint_pos = getJointsPosition();
  if (isOk()) mode = bhand_::IDLE;
}

void Bh282SimRobot::protectiveStop()
{
  if (getMode() == bhand_::PROTECTIVE_STOP) return;

  joint_pos = getJointsPosition();
  prev_joint_pos = joint_pos;
  mode = bhand_::Mode::PROTECTIVE_STOP;
  print_warn_msg("Mode changed to \"" + getModeName(getMode()) + "\"\n");
}

void Bh282SimRobot::setJointsPosition(const arma::vec &j_pos)
{
  if (getMode() != bhand_::Mode::JOINT_POS_CONTROL)
  {
    print_warn_msg("[bhand::Bh282SimRobot::setJointsPosition]: Cannot set joints position. Current mode is \"" + getModeName(getMode()) + "\"\n");
    return;
  }

  setJointsPositionHelper(j_pos);
}

void Bh282SimRobot::setJointsVelocity(const arma::vec &j_vel)
{
  if (getMode() != bhand_::Mode::JOINT_VEL_CONTROL)
  {
    print_warn_msg("[bhand::Bh282SimRobot::setJointsVelocity]: Cannot set joints velocity. Current mode is \"" + getModeName(getMode()) + "\"\n");
    return;
  }

  setJointsVelocityHelper(j_vel);
}

void Bh282SimRobot::setTaskVelocity(bhand_::ChainName &chain_name, const arma::vec &task_vel)
{
  if (getMode() != bhand_::Mode::CART_VEL_CONTROL)
  {
    print_warn_msg("[bhand::Bh282SimRobot::setTaskVelocity]: Cannot set Cartesian velocity. Current mode is \"" + getModeName(getMode()) + "\"\n");
    return;
  }

  setTaskVelocityHelper(chain_name, task_vel);
}

arma::vec Bh282SimRobot::getJointsTorque() const
{
  int N_joints = getNumJoints();
  arma::vec torq = arma::vec().zeros(N_joints);

  return torq;
}

void Bh282SimRobot::jStateSubCallback(const sensor_msgs::JointState::ConstPtr& j_state)
{
  if (getMode() != bhand_::Mode::FREEDRIVE) return;

  std::map<std::string, double> j_map;

  for (int i=0;i<j_state->name.size();i++)
    j_map.insert( std::pair<std::string,double>(j_state->name[i], j_state->position[i]) );

  for (int i=0;i<joint_names.size();i++)
    joint_pos[i] = j_map[joint_names[i]];
  // joint_vel = j_state->velocity;
  // joint_torques = j_state->effort;
}

}; // namespace bhand_

}; // namespace as64_
