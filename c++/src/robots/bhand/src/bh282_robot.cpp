#include <bhand/bh282_robot.h>

namespace as64_
{

namespace bhand_
{

Bh282Robot::Bh282Robot(urdf::Model &urdf_model, const std::string &base_link,
                      const std::vector<std::string> &tool_link, double ctrl_cycle)
                      :RobotHand(urdf_model, base_link, tool_link, ctrl_cycle)
{
  initRobot();
}

Bh282Robot::Bh282Robot(const std::string &robot_desc_param, const std::string &base_link,
                      const std::vector<std::string> &tool_link, double ctrl_cycle)
                      :RobotHand(robot_desc_param, base_link, tool_link, ctrl_cycle)
{
  initRobot();
}

Bh282Robot::Bh282Robot()
{
  initRobot();
}

Bh282Robot::~Bh282Robot()
{
  hw_i.terminate();
}

void Bh282Robot::initRobot()
{
  hw_i.initialize("BH8-280", true);
  k_click = 10.0;
}

void Bh282Robot::update()
{
  hw_i.RTUpdate();

  checkJointPosDeviationError();

  if (getMode() == bhand_::FREEDRIVE) updateFreedrive();
}

void Bh282Robot::setMode(const bhand_::Mode &m)
{
  if (getMode() == m) return;

  if (m == bhand_::PROTECTIVE_STOP)
  {
    protectiveStop();
    return;
  }

  stop();
  if (!isOk()) return;

  switch (m)
  {
    case bhand_::IDLE:
      hw_i.setMode(BhandHWInterface::IDLE);
      break;
    case bhand_::FREEDRIVE:
      initFreedrive();
      hw_i.setMode(BhandHWInterface::JOINT_VEL_CONTROL);
      break;
    case JOINT_POS_CONTROL:
    case JOINT_VEL_CONTROL:
    case CART_VEL_CONTROL:
      hw_i.setMode(BhandHWInterface::JOINT_VEL_CONTROL);
      break;
  }

  mode = m;
}

void Bh282Robot::setJointsPosition(const arma::vec &j_pos)
{
  if (getMode() != bhand_::Mode::JOINT_POS_CONTROL)
  {
    print_warn_msg("[bhand::BH282Robot::setJointsPosition]: Cannot set joints position. Current mode is \"" + getModeName(getMode()) + "\"\n");
    return;
  }

  setJointsPositionHelper(j_pos);

  if (!isOk()) hw_i.stop();
  else velCmdToHw();
}

void Bh282Robot::setJointsVelocity(const arma::vec &j_vel)
{
  if (getMode() != bhand_::Mode::JOINT_VEL_CONTROL)
  {
    print_warn_msg("[bhand::BH282Robot::setJointsVelocity]: Cannot set joints velocity. Current mode is \"" + getModeName(getMode()) + "\"\n");
    return;
  }

  setJointsVelocityHelper(j_vel);

  if (!isOk()) hw_i.stop();
  else velCmdToHw();
}

void Bh282Robot::setTaskVelocity(bhand_::ChainName &chain_name, const arma::vec &task_vel)
{
  if (getMode() != bhand_::Mode::CART_VEL_CONTROL)
  {
    print_warn_msg("[bhand::BH282Robot::setTaskVelocity]: Cannot set Cartesian velocity. Current mode is \"" + getModeName(getMode()) + "\"\n");
    return;
  }

  setTaskVelocityHelper(chain_name, task_vel);

  if (!isOk()) hw_i.stop();
  else velCmdToHw();
}

void Bh282Robot::velCmdToHw()
{
  int N_joints = getNumJoints();
  arma::vec q_actual(N_joints);
  // get the actual position from the robot
  for (int i=0;i<4;i++) q_actual(i) = hw_i.getJointPosition(i);

  arma::vec j_vel_cmd = (joint_pos - prev_joint_pos) / getCtrlCycle() + k_click*(joint_pos-q_actual);
  for (int i=0; i<N_joints; i++) hw_i.setJointVelocity(j_vel_cmd(i), i);
}

arma::vec Bh282Robot::getJointPosFromHW()
{
  arma::vec actual_joint_pos(4);
  // get the actual position from the robot
  for (int i=0;i<4;i++) actual_joint_pos(i) = hw_i.getJointPosition(i);

  return actual_joint_pos;
}

void Bh282Robot::checkJointPosDeviationError()
{
  arma::vec actual_joint_pos = getJointPosFromHW();
  arma::vec j_pos = getJointsPosition();
  if (arma::norm(j_pos-actual_joint_pos) > 5)
  {
    setMode(bhand_::PROTECTIVE_STOP);
    bhand_::print_err_msg("[Bh282Robot ERROR]: Joint position deviation limit exceeded!\n");
  }
}

void Bh282Robot::initFreedrive()
{
  q_a = arma::vec().zeros(4);
  dq_a = arma::vec().zeros(4);
  ddq_a = arma::vec().zeros(4);
  p_a = 10;
  t_s = 1.0;
}

void Bh282Robot::updateFreedrive()
{
  double d_a = 0.3;
  double k_a = 0.0;
  t_s = 1.5;

  arma::vec torq = getJointsTorque();

  // arma::vec dddq_a = -3*p_a*ddq_a -3*std::pow(p_a,2)*dq_a -std::pow(p_a,3)*q_a + t_s*torq;
  ddq_a = -d_a*dq_a -k_a*q_a + t_s*torq;

  std::cout << "torq = " << torq.t() << "\n";
  // std::cout << "q_a = " << q_a.t() << "\n";
  std::cout << "dq_a = " << dq_a.t() << "\n";
  // std::cout << "ddq_a = " << ddq_a.t() << "\n";

  setJointsVelocity(dq_a);

  double Ts = getCtrlCycle();
  q_a = q_a + dq_a*Ts;
  dq_a = dq_a + ddq_a*Ts;
  // ddq_a = ddq_a + dddq_a*Ts;
}

arma::vec Bh282Robot::getJointsTorque() const
{
  int N_joints = getNumJoints();
  arma::vec torq(N_joints);

  auto self = const_cast<Bh282Robot *>(this);

  torq(0) = 0;
  for (int i=1;i<N_joints; i++) torq(i) = self->hw_i.getJointTorque(i);

  return torq;
}

void Bh282Robot::stop()
{
  if (getMode() == bhand_::IDLE) return;

  hw_i.stop();

  update();
  arma::vec q_current = getJointsPosition();
  setJointsPositionHelper(q_current);
  prev_joint_pos = getJointsPosition();
  if (isOk()) mode = bhand_::IDLE;
}

void Bh282Robot::protectiveStop()
{
  if (getMode() == bhand_::PROTECTIVE_STOP) return;

  hw_i.stop();

  joint_pos = getJointsPosition();
  prev_joint_pos = joint_pos;
  mode = bhand_::Mode::PROTECTIVE_STOP;
  print_warn_msg("Mode changed to \"" + getModeName(getMode()) + "\"\n");
}

}; // namespace bhand_

}; // namespace as64_
