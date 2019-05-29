#ifndef BH282_ROBOT_H
#define BH282_ROBOT_H

#include <bhand/robot_hand.h>
#include <bhand/bhand_API/BhandHWInterface.h>

namespace as64_
{

namespace bhand_
{

class Bh282Robot : public RobotHand
{
public:
  Bh282Robot();
  Bh282Robot(urdf::Model &urdf_model, const std::string &base_link, const std::vector<std::string> &tool_link, double ctrl_cycle);
  Bh282Robot(const std::string &robot_desc_param, const std::string &base_link, const std::vector<std::string> &tool_link, double ctrl_cycle);
  ~Bh282Robot();

  void update();
  void setMode(const bhand_::Mode &m);

  void setJointsPosition(const arma::vec &j_pos);
  void setJointsVelocity(const arma::vec &j_vel);
  void setTaskVelocity(bhand_::ChainName &chain_name, const arma::vec &task_vel);

  arma::vec getJointsTorque() const;

private:
  BhandHWInterface hw_i;

  void stop();
  void protectiveStop();

  void velCmdToHw();
  void initRobot();

  arma::vec getJointPosFromHW();
  void checkJointPosDeviationError();

  double k_click;

  // freedrive model
  arma::vec q_a, dq_a, ddq_a;
  double p_a; // admittance triple pole
  double t_s; // torque scaling factor

  void initFreedrive();
  void updateFreedrive();
};

}; // namespace bhand_

}; // namespace as64_

#endif // BH282_ROBOT_H
