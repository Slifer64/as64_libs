#ifndef BH282_SIM_ROBOT_H
#define BH282_SIM_ROBOT_H

#include <bhand/robot_hand.h>

namespace as64_
{

namespace bhand_
{

class Bh282SimRobot : public RobotHand
{
public:
  Bh282SimRobot();
  Bh282SimRobot(urdf::Model &urdf_model, const std::string &base_link, const std::vector<std::string> &tool_link, double ctrl_cycle);
  Bh282SimRobot(const std::string &robot_desc_param, const std::string &base_link, const std::vector<std::string> &tool_link, double ctrl_cycle);
  ~Bh282SimRobot();

  void update();
  void setMode(const bhand_::Mode &m);

  void setJointsPosition(const arma::vec &j_pos);
  void setJointsVelocity(const arma::vec &j_vel);
  void setTaskVelocity(bhand_::ChainName &chain_name, const arma::vec &task_vel);

  arma::vec getJointsTorque() const;

private:
  void initSimRobot();

  void stop();
  void protectiveStop();

  bhand_::Timer timer;
  unsigned long update_time;

  ros::Subscriber jState_sub; ///< joint state subscriber
  void jStateSubCallback(const sensor_msgs::JointState::ConstPtr& jState);
};

}; // namespace bhand_

}; // namespace as64_

#endif // BH282_SIM_ROBOT_H
