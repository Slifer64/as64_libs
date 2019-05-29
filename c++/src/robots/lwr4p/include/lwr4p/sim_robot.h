#ifndef LWR4P_SIM_ROBOT_H
#define LWR4P_SIM_ROBOT_H

#include <lwr4p/robot_arm.h>

namespace as64_
{

namespace lwr4p_
{

class SimRobot : public RobotArm
{
public:
  SimRobot();
  SimRobot(urdf::Model &urdf_model, const std::string &base_link, const std::string &tool_link, double ctrl_cycle);
  SimRobot(const std::string &robot_desc_param, const std::string &base_link, const std::string &tool_link, double ctrl_cycle);

  // void setJointLimitCheck(bool check);
  // void setSingularityCheck(bool check);
  // void setSingularityThreshold(double thres);
  // void readWrenchFromTopic(bool set, const std::string &topic="");

  // virtual bool isOk() const;
  // virtual void enable();
  // std::string getErrMsg() const;
  // lwr4p_::Mode getMode() const;
  // double getCtrlCycle() const;
  // int getNumJoints() const;
  // bool setJointsTrajectory(const arma::vec &j_targ, double duration);

  virtual void update();
  virtual void setMode(const lwr4p_::Mode &m);

  void setJointsPosition(const arma::vec &j_pos);
  void setJointsVelocity(const arma::vec &j_vel);
  void setTaskVelocity(const arma::vec &task_vel);
  void setJointsTorque(const arma::vec &j_torques);
  void setTaskPose(const arma::mat &task_pose);
  void setWrench(const arma::vec &wrench);

  // virtual void setCartStiffness(const arma::vec &cart_stiff);
  // virtual void setCartDamping(const arma::vec &cart_damp);
  //
  // virtual arma::vec getJointsPosition() const;
  // virtual arma::vec getJointsVelocity() const;
  // virtual arma::mat getTaskPose() const;
  // virtual arma::vec getTaskPosition() const;
  // virtual arma::vec getTaskOrientation() const;
  // virtual arma::mat getJacobian() const;
  // virtual arma::mat getEEJacobian() const;
  // virtual arma::vec getJointsTorque() const;
  // virtual arma::vec getExternalWrench() const;

  virtual void setCartStiffness(const arma::vec &cart_stiff);
  virtual void setCartDamping(const arma::vec &cart_damp);

  // arma::vec getJointsPosition(const arma::mat &pose, const arma::vec &q0, bool *found_solution=NULL) const;
  // arma::mat getTaskPose(const arma::vec &j_pos) const;
  // arma::mat getJacobian(const arma::vec j_pos) const;
  //
  // void addJointState(sensor_msgs::JointState &joint_state_msg);

private:

  arma::vec cart_stiff;
  arma::vec cart_damp;

  void initSimRobot();

  arma::vec getExternalWrenchImplementation();

  void stop();
  void protectiveStop();

  lwr4p_::Timer timer;
  unsigned long update_time;

  ros::Subscriber jState_sub; ///< joint state subscriber
  void jStateSubCallback(const sensor_msgs::JointState::ConstPtr& jState);
};

}; // namespace lwr4p_

}; // namespace as64_

#endif // LWR4P_SIM_ROBOT_H
