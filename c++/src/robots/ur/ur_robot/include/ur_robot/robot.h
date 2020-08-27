#ifndef UR_ROBOT_H
#define UR_ROBOT_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <chrono>
#include <functional>
#include <thread>
#include <mutex>
#include <memory>

#include <ros/ros.h>
//#include <std_msgs/String.h>
//#include <geometry_msgs/WrenchStamped.h>
//#include <geometry_msgs/TwistStamped.h>
//#include <sensor_msgs/JointState.h>
//#include <tf2_ros/transform_listener.h>
//#include <geometry_msgs/TransformStamped.h>

#include <ur_modern_driver/utils.h>
#include <ur_modern_driver/ur_driver.h>

#include <armadillo>

#include <ur_robot/robot_arm.h>

namespace as64_
{

namespace ur_
{

#define UR_Robot_fun_ std::string("[ur_::Robot::") + __func__ + "]: "

class Robot : public RobotArm
{
public:
  Robot(const std::string &robot_ip, int reverse_port, double ctrl_cycle);
  Robot(urdf::Model &urdf_model, const std::string &base_link, const std::string &tool_link,
    double ctrl_cycle, const std::string &robot_ip, int reverse_port);
  Robot(const std::string &robot_desc_param, const std::string &base_link, const std::string &tool_link,
    double ctrl_cycle, const std::string &robot_ip, int reverse_port);
  ~Robot();

  bool isOk() const override;
  void enable() override;
  void update() override;
  void setFreedriveMode() override;
  void setNormalMode() override;

  arma::vec getJointsPosition() const override
  { return ur_driver->getJointPos(); }

  arma::vec getJointsVelocity() const override
  { return ur_driver->getJointVel(); }

  arma::vec getTaskVelocity() const override
  { return ur_driver->getTcpVel(); }

  arma::mat getEEJacobian() const override
  {
    // std::unique_lock<std::mutex> lck(robot_state_mtx);
    arma::mat R = getTaskPose().submat(0,0,2,2);
    arma::mat Jrobot = getJacobian();
    arma::mat Jee(6, N_JOINTS);
    Jee.submat(0,0,2,N_JOINTS-1) = R * Jrobot.submat(0,0,2,N_JOINTS-1);
    Jee.submat(3,0,5,N_JOINTS-1) = R * Jrobot.submat(3,0,5,N_JOINTS-1);

    return Jee;
  }

  arma::mat getJacobian() const override
  { return robot_urdf->getJacobian(getJointsPosition()); }

  arma::mat getTaskPose() const override
  {
    arma::mat output = arma::mat().eye(4, 4);
    return output;
  }

  arma::vec getTaskPosition() const override
  { return ur_driver->getTcpPos(); }

  arma::mat getTaskRotm() const override
  {
    return quat2rotm(getTaskQuat());
  }

  arma::vec getTaskQuat() const override
  { return ur_driver->getTcpQuat(); }

  arma::vec getTcpWrench() const override
  { return ur_driver->getTcpWrench(); }

  void setJointsPosition(const arma::vec &j_pos) override
  { this->movej(j_pos, 1.4, 1.0, 0.012); }

  void setJointsVelocity(const arma::vec &j_vel) override
  {
    // ur_driver->joint_vel_cmd = dqd;
    // ur_driver->joint_pos_cmd = qd;
    this->speedj(j_vel, 10, 0.012);
  }

  void setTaskVelocity(const arma::vec &task_vel) override
  {
    //this->speedl(Twist, arma::max(arma::abs((Twist-getTaskVelocity()))/this->cycle), this->cycle);
    this->speedl(task_vel, 2, 0.012);
  }

  void setTaskPose(const arma::mat &input) override
  {
    throw std::runtime_error(UR_Robot_fun_ + "Not implemented...");
    // const arma::vec p;
    //convertPose2PosAngles(pose, p);
    // this->movel(p, 1.2, 1.0, this->cycle);
  }

private:

  /** Sends a urscript command to set the robot in control mode 'mode'. */
  void command_mode(const std::string &mode) const;

  void protectiveStop() override;

  void initRobot(const std::string &robot_ip, int reverse_port);

  arma::vec last_joint_pos;

  UrDriver *ur_driver;
  void runUrDriver();
  std::thread ur_driver_thr;
  std::string robot_ip;
  int reverse_port;
  ur_::Semaphore shutdown_sem;

  std::string ur_script; ///< string that can store an entire URscript file.

  /** Prints a vector as a URscript string. */
  std::string print_vector(const arma::vec &v) const
  {
    std::ostringstream out;
    out << "[" << v(0);
    for (int i=1;i<v.size();i++) out << "," << v(i);
    out << "]";

    return out.str();
  }

  void movej(const arma::vec &q, double a, double v, double t, double r=0);
  void movel(const arma::vec &p, double a, double v, double t, double r=0);
  void speedj(arma::vec dq, double a, double t);
  void speedl(arma::vec dp, double a, double t);
  void stopj(double a);
  void stopl(double a);
  void sleep(double t);
  void powerdown();

};

}  // namespace ur_

} // namespace as64_

#endif  // UR_ROBOT_H
