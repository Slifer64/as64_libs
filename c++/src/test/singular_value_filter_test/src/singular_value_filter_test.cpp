#include <iostream>
#include <cstdlib>
#include <chrono>
#include <memory>
#include <thread>
#include <vector>
#include <ros/ros.h>
#include <ros/package.h>
#include <armadillo>

#include <lwr4p/robot_arm.h>
#include <lwr4p/sim_robot.h>
#include <lwr4p/robot.h>
#include <misc_lib/joint_state_publisher.h>
#include <robo_lib/singular_value_filter.h>
#include <robo_lib/ppc_joint_limit_avoid.h>

using namespace as64_;

double err_thres = 1e-1;

void PRINT_INFO_MSG(const std::string &msg)
{
  std::cerr << "\033[1m\033[34m" << "[INFO]: " << msg << "\033[0m\n";
}

void PRINT_WARN_MSG(const std::string &msg)
{
  std::cerr << "\033[1m\033[33m" << "[INFO]: " << msg << "\033[0m\n";
}

void PRINT_ERR_MSG(const std::string &msg)
{
  std::cerr << "\033[1m\033[31m" << "[ERROR]: " << msg << "\033[0m\n";
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "singular_value_filter_test");

  // give some time to other nodes (rviz) to start as well
  std::this_thread::sleep_for(std::chrono::milliseconds(1500));

  std::string robot_desc;
  std::string base_link;
  std::string tool_link;
  double ctrl_cycle;

  arma::vec q1;
  arma::vec q2;
  double time_duration; // sec
  bool use_sim;

  // =========================================
  // =========== Parse values  ===============
  // =========================================
  ros::NodeHandle nh("~");
  if (!nh.getParam("robot_description_name",robot_desc)) throw std::ios_base::failure("Failed to read parameter \"robot_description_name\".");
  if (!nh.getParam("base_link",base_link)) throw std::ios_base::failure("Failed to read parameter \"base_link\".");
  if (!nh.getParam("tool_link",tool_link)) throw std::ios_base::failure("Failed to read parameter \"tool_link\".");
  if (!nh.getParam("ctrl_cycle",ctrl_cycle)) throw std::ios_base::failure("Failed to read parameter \"ctrl_cycle\".");

  std::vector<double> q_vec;
  if (!nh.getParam("q1",q_vec)) throw std::ios_base::failure("Failed to read parameter \"q1\".");
  q1 = q_vec;
  if (!nh.getParam("q2",q_vec)) throw std::ios_base::failure("Failed to read parameter \"q2\".");
  q2 = q_vec;
  if (!nh.getParam("time_duration",time_duration)) throw std::ios_base::failure("Failed to read parameter \"time_duration\".");
  if (!nh.getParam("use_sim",use_sim)) throw std::ios_base::failure("Failed to read parameter \"use_sim\".");

  // ========================================
  // =========== initialize robot ===========
  // ========================================
  std::shared_ptr<lwr4p_::RobotArm> lwr4p_robot;
  if (use_sim) lwr4p_robot.reset(new lwr4p_::SimRobot(robot_desc,base_link,tool_link,ctrl_cycle));
  else lwr4p_robot.reset(new lwr4p_::Robot(robot_desc,base_link,tool_link,ctrl_cycle));
  lwr4p_robot->setJointLimitCheck(true);
  lwr4p_robot->setSingularityCheck(true);
  // lwr4p_robot->setSingularityThreshold(0.05);

  // ========================================================
  // =========== initialize joint state publisher ===========
  // ========================================================
  as64_::misc_::JointStatePublisher jState_pub;
  jState_pub.setPublishCycle(0.0333); // 30 Hz
  std::string publish_states_topic;
  nh.getParam("publish_states_topic",publish_states_topic);
  jState_pub.setPublishTopic(publish_states_topic);
  jState_pub.addFun(&lwr4p_::RobotArm::addJointState, lwr4p_robot.get());

  jState_pub.start(); // launches joint states publisher thread

  // =================================================
  // =========== Start robot execution ===============
  // =================================================

  // get final Cartesian pose
  arma::mat pose2 = lwr4p_robot->robot_urdf->getTaskPose(q2);
  arma::vec p2 = pose2.submat(0,3,2,3);
  arma::vec Q2 = lwr4p_::rotm2quat(pose2.submat(0,0,3,3));

  // init some variables
  double Ts = lwr4p_robot->getCtrlCycle();
  double t = 0;

  // init jlav object
  double kq;
  if (!nh.getParam("kq",kq)) kq = 8e-4;
  double jlim_safety_margin;
  if (!nh.getParam("jlim_safety_margin",jlim_safety_margin)) jlim_safety_margin = 5;
  jlim_safety_margin *= 3.14/180; // convert to rad
  arma::vec q_min = lwr4p_robot->getLowerJointLimits() + jlim_safety_margin;
  arma::vec q_max = lwr4p_robot->getUpperJointLimits() - jlim_safety_margin;
  std::shared_ptr<robo_::PPCJointLimAvoid> jlav(new robo_::PPCJointLimAvoid(q_min, q_max));
  jlav->setGains(kq);

  // =========== Move to start pose ===============
  PRINT_INFO_MSG("==> Moving to start pose...\n");
  lwr4p_robot->setJointsTrajectory(q1, time_duration);
  PRINT_INFO_MSG("==> Reached start pose!\n");
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  // =========== Move to target pose without singular value filter ===============
  PRINT_INFO_MSG("==> Moving to target pose without svf...\n");
  lwr4p_robot->setMode(lwr4p_::CART_VEL_CONTROL);
  lwr4p_robot->update();
  t = 0;
  while (true)
  {
    arma::vec q = lwr4p_robot->getJointsPosition();
    arma::vec p = lwr4p_robot->getTaskPosition();
    arma::vec Q = lwr4p_robot->getTaskQuat();

    arma::vec V_cmd(6);
    arma::vec err = arma::join_vert(p2 - p, Q2.subvec(1,3) - Q.subvec(1,3) );
    V_cmd = (5/time_duration)*( 1 - std::exp(-t) ) * err;
    arma::vec dq_cmd = arma::pinv(lwr4p_robot->getJacobian()) * V_cmd + jlav->getControlSignal(q);

    lwr4p_robot->setTaskVelocity(V_cmd);
    lwr4p_robot->update();
    t += Ts;

    if (!ros::ok()) exit(-1);

    if (arma::norm(err) < err_thres)
    {
      PRINT_INFO_MSG("==> Reached target pose!");
      break;
    }

    if (t > 2*time_duration)
    {
      PRINT_WARN_MSG("==> Couldn't reach target with the desired accuracy...");
      break;
    }

    if (! lwr4p_robot->isOk() )
    {
      PRINT_ERR_MSG("[ERROR]: " + lwr4p_robot->getErrMsg());
      lwr4p_robot->enable();
      break;
    }
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));


  // =========== Move back to start pose ===============
  PRINT_INFO_MSG("==> Moving to start pose...\n");
  bool reached_target = lwr4p_robot->setJointsTrajectory(q1, time_duration);
  PRINT_INFO_MSG("==> Reached start pose!\n");
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  // =========== Move to target pose with singular value filter ===============

  // init svf object
  double sigma_min;
  if (!nh.getParam("sigma_min",sigma_min)) sigma_min = 1e-3;
  double shape_f;
  if (!nh.getParam("shape_f",shape_f)) shape_f = 10;
  std::shared_ptr<robo_::SingularValueFilter> svf(new robo_::SingularValueFilter(sigma_min, shape_f));

  PRINT_INFO_MSG("==> Moving to target pose with svf...\n");
  lwr4p_robot->setMode(lwr4p_::JOINT_VEL_CONTROL);
  lwr4p_robot->update();
  t = 0;
  while (true)
  {
    arma::vec q = lwr4p_robot->getJointsPosition();
    arma::vec p = lwr4p_robot->getTaskPosition();
    arma::vec Q = lwr4p_robot->getTaskQuat();

    arma::vec V_cmd(6);
    arma::vec err = arma::join_vert(p2 - p, Q2.subvec(1,3) - Q.subvec(1,3) );
    V_cmd = (5/time_duration)*( 1 - std::exp(-t) ) * err;
    arma::vec dq_cmd = svf->pinv(lwr4p_robot->getJacobian()) * V_cmd + jlav->getControlSignal(q);

    lwr4p_robot->setJointsVelocity(dq_cmd);
    lwr4p_robot->update();
    t += Ts;

    if (!ros::ok()) exit(-1);

    if (arma::norm(err) < err_thres)
    {
      PRINT_INFO_MSG("==> Reached target pose!");
      break;
    }

    if (t > 2*time_duration)
    {
      PRINT_WARN_MSG("==> Couldn't reach target with the desired accuracy...");
      break;
    }

    if (! lwr4p_robot->isOk() )
    {
      PRINT_ERR_MSG("[ERROR]: " + lwr4p_robot->getErrMsg());
      lwr4p_robot->enable();
      break;
    }
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  PRINT_INFO_MSG("Press ctrl+c to exit...");
  while (ros::ok());

  return 0;
}
