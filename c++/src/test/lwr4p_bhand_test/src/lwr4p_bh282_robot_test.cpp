#include <lwr4p_bhand_test/utils.h>

using namespace as64_;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lwr4p_robot_test");
  ros::NodeHandle nh("~");

  bool run = true;

  // give some time to other nodes (rviz) to start as well
  std::this_thread::sleep_for(std::chrono::milliseconds(1500));

  // ============================================
  // =========== Parse values  ==================
  // ============================================

  std::string robot_desc;
  if (!nh.getParam("robot_description_name",robot_desc)) throw std::ios_base::failure("Failed to read parameter \"robot_description_name\".");

  std::vector<double> q_vec;

  // =========== lwr args  ==================
  std::string lwr_base_link;
  std::string lwr_tool_link;
  double lwr_ctrl_cycle;
  arma::vec lwr_q1;
  arma::vec lwr_q2;
  double lwr_time_duration; // sec
  bool lwr_use_sim;

  if (!nh.getParam("lwr_base_link",lwr_base_link)) throw std::ios_base::failure("Failed to read parameter \"lwr_base_link\".");
  if (!nh.getParam("lwr_tool_link",lwr_tool_link)) throw std::ios_base::failure("Failed to read parameter \"lwr_tool_link\".");
  if (!nh.getParam("lwr_ctrl_cycle",lwr_ctrl_cycle)) throw std::ios_base::failure("Failed to read parameter \"lwr_ctrl_cycle\".");
  if (!nh.getParam("lwr_q1",q_vec)) throw std::ios_base::failure("Failed to read parameter \"lwr_q1\".");
  lwr_q1 = q_vec;
  if (!nh.getParam("lwr_q2",q_vec)) throw std::ios_base::failure("Failed to read parameter \"lwr_q2\".");
  lwr_q2 = q_vec;
  if (!nh.getParam("lwr_time_duration",lwr_time_duration)) throw std::ios_base::failure("Failed to read parameter \"lwr_time_duration\".");
  if (!nh.getParam("lwr_use_sim",lwr_use_sim)) throw std::ios_base::failure("Failed to read parameter \"lwr_use_sim\".");

  // =========== bhand args  ==================
  std::string bh_base_link;
  std::vector<std::string> bh_tool_link;
  double bh_ctrl_cycle;
  arma::vec bh_q1;
  arma::vec bh_q2;
  double bh_time_duration; // sec
  bool bh_use_sim;

  if (!nh.getParam("bh_base_link",bh_base_link)) throw std::ios_base::failure("Failed to read parameter \"bh_base_link\".");
  if (!nh.getParam("bh_tool_link",bh_tool_link)) throw std::ios_base::failure("Failed to read parameter \"bh_tool_link\".");
  if (!nh.getParam("bh_ctrl_cycle",bh_ctrl_cycle)) throw std::ios_base::failure("Failed to read parameter \"bh_ctrl_cycle\".");
  if (!nh.getParam("bh_q1",q_vec)) throw std::ios_base::failure("Failed to read parameter \"bh_q1\".");
  bh_q1 = q_vec;
  if (!nh.getParam("bh_q2",q_vec)) throw std::ios_base::failure("Failed to read parameter \"bh_q2\".");
  bh_q2 = q_vec;
  if (!nh.getParam("bh_time_duration",bh_time_duration)) throw std::ios_base::failure("Failed to read parameter \"bh_time_duration\".");
  if (!nh.getParam("bh_use_sim",bh_use_sim)) throw std::ios_base::failure("Failed to read parameter \"bh_use_sim\".");


  // ========================================
  // =========== initialize robot ===========
  // ========================================

  // =========== init lwr4p robot ===========
  std::shared_ptr<lwr4p_::RobotArm> lwr4p_robot;
  if (lwr_use_sim) lwr4p_robot.reset(new lwr4p_::SimRobot(robot_desc,lwr_base_link,lwr_tool_link,lwr_ctrl_cycle));
  else lwr4p_robot.reset(new lwr4p_::Robot(robot_desc,lwr_base_link,lwr_tool_link,lwr_ctrl_cycle));
  lwr4p_robot->setJointLimitCheck(true);
  lwr4p_robot->setSingularityCheck(true);
  // lwr4p_robot->setSingularityThreshold(8e-3);
  // lwr4p_robot->readWrenchFromTopic("/wrench");
  // or
  // lwr4p_robot->setGetExternalWrenchFun(/*some function pointer*/);

  // =========== init bh282 robot ===========
  std::shared_ptr<bhand_::RobotHand> bhand_robot;
  if (bh_use_sim) bhand_robot.reset(new bhand_::Bh282SimRobot(robot_desc,bh_base_link,bh_tool_link,bh_ctrl_cycle));
  else bhand_robot.reset(new bhand_::Bh282Robot(robot_desc,bh_base_link,bh_tool_link,bh_ctrl_cycle));
  bhand_robot->setJointLimitCheck(true);

  // ========================================================
  // =========== initialize joint state publisher ===========
  // ========================================================
  as64_::misc_::JointStatePublisher jState_pub;
  jState_pub.setPublishCycle(0.0333); // 30 Hz
  std::string publish_states_topic;
  nh.getParam("publish_states_topic",publish_states_topic);
  jState_pub.setPublishTopic(publish_states_topic);
  // jState_pub.setPublishTopic("/robot_joint_states");
  jState_pub.addFun(&lwr4p_::RobotArm::addJointState, lwr4p_robot.get());
  jState_pub.addFun(&bhand_::RobotHand::addJointState, bhand_robot.get());

  jState_pub.start(); // launches joint states publisher thread

  // =================================================
  // =========== Start robot execution ===============
  // =================================================

  // =========== Start lwr robot execution ===============
  std::thread lwr_thread;
  LwrExecArgs lwr_args(lwr_q1, lwr_q2, lwr_time_duration, lwr4p_robot.get(), &run);
  lwr_thread = std::thread(lwrRobotRun, &lwr_args);

  // =========== Start bh robot execution ===============
  std::thread bh_thread;
  BhExecArgs bh_args(bh_q1, bh_q2, bh_time_duration, bhand_robot.get(), &run);
  bh_thread = std::thread(bhRobotRun, &bh_args);

  // ===========  join threads  =================
  while (ros::ok());
  run = false;
  lwr_thread.join();
  bh_thread.join();

  return 0;
}
