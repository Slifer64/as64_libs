#include <bhand_test/utils.h>

using namespace as64_;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "multi_bhand_robot_test");

  // give some time to other nodes (rviz) to start as well
  std::this_thread::sleep_for(std::chrono::milliseconds(1500));

  // ============================================
  // =========== Parse values  ==================
  // ============================================
  std::string robot_descr_name;
  std::vector<std::string> base_link;
  std::vector<std::vector<std::string>> tool_link;
  std::vector<double> ctrl_cycle;

  std::vector<arma::vec> q1;
  std::vector<arma::vec> q2;
  std::vector<double> time_duration;
  std::vector<bool> use_sim;

  parseArgs(robot_descr_name, base_link, tool_link, ctrl_cycle, q1, q2, time_duration, use_sim);

  int N_robots = base_link.size();

  // =========================================
  // =========== initialize robots ===========
  // =========================================
  std::vector<std::shared_ptr<bhand_::RobotHand>> bhand_robot(N_robots);
  for (int i=0; i<N_robots; i++)
  {
    if (use_sim[i]) bhand_robot[i].reset(new bhand_::Bh282SimRobot(robot_descr_name, base_link[i], tool_link[i], ctrl_cycle[i]));
    else bhand_robot[i].reset(new bhand_::Bh282Robot(robot_descr_name, base_link[i], tool_link[i], ctrl_cycle[i]));
    bhand_robot[i]->setJointLimitCheck(true);
  }

  // ========================================================
  // =========== initialize joint state publisher ===========
  // ========================================================
  as64_::misc_::JointStatePublisher jState_pub;
  jState_pub.setPublishCycle(0.0333); // 30 Hz
  std::string publish_states_topic;
  ros::NodeHandle nh("~");
  if (!nh.getParam("publish_states_topic",publish_states_topic)) throw std::ios_base::failure("Failed to read parameter \"publish_states_topic\".");
  jState_pub.setPublishTopic(publish_states_topic);
  // jState_pub.setPublishTopic("/robot_joint_states");
  for (int i=0; i<N_robots; i++) jState_pub.addFun(&bhand_::Bh282SimRobot::addJointState, bhand_robot[i].get());

  jState_pub.start(); // launches joint states publisher thread

  // ========================================================
  // =========== Launch robots execution threads ============
  // ========================================================
  std::vector<std::thread> robot_run_thead(N_robots);
  std::vector<std::shared_ptr<ExecArgs>> args(N_robots);
  for (int i=0; i<N_robots; i++)
  {
    args[i].reset(new ExecArgs(q1[i], q2[i], time_duration[i], bhand_robot[i].get()));
    robot_run_thead[i] = std::thread(robotRun, args[i].get());
  }

  for (int i=0; i<N_robots; i++)
  {
    if (robot_run_thead[i].joinable()) robot_run_thead[i].join();
  }

  jState_pub.stop();

  return 0;
}
