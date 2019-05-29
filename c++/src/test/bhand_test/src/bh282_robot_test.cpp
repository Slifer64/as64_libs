#include <bhand_test/utils.h>

using namespace as64_;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "bhand_robot_test");
  ros::NodeHandle nh("~");

  // give some time to other nodes (rviz) to start as well
  std::this_thread::sleep_for(std::chrono::milliseconds(1500));

  // ============================================
  // =========== Parse values  ==================
  // ============================================

  std::string robot_desc;
  std::string base_link;
  std::vector<std::string> tool_link;
  double ctrl_cycle;

  arma::vec q1;
  arma::vec q2;
  double time_duration; // sec
  bool use_sim;

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
  std::shared_ptr<bhand_::RobotHand> bhand_robot;
  if (use_sim) bhand_robot.reset(new bhand_::Bh282SimRobot(robot_desc,base_link,tool_link,ctrl_cycle));
  else bhand_robot.reset(new bhand_::Bh282Robot(robot_desc,base_link,tool_link,ctrl_cycle));
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
  jState_pub.addFun(&bhand_::RobotHand::addJointState, bhand_robot.get());

  jState_pub.start(); // launches joint states publisher thread

  // =================================================
  // =========== Start robot execution ===============
  // =================================================

  // ===========   Set Joints trajectory  =================
  jointsTrajectory(q1, time_duration, bhand_robot.get());

  // ===========   Joint Position Control  ================
  jointPositionControl(q2, time_duration, bhand_robot.get());

  // ===========   Joint Velocity Control  ================
  jointVelocityControl(q1, time_duration, bhand_robot.get());

  jState_pub.stop(); // launches joint states publisher thread
  // ===========   FREEDIRVE  ================
  freedrive(bhand_robot.get());

  return 0;
}
