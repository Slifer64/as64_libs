#include <bhand_test/utils.h>

double err_thres = 1e-2;

void PRINT_INFO_MSG(const std::string &msg)
{
  std::cerr << "\033[1m\033[34m" << "[INFO]: " << msg << "\033[0m\n";
}

void PRINT_ERR_MSG(const std::string &msg)
{
  std::cerr << "\033[1m\033[31m" << "[ERROR]: " << msg << "\033[0m\n";
}

void parseArgs(std::string &robot_descr_name, std::vector<std::string> &base_link,
  std::vector<std::vector<std::string>> &tool_link, std::vector<double> &ctrl_cycle,
  std::vector<arma::vec>  &q1, std::vector<arma::vec>  &q2,
  std::vector<double> &time_duration, std::vector<bool> &use_sim)
{
  ros::NodeHandle nh("~");
  if (!nh.getParam("robot_description_name",robot_descr_name)) throw std::ios_base::failure("Failed to read parameter \"robot_description_name\".");

  int k = 0;

  try{
    while (true)
    {
      std::string bl;
      std::vector<std::string> tl;
      double cc;
      std::vector<double> j1;
      std::vector<double> j2;
      double td;
      bool usim;

      std::ostringstream oss;
      oss << "_" << k+1;
      std::string suffix = oss.str();

      if (!nh.getParam("base_link"+suffix,bl)) throw std::ios_base::failure("Failed to read parameter \"base_link" + suffix + "\".");
      if (!nh.getParam("tool_link"+suffix,tl)) throw std::ios_base::failure("Failed to read parameter \"tool_link" + suffix + "\".");
      if (!nh.getParam("ctrl_cycle"+suffix,cc)) throw std::ios_base::failure("Failed to read parameter \"ctrl_cycle" + suffix + "\".");
      if (!nh.getParam("q1"+suffix,j1)) throw std::ios_base::failure("Failed to read parameter \"q1" + suffix + "\".");
      if (!nh.getParam("q2"+suffix,j2)) throw std::ios_base::failure("Failed to read parameter \"q2" + suffix + "\".");
      if (!nh.getParam("time_duration"+suffix,td)) throw std::ios_base::failure("Failed to read parameter \"time_duration" + suffix + "\".");
      if (!nh.getParam("use_sim"+suffix,usim)) throw std::ios_base::failure("Failed to read parameter \"use_sim" + suffix + "\".");

      base_link.push_back(bl);
      tool_link.push_back(tl);
      ctrl_cycle.push_back(cc);
      q1.push_back(j1);
      q2.push_back(j2);
      time_duration.push_back(td);
      use_sim.push_back(usim);
      k++;
    }
  }
  catch(std::exception &e)
  {
    if (k==0)
    {
      std::cerr << "[ERROR]: " << e.what() << "\n";
      exit(-1);
    }
  }
}

void jointsTrajectory(const arma::vec &qT, double total_time, bhand_::RobotHand *robot)
{

  // ======================================================
  // ===========   Set Joints trajectory  =================
  // ======================================================
  std::cerr << "=====================================\n";
  PRINT_INFO_MSG("==> Moving with Joints Trajectory...");
  bool reached_target = robot->setJointsTrajectory(qT, total_time);
  if (reached_target) PRINT_INFO_MSG("==> Reached target pose!");
  else  PRINT_ERR_MSG("==> Failed to reach target pose!\n");
  if (!robot->isOk())
  {
    PRINT_ERR_MSG("==> " + robot->getErrMsg());
    robot->enable();
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}

void jointPositionControl(const arma::vec &qT, double total_time, bhand_::RobotHand *robot)
{

  // ======================================================
  // ===========   Joint Position Control  ================
  // ======================================================
  std::cerr << "=====================================\n";
  PRINT_INFO_MSG("==> Setting the robot to JOINT_POS_CONTROL...");
  robot->setMode(bhand_::JOINT_POS_CONTROL);
  PRINT_INFO_MSG("==> Moving with joint position control...");
  robot->update();
  double Ts = robot->getCtrlCycle();
  arma::vec q = robot->getJointsPosition();
  arma::vec q0 = q;
  double t = 0;
  while (arma::norm(qT-q)>err_thres && robot->isOk())
  {
    if (!ros::ok()) exit(-1);
    t += Ts;
    arma::vec q_ref = bhand_::get5thOrder(t, q0, qT, total_time)[0];
    robot->setJointsPosition(q_ref);
    robot->update();
    q = robot->getJointsPosition();
  }
  if (!robot->isOk())
  {
    PRINT_ERR_MSG("[ERROR]: " + robot->getErrMsg());
    robot->enable();
  }
  else PRINT_INFO_MSG("==> Reached target pose!");
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}

void jointVelocityControl(const arma::vec &qT, double total_time, bhand_::RobotHand *robot)
{

  // ======================================================
  // ===========   Joint Velocity Control  ================
  // ======================================================
  std::cerr << "=====================================\n";
  PRINT_INFO_MSG("==> Setting the robot to JOINT_VEL_CONTROL...");
  robot->setMode(bhand_::JOINT_VEL_CONTROL);
  PRINT_INFO_MSG("==> Moving with joint velocity control...");
  robot->update();
  double Ts = robot->getCtrlCycle();
  arma::vec q = robot->getJointsPosition();
  arma::vec q0 = q;
  double k_click = 0.1;
  double t = 0;
  while (arma::norm(qT-q)>err_thres && robot->isOk())
  {
    if (!ros::ok()) exit(-1);
    t += Ts;
    arma::vec q_ref = bhand_::get5thOrder(t, q0, qT, total_time)[0];
    arma::vec dq_ref = bhand_::get5thOrder(t, q0, qT, total_time)[1];
    robot->setJointsVelocity(dq_ref + k_click*(q_ref-q));
    robot->update();
    q = robot->getJointsPosition();
  }
  if (!robot->isOk())
  {
    PRINT_ERR_MSG("[ERROR]: " + robot->getErrMsg());
    robot->enable();
  }
  else PRINT_INFO_MSG("==> Reached target pose!");
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}

void freedrive(bhand_::RobotHand *robot)
{

  // =========================================
  // ===========   FREEDIRVE  ================
  // =========================================
  std::cerr << "=====================================\n";
  PRINT_INFO_MSG("==> Setting the robot to FREEDRIVE...");
  robot->setMode(bhand_::FREEDRIVE);
  PRINT_INFO_MSG("==> The robot is in FREEDIRVE. Move it wherever you want. Press ctrl+C to exit...");

  while (ros::ok() && robot->isOk())
  {
    robot->update();
  }

  if (!robot->isOk())
  {
    PRINT_INFO_MSG("[ERROR]: " + robot->getErrMsg());
    robot->enable();
  }
}

void robotRun(ExecArgs *args)
{
  arma::vec q1 = args->q1;
  arma::vec q2 = args->q2;
  double time_duration = args->total_time;
  bhand_::RobotHand *bhand_robot = args->robot;

  // ===========   Set Joints trajectory  =================
  jointsTrajectory(q1, time_duration, bhand_robot);

  // ===========   Joint Position Control  ================
  jointPositionControl(q2, time_duration, bhand_robot);

  // ===========   Joint Velocity Control  ================
  jointVelocityControl(q1, time_duration, bhand_robot);

  // ===========   FREEDIRVE  ================
  // sync all robots before stop publishing...
  // if (use_sim) jState_pub.stop(); // stop publishing in freedrive when using SimRobot
  freedrive(bhand_robot);
}
