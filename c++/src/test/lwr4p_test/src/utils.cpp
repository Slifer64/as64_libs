#include <lwr4p_test/utils.h>

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
  std::vector<std::string> &tool_link, std::vector<double> &ctrl_cycle,
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
      std::string tl;
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

void jointsTrajectory(const arma::vec &qT, double total_time, lwr4p_::RobotArm *robot)
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

void jointPositionControl(const arma::vec &qT, double total_time, lwr4p_::RobotArm *robot)
{

  // ======================================================
  // ===========   Joint Position Control  ================
  // ======================================================
  std::cerr << "=====================================\n";
  PRINT_INFO_MSG("==> Setting the robot to JOINT_POS_CONTROL...");
  robot->setMode(lwr4p_::JOINT_POS_CONTROL);
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
    arma::vec q_ref = lwr4p_::get5thOrder(t, q0, qT, total_time)[0];
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

void jointVelocityControl(const arma::vec &qT, double total_time, lwr4p_::RobotArm *robot)
{

  // ======================================================
  // ===========   Joint Velocity Control  ================
  // ======================================================
  std::cerr << "=====================================\n";
  PRINT_INFO_MSG("==> Setting the robot to JOINT_VEL_CONTROL...");
  robot->setMode(lwr4p_::JOINT_VEL_CONTROL);
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
    arma::vec q_ref = lwr4p_::get5thOrder(t, q0, qT, total_time)[0];
    arma::vec dq_ref = lwr4p_::get5thOrder(t, q0, qT, total_time)[1];
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

void cartesianVelocityControl(const arma::vec &qT, double total_time, lwr4p_::RobotArm *robot)
{
  // ==========================================================
  // ===========   Cartesian Velocity Control  ================
  // ==========================================================
  std::cerr << "=====================================\n";
  PRINT_INFO_MSG("==> Setting the robot to CART_VEL_CONTROL...");
  robot->setMode(lwr4p_::CART_VEL_CONTROL);
  PRINT_INFO_MSG("==> Moving with Cartesian velocity control...");
  robot->update();
  arma::vec q = robot->getJointsPosition();
  arma::vec q0 = q;
  arma::mat p0 = robot->getTaskPose(q0);
  arma::mat pT = robot->getTaskPose(qT);
  arma::mat p = robot->getTaskPose();
  double t = 0;
  double Ts = robot->getCtrlCycle();
  // Notice that if we were to check the joint position instead of the Cartesian we may never
  // reach qT, because due to pinv(J) we may arrive at pT with a different joint configuration.
  while (arma::norm(arma::vectorise(pT)-arma::vectorise(p))>err_thres && robot->isOk())
  {
    if (!ros::ok()) exit(-1);
    t += Ts;
    arma::vec q_ref = lwr4p_::get5thOrder(t, q0, qT, total_time)[0];
    // arma::vec dq_ref = lwr4p_::get5thOrder(t, q0, qT, total_time)[1];
    arma::vec v_click(6);
    arma::mat p_ref = robot->getTaskPose(q_ref);
    // position error
    v_click.subvec(0,2) = p_ref.submat(0,3,2,3) - p.submat(0,3,2,3);
    // orientation error (difference of quaternions vector parts)
    v_click.subvec(3,5) = lwr4p_::rotm2quat(p_ref.submat(0,0,2,2)).subvec(1,3) - lwr4p_::rotm2quat(p.submat(0,0,2,2)).subvec(1,3);
    // v_click *= 4.0;
    // arma::vec V = robot->getJacobian()*dq_ref + v_click;
    arma::vec V = v_click/Ts;
    robot->setTaskVelocity(V);
    robot->update();
    p = robot->getTaskPose();
  }
  if (!robot->isOk())
  {
    PRINT_ERR_MSG("[ERROR]: " + robot->getErrMsg());
    robot->enable();
  }
  else PRINT_INFO_MSG("==> Reached target pose!");
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}

void taskTrajectory(const arma::vec &qT, double total_time, lwr4p_::RobotArm *robot)
{

  // ======================================================
  // ===========   Set Task trajectory  =================
  // ======================================================
  std::cerr << "=====================================\n";
  PRINT_INFO_MSG("==> Moving with Task Trajectory...");
  arma::mat target_pose = robot->getTaskPose(qT);
  bool reached_target = robot->setTaskTrajectory(target_pose, total_time);
  if (reached_target) PRINT_INFO_MSG("==> Reached target pose!");
  else  PRINT_ERR_MSG("==> Failed to reach target pose!");
  if (!robot->isOk())
  {
    PRINT_ERR_MSG("==> " + robot->getErrMsg());
    robot->enable();
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}

void freedrive(lwr4p_::RobotArm *robot)
{

  // =========================================
  // ===========   FREEDIRVE  ================
  // =========================================
  std::cerr << "=====================================\n";
  PRINT_INFO_MSG("==> Setting the robot to FREEDRIVE...");
  robot->setMode(lwr4p_::FREEDRIVE);
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
  lwr4p_::RobotArm *lwr4p_robot = args->robot;

  // ===========   Set Joints trajectory  =================
  jointsTrajectory(q1, time_duration, lwr4p_robot);

  // ===========   Joint Position Control  ================
  jointPositionControl(q2, time_duration, lwr4p_robot);

  // ===========   Joint Velocity Control  ================
  jointVelocityControl(q1, time_duration, lwr4p_robot);

  // ===========   Cartesian Velocity Control  ================
  cartesianVelocityControl(q2, time_duration, lwr4p_robot);

  // ===========   Set Tasktrajectory  =================
  taskTrajectory(q1, time_duration, lwr4p_robot);

  // ===========   FREEDIRVE  ================
  // sync all robots before stop publishing...
  // if (use_sim) jState_pub.stop(); // stop publishing in freedrive when using SimRobot
  freedrive(lwr4p_robot);
}
