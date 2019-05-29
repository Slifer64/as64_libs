#include <lwr4p_bhand_test/utils.h>

double err_thres = 1e-2;

void PRINT_INFO_MSG(const std::string &msg)
{
  std::cerr << "\033[1m\033[34m" << "[INFO]: " << msg << "\033[0m\n";
}

void PRINT_ERR_MSG(const std::string &msg)
{
  std::cerr << "\033[1m\033[31m" << "[ERROR]: " << msg << "\033[0m\n";
}

void lwrRobotRun(LwrExecArgs *args)
{
  arma::vec q1 = args->q1;
  arma::vec q2 = args->q2;
  double total_time = args->total_time;
  lwr4p_::RobotArm *robot = args->robot;
  const bool *run = args->run;

  arma::vec qT;

  // ======================================================
  // ===========   Set Joints trajectory  =================
  // ======================================================
  std::cerr << "=====================================\n";
  PRINT_INFO_MSG("==> Moving with Joints Trajectory...");
  qT = q1;
  bool reached_target = robot->setJointsTrajectory(qT, total_time);
  if (reached_target) PRINT_INFO_MSG("==> Reached target pose!");
  else  PRINT_ERR_MSG("==> Failed to reach target pose!\n");
  if (!robot->isOk())
  {
    PRINT_ERR_MSG("==> " + robot->getErrMsg());
    robot->enable();
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

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
  qT = q2;
  while (arma::norm(qT-q)>err_thres && robot->isOk())
  {
    if (!(*run)) exit(-1);
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

  // =========================================
  // ===========   FREEDIRVE  ================
  // =========================================
  std::cerr << "=====================================\n";
  PRINT_INFO_MSG("==> Setting the robot to FREEDRIVE...");
  robot->setMode(lwr4p_::FREEDRIVE);
  PRINT_INFO_MSG("==> The robot is in FREEDIRVE. Move it wherever you want. Press ctrl+C to exit...");

  while ((*run) && robot->isOk())
  {
    robot->update();
  }

  if (!robot->isOk())
  {
    PRINT_INFO_MSG("[ERROR]: " + robot->getErrMsg());
    robot->enable();
  }
}

void bhRobotRun(BhExecArgs *args)
{
  arma::vec q1 = args->q1;
  arma::vec q2 = args->q2;
  double total_time = args->total_time;
  bhand_::RobotHand *robot = args->robot;
  const bool *run = args->run;

  arma::vec qT;

  // ======================================================
  // ===========   Set Joints trajectory  =================
  // ======================================================
  std::cerr << "=====================================\n";
  PRINT_INFO_MSG("==> Moving with Joints Trajectory...");
  qT = q1;
  bool reached_target = robot->setJointsTrajectory(qT, total_time);
  if (reached_target) PRINT_INFO_MSG("==> Reached target pose!");
  else  PRINT_ERR_MSG("==> Failed to reach target pose!\n");
  if (!robot->isOk())
  {
    PRINT_ERR_MSG("==> " + robot->getErrMsg());
    robot->enable();
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

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
  qT = q2;
  while (arma::norm(qT-q)>err_thres && robot->isOk())
  {
    if (!(*run)) exit(-1);
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

  // ======================================================
  // ===========   Joint Velocity Control  ================
  // ======================================================
  std::cerr << "=====================================\n";
  PRINT_INFO_MSG("==> Setting the robot to JOINT_VEL_CONTROL...");
  robot->setMode(bhand_::JOINT_VEL_CONTROL);
  PRINT_INFO_MSG("==> Moving with joint velocity control...");
  robot->update();
  q = robot->getJointsPosition();
  q0 = q;
  double k_click = 0.1;
  t = 0;
  qT = q1;
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

  // =========================================
  // ===========   FREEDIRVE  ================
  // =========================================
  std::cerr << "=====================================\n";
  PRINT_INFO_MSG("==> Setting the robot to FREEDRIVE...");
  robot->setMode(bhand_::FREEDRIVE);
  PRINT_INFO_MSG("==> The robot is in FREEDIRVE. Move it wherever you want. Press ctrl+C to exit...");

  while ((*run) && robot->isOk())
  {
    robot->update();
  }

  if (!robot->isOk())
  {
    PRINT_INFO_MSG("[ERROR]: " + robot->getErrMsg());
    robot->enable();
  }
}
