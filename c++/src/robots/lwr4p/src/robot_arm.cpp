#include <lwr4p/robot_arm.h>

#include <stdexcept>
#include <sstream>
#include <chrono>
#include <map>
#include <stack>

#include <ros/package.h>
#include <kdl_parser/kdl_parser.hpp>

namespace as64_
{

namespace lwr4p_
{

RobotArm::RobotArm()
{
  std::string robot_description_name;
  if (!node.getParam("/lwr4p_robot/robot_description_name",robot_description_name))
  {
    throw std::runtime_error("[RobotArm Error]: Failed to load parameter \"/lwr4p_robot/robot_description_name\" ...\n");
  }

  if (!node.getParam("/lwr4p_robot/base_frame",base_link_name))
  {
    throw std::runtime_error("[RobotArm Error]: Failed to load parameter \"/lwr4p_robot/base_frame\" ...\n");
  }

  if (!node.getParam("/lwr4p_robot/tool_frame",tool_link_name))
  {
    throw std::runtime_error("[RobotArm Error]: Failed to load parameter \"/lwr4p_robot/tool_frame\" ...\n");
  }

  if (!node.getParam("/lwr4p_robot/ctrl_cycle",ctrl_cycle))
  {
    ctrl_cycle = 0.01;
  }

  if (!node.getParam("/lwr4p_robot/check_limits",check_limits))
  {
    check_limits = false;
  }

  if (!node.getParam("/lwr4p_robot/check_singularity",check_singularity))
  {
    check_singularity = false;
  }

  //std::string urdf_file_path = ros::package::getPath("lwr4p") + "/urdf/lwr4p_robot.urdf";
  if (!urdf_model.initParam(robot_description_name.c_str()))
  // if (!urdf_model.initFile(urdf_file_path.c_str()))
  {
    throw std::ios_base::failure("[RobotArm Error]: Couldn't load urdf model from \"" + robot_description_name + "\"...\n");
  }

  init();
}

RobotArm::RobotArm(urdf::Model &urdf_model, const std::string &base_link, const std::string &tool_link, double ctrl_cycle)
{
  this->urdf_model = urdf_model;
  this->base_link_name = base_link;
  this->tool_link_name = tool_link;
  this->ctrl_cycle = ctrl_cycle;
  this->check_limits = false;
  this->check_singularity = false;

  init();
}

RobotArm::RobotArm(const std::string &robot_desc_param, const std::string &base_link, const std::string &tool_link, double ctrl_cycle)
{
  if (!urdf_model.initParam(robot_desc_param.c_str()))
  {
    throw std::ios_base::failure("Couldn't load urdf model from \"" + robot_desc_param + "\"...\n");
  }

  this->base_link_name = base_link;
  this->tool_link_name = tool_link;
  this->ctrl_cycle = ctrl_cycle;
  this->check_limits = false;
  this->check_singularity = false;

  init();
}

RobotArm::~RobotArm() {}

void RobotArm::init()
{
  mode_name[lwr4p_::IDLE] = "IDLE";
  mode_name[lwr4p_::FREEDRIVE] = "FREEDRIVE";
  mode_name[lwr4p_::JOINT_POS_CONTROL] = "JOINT_POS_CONTROL";
  mode_name[lwr4p_::JOINT_VEL_CONTROL] = "JOINT_VEL_CONTROL";
  mode_name[lwr4p_::JOINT_TORQUE_CONTROL] = "JOINT_TORQUE_CONTROL";
  mode_name[lwr4p_::CART_VEL_CONTROL] = "CART_VEL_CONTROL";
  mode_name[lwr4p_::CART_IMPEDANCE_CONTROL] = "CART_IMPEDANCE_CONTROL";
  mode_name[lwr4p_::PROTECTIVE_STOP] = "PROTECTIVE_STOP";

  mode = lwr4p_::Mode::IDLE;

  setSingularityThreshold(0.05);

  // find base_link and tool_link
  bool found_base_link = false;
  bool found_tool_link = false;
  boost::shared_ptr<const urdf::Link> base_link;
  boost::shared_ptr<const urdf::Link> tool_link;
  std::stack<boost::shared_ptr<const urdf::Link>> link_stack;
  link_stack.push(urdf_model.getRoot());
  while (!link_stack.empty())
  {
    auto link = link_stack.top();
    link_stack.pop();

    if (base_link_name.compare(link->name) == 0)
    {
      base_link = link;
      found_base_link = true;
    }

    if (tool_link_name.compare(link->name) == 0)
    {
      tool_link = link;
      found_tool_link = true;
    }

    for (int i=0;i<link->child_links.size();i++) link_stack.push(link->child_links[i]);
    // for (int i=0;i<link->child_joints.size();i++) _joints.push_back(link->child_joints[i]);
  }

  if (!found_base_link)
    throw std::runtime_error("[RobotArm Error]: Couldn't find specified base link \"" + base_link_name + "\" in the robot urdf model...\n");

  if (!found_tool_link)
    throw std::runtime_error("[RobotArm Error]: Couldn't find specified tool link \"" + tool_link_name + "\" in the robot urdf model...\n");

  // find all links in the chain from tool_link to base_link
  std::vector<boost::shared_ptr<const urdf::Link>> chain_links;
  auto link = tool_link;
  while (link->name.compare(base_link->name))
  {
    chain_links.push_back(link);
    link = link->getParent();
  }
  chain_links.push_back(base_link);

  // parse all joints for each link starting from base_link
  for (int i=chain_links.size()-1; i>0; i--)
  {
    link = chain_links[i];
    auto next_link = chain_links[i-1];

    for (int i=0;i<link->child_joints.size();i++)
    {
      auto joint = link->child_joints[i];
      auto jtype = joint->type;

      if (jtype==urdf::Joint::FIXED || jtype==urdf::Joint::FLOATING) continue;

      if (joint->mimic) continue;

      if (joint->child_link_name.compare(next_link->name)) continue;

      joint_names.push_back(joint->name);

      if (jtype==urdf::Joint::CONTINUOUS)
      {
        joint_pos_lower_lim.push_back(-M_PI);
        joint_pos_upper_lim.push_back(M_PI);
      }
      else
      {
        joint_pos_lower_lim.push_back(joint->limits->lower);
        joint_pos_upper_lim.push_back(joint->limits->upper);
      }

      effort_lim.push_back(joint->limits->effort);
      joint_vel_lim.push_back(joint->limits->velocity);
    }
  }

  // create KDL::Chain and forward/inverse kinematics and Jacobian solvers
  KDL::Tree tree;
  kdl_parser::treeFromUrdfModel(urdf_model, tree);

  if (!tree.getChain(base_link_name, tool_link_name, chain))
  {
    throw std::runtime_error("[RobotArm Error]: Failed to create kdl chain from " + base_link_name + " to " + tool_link_name + " ...\n");
  }
  else
  {
    fk_solver.reset(new KDL::ChainFkSolverPos_recursive(chain));
    jac_solver.reset(new KDL::ChainJntToJacSolver(chain));
    ik_vel_solver.reset(new KDL::ChainIkSolverVel_pinv(chain));
    ik_solver.reset(new KDL::ChainIkSolverPos_NR(chain,*fk_solver,*ik_vel_solver,200,1e-6));
  }

  // preallocate space for all states
  N_JOINTS = joint_names.size();
  joint_pos.zeros(N_JOINTS);
  prev_joint_pos.zeros(N_JOINTS);
  Fext.zeros(6);
}

bool RobotArm::isOk() const
{
  return getMode()!=lwr4p_::Mode::PROTECTIVE_STOP;
}

void RobotArm::setJointLimitCheck(bool check)
{
  check_limits = check;
}

void RobotArm::setSingularityCheck(bool check)
{
  check_singularity = check;
}

void RobotArm::setSingularityThreshold(double thres)
{
  SINGULARITY_THRES = thres;
}

void RobotArm::setGetExternalWrenchFun(arma::vec (*getWrenchFun)(void))
{
  get_wrench_fun_ptr = std::bind(getWrenchFun);
}

void RobotArm::enable()
{
  mode = lwr4p_::Mode::IDLE;
  joint_pos = getJointsPosition();
  prev_joint_pos = joint_pos;
  update();
}

std::string RobotArm::getErrMsg() const
{
  return err_msg;
}

void RobotArm::addJointState(sensor_msgs::JointState &joint_state_msg)
{
  std::unique_lock<std::mutex> lck(robot_state_mtx);

  arma::vec j_pos = getJointsPosition();
  arma::vec j_vel = getJointsVelocity();

  for (int i=0;i<N_JOINTS;i++)
  {
    joint_state_msg.name.push_back(joint_names[i]);
    joint_state_msg.position.push_back(j_pos(i));
    joint_state_msg.velocity.push_back(j_vel(i));
    joint_state_msg.effort.push_back(0.0);
  }
}

double RobotArm::getCtrlCycle() const
{
  return ctrl_cycle;
}

lwr4p_::Mode RobotArm::getMode() const
{
  return mode;
}

std::string RobotArm::getModeName(Mode mode) const
{
  return (mode_name.find(mode))->second;
}

int RobotArm::getNumJoints() const
{
  return N_JOINTS;
}

bool RobotArm::setJointsTrajectory(const arma::vec &j_targ, double duration)
{
  // keep last known robot mode
  lwr4p_::Mode prev_mode = getMode();

  // initialize position
  arma::vec q0 = getJointsPosition();
  arma::vec qref = q0;

  // initalize time
  double t = 0.0;
  double Ts = getCtrlCycle();

  int iters = 0;

  // start conttroller
  setMode(lwr4p_::Mode::JOINT_POS_CONTROL);
  while (isOk() && getMode()!=lwr4p_::Mode::IDLE && t<duration)
  {
    update();
    t += Ts;
    qref = get5thOrder(t, q0, j_targ, duration)[0];
    setJointsPosition(qref);
  }

  bool reached_target = t>=duration;

  if (isOk() && getMode()==lwr4p_::Mode::JOINT_POS_CONTROL) setMode(prev_mode);

  return reached_target;
}

arma::vec posError(const arma::vec &p1, const arma::vec &p2)
{
  return p1-p2;
}

arma::vec orientError(const arma::mat &R1, const arma::mat &R2)
{
  return (lwr4p_::rotm2quat(R1).subvec(1,3) - lwr4p_::rotm2quat(R2).subvec(1,3));
}

bool RobotArm::setTaskTrajectory(const arma::mat &target_pose, double duration)
{
  bool found_solution = false;
  // arma::vec j_pos0 = joint_pos;
  arma::vec j_pos0 = arma::vec().zeros(N_JOINTS);

  // find an initial joint configuration that yiels a pose close to target_pose
  for (int i=N_JOINTS-1; i>-1; i--)
  {
    int n_div = 8;
    arma::vec jp = arma::linspace<arma::vec>(joint_pos_lower_lim[i], joint_pos_upper_lim[i], n_div);
    jp = jp.subvec(1,n_div-2);
    arma::vec j_pos = j_pos0;
    for (int j=0; j<jp.size() ; j++)
    {
      j_pos[i] = jp[j];
      arma::mat pose0 = getTaskPose(j_pos0);
      arma::mat pose = getTaskPose(j_pos);
      double e0 = arma::norm(lwr4p_::posError(pose0.submat(0,3,2,3),target_pose.submat(0,3,2,3))) + arma::norm(lwr4p_::orientError(pose0.submat(0,0,2,2),target_pose.submat(0,0,2,2)));
      double e = arma::norm(lwr4p_::posError(pose.submat(0,3,2,3),target_pose.submat(0,3,2,3))) + arma::norm(lwr4p_::orientError(pose.submat(0,0,2,2),target_pose.submat(0,0,2,2)));
      if (e < e0) j_pos0 = j_pos;
    }
  }

  arma::vec j_target = getJointsPosition(target_pose, j_pos0, &found_solution);

  if (found_solution)
  {
    // check if the found solution respects the joint limits
    for (int i=0; i<j_target.size(); i++)
    {
      if (j_target(i)>joint_pos_upper_lim[i] || j_target(i)<joint_pos_lower_lim[i]) return false;
    }
    return setJointsTrajectory(j_target, duration);
  }


  return false;
}

void RobotArm::setJointsPositionHelper(const arma::vec &j_pos)
{
  arma::vec current_j_pos = joint_pos; // getJointsPosition();
  arma::vec dj_pos = (j_pos - current_j_pos) / ctrl_cycle;

  if (check_limits)
  {
    if (!checkJointPosLimits(j_pos)) return;
    if (!checkJointVelLimits(dj_pos)) return;
  }

  if (check_singularity)
  {
    if (!checkSingularity()) return;
  }

  std::unique_lock<std::mutex> lck(robot_state_mtx);

  prev_joint_pos = current_j_pos;
  joint_pos = j_pos;
}

void RobotArm::setJointsVelocityHelper(const arma::vec &j_vel)
{
  setJointsPositionHelper(getJointsPosition() + j_vel*ctrl_cycle);
}

void RobotArm::setTaskVelocityHelper(const arma::vec &task_vel)
{
  setJointsVelocityHelper(arma::solve(getJacobian(),task_vel));
}

arma::vec RobotArm::getJointsPosition() const
{
  // std::unique_lock<std::mutex> lck(robot_state_mtx);
  return joint_pos;
}

arma::vec RobotArm::getJointsPosition(const arma::mat &pose, const arma::vec &q0, bool *found_solution) const
{
  KDL::JntArray jnt(N_JOINTS);
  KDL::JntArray jnt0(N_JOINTS);

  for (int i=0;i<N_JOINTS;i++) jnt0(i) = q0(i);

  KDL::Frame kdl_pose;
  for (int i=0;i<3;i++)
  {
    kdl_pose.p[i] = pose(i,3);
    for (int j=0;j<3;j++) kdl_pose.M(i,j) = pose(i,j);
  }

  int ret = ik_solver->CartToJnt(jnt0,kdl_pose,jnt);

  if (found_solution) *found_solution = ret >= 0;

  arma::vec q = arma::vec().zeros(N_JOINTS);

  if (ret>=0)
  {
    for (int i=0;i<N_JOINTS;i++) q(i) = jnt(i);
  }

  return q;
}

arma::vec RobotArm::getJointsVelocity() const
{
  // std::unique_lock<std::mutex> lck(robot_state_mtx);
  return (joint_pos - prev_joint_pos)/getCtrlCycle();
}

arma::mat RobotArm::getTaskPose() const
{
  // std::unique_lock<std::mutex> lck(robot_state_mtx);
  return getTaskPose(getJointsPosition());
}

arma::mat RobotArm::getTaskPose(const arma::vec &j_pos) const
{
  arma::mat task_pose(4,4);

  KDL::JntArray jnt(N_JOINTS);
  for (int i=0;i<N_JOINTS;i++) jnt(i) = j_pos(i);

  KDL::Frame fk;
  fk_solver->JntToCart(jnt, fk);
  for (int i=0;i<3;i++)
  {
    for (int j=0;j<4;j++) task_pose(i,j) = fk(i,j);
  }
  task_pose.row(3) = arma::rowvec({0,0,0,1});

  return task_pose;
}

arma::vec RobotArm::getTaskPosition() const
{
  // std::unique_lock<std::mutex> lck(robot_state_mtx);
  return getTaskPose().submat(0,3,2,3);
}

arma::vec RobotArm::getTaskOrientation() const
{
  // std::unique_lock<std::mutex> lck(robot_state_mtx);
  arma::mat R = getTaskPose().submat(0,0,2,2);
  arma::vec quat = rotm2quat(R);
  return quat;
}

arma::mat RobotArm::getJacobian() const
{
  // std::unique_lock<std::mutex> lck(robot_state_mtx);
  return getJacobian(getJointsPosition());
}

arma::mat RobotArm::getJacobian(const arma::vec &j_pos) const
{
  KDL::JntArray jnt(N_JOINTS);
  for (int i=0;i<N_JOINTS;i++) jnt(i) = j_pos(i);

  KDL::Jacobian J(N_JOINTS);
  jac_solver->JntToJac(jnt, J);
  arma::mat Jac(6, N_JOINTS);
  for (int i=0;i<Jac.n_rows;i++)
  {
    for (int j=0;j<Jac.n_cols;j++) Jac(i,j) = J(i,j);
  }

  return Jac;
}

arma::mat RobotArm::getEEJacobian() const
{
  // std::unique_lock<std::mutex> lck(robot_state_mtx);
  arma::mat R = getTaskPose().submat(0,0,2,2);
  arma::mat Jrobot = getJacobian();
  arma::mat Jee(6, N_JOINTS);
  Jee.submat(0,0,2,N_JOINTS-1) = R * Jrobot.submat(0,0,2,N_JOINTS-1);
  Jee.submat(3,0,5,N_JOINTS-1) = R * Jrobot.submat(3,0,5,N_JOINTS-1);

  return Jee;
}

arma::vec RobotArm::getJointsTorque() const
{
  // std::unique_lock<std::mutex> lck(robot_state_mtx);
  return getJacobian().t() * getExternalWrench();
}

arma::vec RobotArm::getExternalWrench() const
{
  // std::unique_lock<std::mutex> lck(robot_state_mtx);
  return get_wrench_fun_ptr();
}

bool RobotArm::checkJointPosLimits(const arma::vec &j_pos)
{
  for (int i=0;i<N_JOINTS;i++)
  {
    if (j_pos(i)>joint_pos_upper_lim[i] || j_pos(i)<joint_pos_lower_lim[i])
    {
      std::ostringstream out;
      out << joint_names[i] << ": position limit reached: " << j_pos(i) << " rad";
      err_msg = out.str();
      // print_err_msg(err_msg);
      setMode(lwr4p_::Mode::PROTECTIVE_STOP);
      return false;
    }
  }

  return true;
}

bool RobotArm::checkJointVelLimits(const arma::vec &dj_pos)
{
  for (int i=0;i<N_JOINTS;i++)
  {
    if (std::fabs(dj_pos(i))>joint_vel_lim[i])
    {
      std::ostringstream out;
      out << joint_names[i] << ": velocity limit reached: " << dj_pos(i) << " rad/s";
      err_msg = out.str();
      // print_err_msg(err_msg);
      setMode(lwr4p_::Mode::PROTECTIVE_STOP);
      return false;
    }
  }

  return true;
}

bool RobotArm::checkSingularity()
{

  if (getMode()!=lwr4p_::Mode::CART_VEL_CONTROL &&
      getMode()!=lwr4p_::Mode::CART_VEL_CONTROL) return true;

  bool singularity_reached = false;

  arma::vec eigval; // = arma::eig_gen(Jrobot);
  arma::svd(eigval, getJacobian());

  if (arma::min(arma::abs(eigval)) < SINGULARITY_THRES) singularity_reached = true;

  if (singularity_reached)
  {
    err_msg = "Singularity reached!";
    // print_err_msg(err_msg);
    setMode(lwr4p_::Mode::PROTECTIVE_STOP);
    return false;
  }

  return true;
}

void RobotArm::readWrenchFromTopic(const std::string &topic)
{
  wrench_reader.reset(new WrenchReader(topic));
  setGetExternalWrenchFun(&WrenchReader::getWrench, wrench_reader.get());
}



}; // namespace lwr4p_

}; // namespace as64_
