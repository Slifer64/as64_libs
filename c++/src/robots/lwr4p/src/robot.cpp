#include <lwr4p/robot.h>
#include <iostream>
#include <string>

namespace as64_
{

  namespace lwr4p_
  {

    Robot::Robot(const char *path_to_FRI_init)
    {
      initRobot();
    }

    Robot::Robot(urdf::Model &urdf_model, const std::string &base_link, const std::string &tool_link,
                 double ctrl_cycle, const char *path_to_FRI_init):
        RobotArm(urdf_model, base_link, tool_link, ctrl_cycle)
    {
      initRobot(path_to_FRI_init);
    }

    Robot::Robot(const std::string &robot_desc_param, const std::string &base_link, const std::string &tool_link,
                 double ctrl_cycle, const char *path_to_FRI_init):
        RobotArm(robot_desc_param, base_link, tool_link, ctrl_cycle)
    {
      initRobot(path_to_FRI_init);
    }

    void Robot::initRobot(const char *path_to_FRI_init)
    {
      if (path_to_FRI_init == NULL)
        FRI.reset(new FastResearchInterface("/home/user/lwr/980500-FRI-Driver.init"));
      else
        FRI.reset(new FastResearchInterface(path_to_FRI_init));

      ctrl_cycle = FRI->GetFRICycleTime();
      stop();  // Initially the robot is stopped
      this->mode = lwr4p_::Mode::IDLE;
      // startJointPositionController();

      // preallocate memory for jacobian
      jacob_temp = reinterpret_cast<float**>(malloc(sizeof(float *) * 6));
      for (size_t i = 0; i < 6; i++) {
        jacob_temp[i] = reinterpret_cast<float*>(malloc(sizeof(float) * 7));
      }

      joint_pos = getJointsPositionFromFRI();
      prev_joint_pos = joint_pos;

      setGetExternalWrenchFun(&Robot::getExternalWrenchImplementation, this);
    }

    Robot::~Robot()
    {
      FRI->StopRobot();

      for (size_t i = 0; i < 6; i++) free(jacob_temp[i]);
      free(jacob_temp);
    }

    void Robot::setMode(const lwr4p_::Mode &m)
    {
      if (getMode() == m) return;

      if (m == PROTECTIVE_STOP)
      {
        protectiveStop();
        return;
      }

      stop();
      if (!isOk()) return;

      switch (m)
      {
        case lwr4p_::Mode::IDLE:
          break;
        case lwr4p_::Mode::JOINT_POS_CONTROL:
        case lwr4p_::Mode::JOINT_VEL_CONTROL:
        case lwr4p_::Mode::CART_VEL_CONTROL:
          startJointPositionController();
          break;
        case lwr4p_::Mode::FREEDRIVE:
        case lwr4p_::Mode::JOINT_TORQUE_CONTROL:
          startJointTorqueController();
          break;
        case lwr4p_::Mode::CART_IMPEDANCE_CONTROL:
          startCartImpController();
          break;
      }
      mode = m;
    }

    void Robot::update()
    {
      FRI->WaitForKRCTick();

      if (getMode()!=lwr4p_::IDLE && !FRI->IsMachineOK()) protectiveStop();

      switch (getMode()) {
        case IDLE:
        case PROTECTIVE_STOP:
          break;
        case FREEDRIVE:
          setZeroJointTorques();
          break;
        case JOINT_POS_CONTROL:
        case JOINT_VEL_CONTROL:
        case CART_VEL_CONTROL:
          break;
        case JOINT_TORQUE_CONTROL:
        case CART_IMPEDANCE_CONTROL:
          prev_joint_pos = joint_pos;
          joint_pos = getJointsPositionFromFRI();
          break;
      }
    }

    void Robot::setCartStiffness(const arma::vec &input)
    {
      if (getMode() == lwr4p_::Mode::CART_IMPEDANCE_CONTROL)
      {
        if (FRI->GetFRIMode() == FRIStateSet::FRI_STATE_MON)
        {
          print_warn_msg("[LWR4+::setCartStiffness]: Cartesian wrench is commanded with closed controller.\n");
          print_warn_msg("Opening controller ...\n");
          // start te controller
          startCartImpController();
          // wait one tick
          FRI->WaitForKRCTick();
        }
        // temp variables
        static float temp[6];
        // put the values from arma to float[]
        for (int i = 0; i < 6; i++) {
          temp[i] = input(i);
        }

        // set value
        FRI->SetCommandedCartStiffness(temp);
      }
      else
      {
        print_warn_msg("[LWR4+::setCartStiffness]: Available only in \"" + getModeName(lwr4p_::CART_IMPEDANCE_CONTROL) + "\" mode.");
      }
    }

    void Robot::setCartDamping(const arma::vec &input)
    {
      if (getMode() == lwr4p_::Mode::CART_IMPEDANCE_CONTROL)
      {
        if (FRI->GetFRIMode() == FRIStateSet::FRI_STATE_MON)
        {
          print_warn_msg("[LWR4+::setCartDamping]: Cartesian wrench is commanded with closed controller.\n");
          print_warn_msg("[LWR4+::setCartDamping]: Opening controller ...\n");
          // start te controller
          startCartImpController();
          // wait one tick
          FRI->WaitForKRCTick();
        }
        // temp variables
        static float temp[6];
        // put the values from arma to float[]
        for (int i = 0; i < 6; i++) {
          temp[i] = input(i);
        }

        // set value
        FRI->SetCommandedCartDamping(temp);
      }
      else
      {
        print_warn_msg("[LWR4+::setCartDamping]: Available only in \"" + getModeName(lwr4p_::CART_IMPEDANCE_CONTROL) + "\" mode.");
      }
    }

    void Robot::startJointPositionController()
    {
      // wait one tick
      FRI->WaitForKRCTick();
      // std::cout << "[JointPosController::startController] Starting joint position control." << std::endl;
      int ResultValue = FRI->StartRobot(FastResearchInterface::JOINT_POSITION_CONTROL);
      this->mode = lwr4p_::Mode::JOINT_POS_CONTROL;
      // if there s a problem
      if ((ResultValue != 0) && (ResultValue != EALREADY))
      {
        print_err_msg("[LWR4+::startJointPositionController] Error occurred! The controller will put in \"" + getModeName(lwr4p_::IDLE) + "\" mode.");
        stop();
      }
      // std::cout << "[JointPosController::startController] " << "Finished" << std::endl;
    }

    void Robot::startJointTorqueController()
    {
      // wait one tick
      FRI->WaitForKRCTick();
      // temp variables
      static float stiffness[7];
      static float stiffnessCart[6];
      static float damping[7];
      static float dampingCart[6];
      static float torques[7];
      static float q[7];
      // put zeros everywhere
      for (int i = 0; i < 7; i++) {
        stiffness[i] = 0;
        damping[i] = 0;
        torques[i] = 0;
      }
      for (int i = 0; i < 6; i++) {
        stiffnessCart[i] = 0;
        dampingCart[i] = 0;
      }

      // set stiffness to zero
      FRI->SetCommandedJointStiffness(stiffness);
      // set stiffness to zero
      FRI->SetCommandedCartStiffness(stiffnessCart);
      // set damping to zero
      FRI->SetCommandedJointDamping(damping);
      // set damping to zero
      FRI->SetCommandedCartDamping(dampingCart);
      // set additional torques to zero
      FRI->SetCommandedJointTorques(torques);
      // set commanded joint positions to current
      FRI->GetCommandedJointPositions(q);
      FRI->SetCommandedJointPositions(q);
      // std::cout << "[KukaTorqueController::startController] Starting torque control." << std::endl;
      int ResultValue = FRI->StartRobot(FastResearchInterface::JOINT_IMPEDANCE_CONTROL);
      // if there s a problem
      this->mode = lwr4p_::Mode::JOINT_TORQUE_CONTROL;
      if ((ResultValue != 0) && (ResultValue != EALREADY))
      {
        print_err_msg("[LWR4+::startJointTorqueController] Error occurred! The controller will put in \"" + getModeName(lwr4p_::IDLE) + "\" mode.");
        stop();
      }

      // std::cout << "[KukaTorqueController::startController] " << "Finished" << std::endl;
    }

    void Robot::startCartImpController()
    {
      // wait one tick
      FRI->WaitForKRCTick();
      // temp variables
      static float stiffness[7];
      static float stiffnessCart[6];
      static float damping[7];
      static float dampingCart[6];
      static float torques[7];
      static float p[12];
      // put zeros everywhere
      for (int i = 0; i < 7; i++) {
        stiffness[i] = 0;
        damping[i] = 0;
        torques[i] = 0;
      }
      for (int i = 0; i < 6; i++) {
        stiffnessCart[i] = 0;
        dampingCart[i] = 0;
      }
      // set stiffness to zero
      FRI->SetCommandedJointStiffness(stiffness);
      // set stiffness to zero
      FRI->SetCommandedCartStiffness(stiffnessCart);
      // set damping to zero
      FRI->SetCommandedJointDamping(damping);
      // set damping to zero
      FRI->SetCommandedCartDamping(dampingCart);
      // set additional torques to zero
      FRI->SetCommandedJointTorques(torques);
      // set commanded pose to current (mirror values)
      FRI->GetMeasuredCartPose(p);
      FRI->SetCommandedCartPose(p);
      // std::cout << "[KukaCartImpedanceController::startController] Starting Cartesian Impedance control." << std::endl;
      int ResultValue = FRI->StartRobot(FastResearchInterface::CART_IMPEDANCE_CONTROL);
      // if there is a problem
      this->mode = lwr4p_::Mode::CART_IMPEDANCE_CONTROL;
      if ((ResultValue != 0) && (ResultValue != EALREADY))
      {
        print_err_msg("[LWR4+::startCartImpController] Error occurred! The controller will put in \"" + getModeName(lwr4p_::IDLE) + "\" mode.");
        stop();
        return;
      }
      // std::cout << "[KukaCartImpedanceController::startController] " << "Finished" << std::endl;
    }

    void Robot::stop()
    {
      if (getMode() == lwr4p_::Mode::IDLE) return;

      FRI->WaitForKRCTick();

      arma::vec q_current = getJointsPosition();
      setJointsPositionHelper(q_current);

      prev_joint_pos = getJointsPosition();

      if (!isOk()) protectiveStop();

      // printouts
      // std::cout << "[KukaController::stop] Stopping  control." << std::endl;

      static float pose[12];
      static float poseoff[12];

      static float q[7];
      static float qoff[7];
      static float torques[7];

      // set commanded joint positions to current commanded
      FRI->GetCommandedJointPositions(q);
      FRI->GetCommandedJointPositionOffsets(qoff);

      for (int i = 0; i < 7; i++)
      {
        q[i] += qoff[i];
        torques[i] = 0.0;

        joint_pos(i) = q[i];
      }
      prev_joint_pos = joint_pos;

      std::cout << "stop 22\n";

      FRI->SetCommandedJointPositions(q);

      // set commanded pose  to current commanded
      FRI->GetCommandedCartPose(pose);
      FRI->GetCommandedCartPoseOffsets(poseoff);
      for (int i = 0; i < 12; i++)
      {
        pose[i] += poseoff[i];
      }
      FRI->SetCommandedCartPose(pose);

      // set joint torques to zero
      FRI->SetCommandedJointTorques(torques);

      // call stanford command
      if (FRI->StopRobot() == 0)
      {
        this->mode = lwr4p_::Mode::IDLE;
      }
      else
      {
        protectiveStop();
        err_msg = "[LWR4+::stop]: Error occured!";
      }
    }

    void Robot::protectiveStop()
    {
      if (getMode() == lwr4p_::PROTECTIVE_STOP) return;

      FRI->StopRobot();

      joint_pos = getJointsPosition();
      prev_joint_pos = joint_pos;
      // update();
      mode = lwr4p_::Mode::PROTECTIVE_STOP;
      print_warn_msg("Mode changed to \"" + getModeName(getMode()) + "\"\n");
    }

  }  // namespace lwr4p_

} // namespace as64_
