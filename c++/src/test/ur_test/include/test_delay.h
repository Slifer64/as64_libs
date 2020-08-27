#include <memory>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <thread>
#include <chrono>
#include <exception>

#include <armadillo>

#include <ros/ros.h>
#include <ros/package.h>

#include <io_lib/io_lib.h>
#include <ur_robot/ur_robot.h>

#include <io_utils.h>

using namespace as64_;

class Worker
{
public:
  Worker(const std::string &ip, int port)
  {
    robot.reset(new ur_::Robot("10.0.0.1", 50001));
  }

  void setFreeDriveMode()
  {
    robot->setMode(ur_::Mode::FREEDRIVE_MODE);
  }

  void printJointsPos()
  {
    std::thread([this]()
    {
      arma::vec jpos = robot->getJointsPosition();
      std::cout << "jpos = " << jpos.t() << "\n";
    }).detach();
  }

  bool setJointsTrajectory(const arma::vec &qT, double duration)
  {
    // keep last known robot mode
    ur_::Mode prev_mode = robot->getMode();
    // start controller
    robot->setMode(ur_::Mode::VELOCITY_CONTROL);

    // waits for the next tick
    robot->waitNextCycle();

    arma::vec q0 = robot->getJointsPosition();
    arma::vec q = q0;
    arma::vec qref = q0;
    arma::vec qref_dot;

    double t = 0.0;
    double click = 0.0;
    // the main while
    while (t < duration)
    {
      if (!robot->isOk())
      {
        std::cerr << robot->getErrMsg();
        return false;
      }

      // compute time now
      t += robot->getCtrlCycle();
      // update trajectory
      arma::mat ref_traj =  get5thOrder(t, q0, qT, duration);
      qref = ref_traj.col(0);
      qref_dot = ref_traj.col(1);

      arma::vec q_dot_cmd = qref_dot; // + click*(qref-q);

      robot->setJointsVelocity(q_dot_cmd, qref);

      // waits for the next tick
      robot->waitNextCycle();

      q = robot->getJointsPosition();
    }

    // reset last known robot mode
    robot->setMode(prev_mode);

    return true;
  }

  void executeTrajectory()
  {
    arma::vec q;
    arma::vec q_start = {-0.7827,  -1.2511,  -1.4126,  -1.6371,   1.6009,   0.0254};
    arma::vec q_end = {0.3761,  -1.8148,  -1.4127,  -2.7396,   1.6009,   0.0254};

    // Set the robot in position control mode
    robot->setMode(ur_::Mode::POSITION_CONTROL);
    std::cout << io_::bold << io_::green << "=== The robot is in position control ===\n" << io_::reset;

    if (!ros::ok()) exit(-1);

    robot->waitNextCycle();
    q = robot->getJointsPosition();

    double duration = 6.5;
    std::cout << io_::bold << io_::cyan << "The robot will move to its initial pose in " << duration << " sec.\n" << io_::reset;
    robot->setJointsTrajectory(q_start, duration);
    std::cout << io_::bold << io_::cyan << "Initial pose reached!\n" << io_::reset;
    if (!ros::ok()) exit(-1);

    robot->waitNextCycle();
    q = robot->getJointsPosition();
    duration = 6.5;
    std::cout << io_::bold << io_::cyan << "The robot will move to its final pose in " << duration << " sec.\n" << io_::reset;
    robot->startLogging();
    robot->setJointsTrajectory(q_end, duration);
    robot->stopLogging();
    std::cout << io_::bold << io_::cyan << "*** Final pose reached! ***\n" << io_::reset;

    robot->saveLoggedData(ros::package::getPath("ur_test") + "/data/logged_data.bin");
  }

  std::shared_ptr<ur_::Robot> robot;

private:


  arma::mat get5thOrder(double t, arma::vec p0, arma::vec pT, double totalTime)
  {
    arma::mat retTemp = arma::zeros<arma::mat>(p0.n_rows, 3);

    if (t < 0)
    {
      // before start
      retTemp.col(0) = p0;
    }
    else if (t > totalTime)
    {
      // after the end
      retTemp.col(0) = pT;
    }
    else
    {
      // somewhere betweeen ...
      // position
      retTemp.col(0) = p0 +
                       (pT - p0) * (10 * pow(t / totalTime, 3) -
                       15 * pow(t / totalTime, 4) +
                       6 * pow(t / totalTime, 5));
      // vecolity
      retTemp.col(1) = (pT - p0) * (30 * pow(t, 2) / pow(totalTime, 3) -
                       60 * pow(t, 3) / pow(totalTime, 4) +
                       30 * pow(t, 4) / pow(totalTime, 5));
      // acceleration
      retTemp.col(2) = (pT - p0) * (60 * t / pow(totalTime, 3) -
                       180 * pow(t, 2) / pow(totalTime, 4) +
                       120 * pow(t, 3) / pow(totalTime, 5));
    }

    // return vector
    return retTemp;
  }
};

int main(int argc, char** argv)
{
  // ===========  Initialize the ROS node  ==================
  ros::init(argc, argv, "test_ur");

  // ===========  Create robot  ==================
  Worker worker("10.0.0.1", 50001);
  worker.setFreeDriveMode();

  arma::vec q1 = {-0.7827,  -1.2511,  -1.4126,  -1.6371,   1.6009,   0.0254};
  arma::vec q2 = {0.3761,  -1.8148,  -1.4127,  -2.7396,   1.6009,   0.0254};

  while (ros::ok())
  {
    worker.robot->waitNextCycle();
    char ch = ' ';
    ch = getch();
    if (ch == 'f') worker.setFreeDriveMode();
    if (ch == 't') worker.executeTrajectory();
    if (ch == '1') worker.robot->movej(q1, 1.4, 1.05, 8); //worker.setJointsTrajectory(q1, 6);
    if (ch == '2') worker.robot->movej(q1, 1.4, 1.05, 8); //worker.setJointsTrajectory(q2, 6);
    if (ch == 'j') worker.printJointsPos();
    if (ch == 'q') break;
  }

  // ===========  Shutdown ROS node  ==================
  // ros::shutdown();

  return 0;
}
