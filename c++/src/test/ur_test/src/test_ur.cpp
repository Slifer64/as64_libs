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
#include <ur_robot/robot.h>

#include <io_utils.h>

using namespace as64_;

class Worker
{
public:
  Worker(const std::string &ip, int port)
  {
     robot.reset(new ur_::Robot(ip, port, 0.002));
  }

  void setFreeDriveMode()
  {
    robot->setFreedriveMode();
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
    // waits for the next tick
    robot->update();

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

      arma::vec q_dot_cmd = qref_dot + click*(qref-q);

      robot->setJointsVelocity(q_dot_cmd);

      // waits for the next tick
      robot->update();

      q = robot->getJointsPosition();
    }
    return true;
  }

  std::shared_ptr<as64_::ur_::Robot> robot;

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

  ros::NodeHandle nh("~");
  std::string robot_ip;
  if (!nh.getParam("robot_ip",robot_ip)) throw std::runtime_error("Failed to load param \"robot_ip\"...");

  // ===========  Create robot  ==================
  Worker worker(robot_ip, 50001);
  worker.setFreeDriveMode();

  arma::vec q1 = {-0.7827,  -1.2511,  -1.4126,  -1.6371,   1.6009,   0.0254};
  arma::vec q2 = {0.3761,  -1.8148,  -1.4127,  -2.7396,   1.6009,   0.0254};

  as64_::ur_::Robot robot(robot_ip, 50001, 0.002);

  while (ros::ok())
  {
//    worker.robot->update();
//    char ch = ' ';
//    ch = getch();
//    if (ch == 'f') worker.setFreeDriveMode();
//    if (ch == '1') worker.setJointsTrajectory(q1, 6);  //worker.robot->movej(q1, 1.4, 1.05, 8); //
//    if (ch == '2') worker.setJointsTrajectory(q2, 6); //worker.robot->movej(q1, 1.4, 1.05, 8); //
//    if (ch == 'j') worker.printJointsPos();
//    if (ch == 'q') throw std::runtime_error("Program termination request!");
  }

  // ===========  Shutdown ROS node  ==================
  // ros::shutdown();

  return 0;
}
