#include <ros/ros.h>
#include <ros/package.h>
#include <memory>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <exception>
#include <signal.h>

#include <plot_lib/qt_plot.h>

using namespace as64_;

int main(int argc, char** argv)
{
  // ===========  Initialize the ROS node  ==================
  ros::init(argc, argv, "IO_test_node");
  ros::NodeHandle nh_("~");

  std::string path = ros::package::getPath("plot_test") + "/";

  // ===========  Create data  ==================
  int n_data = 1000;
  arma::rowvec Timed(n_data), Time(n_data);
  arma::mat Pd_data(3, n_data), P_data(3,n_data);
  double dt = 0.002;

  for (int j=0; j<n_data; j++)
  {
    Timed(j) = Time(j) = j*dt;
    Pd_data.col(j) = arma::vec( { 0.5*std::cos(j*dt), 0.2*std::sin(j*dt), 0.35*std::cos(j*dt) });
    P_data.col(j) = arma::vec( { 0.45*std::cos(j*dt), 0.16*std::sin(j*dt), 0.31*std::cos(j*dt) });
  }

  // ===========  Init Qt plot  ==================

  std::cerr << "file: " << __FILE__ << "\nfunction: " << __FUNCTION__ << "\nline: " << __LINE__ << "\n";

  std::cerr << "[main]: Calling init...\n";
  pl_::QtPlot::init();
  std::cerr << "[main]: Init DONE!\n";

  pl_::Figure *fig;
  pl_::Axes *ax;
  pl_::Graph *graph;

  // ===========  Plot data  ==================
  fig = pl_::QtPlot::figure();
  fig->setAxes(3,1);

  std::vector<std::string> py_labels = {"x", "y", "z"};
  for (int i=0; i<3; i++)
  {
    ax = fig->getAxes(i);
    ax->hold(true);
    ax->plot(Time, P_data.row(i), pl_::Color_,pl_::BLUE, pl_::LineStyle_,pl_::SolidLine);
    ax->plot(Timed, Pd_data.row(i), pl_::Color_,pl_::GREEN, pl_::LineStyle_,pl_::DashLine);
    // graph->setColor(pl_::GREEN);
    // graph->setLineStyle(pl_::DashLine);
    ax->ylabel(py_labels[i]);
    if (i==0)
    {
      ax->legend({"sim", "demo"});
      ax->title("Cartesian Position [m]");
    }
    if (i==2) ax->xlabel("time [s]");
    ax->drawnow();
  }


  fig = pl_::QtPlot::figure();
  fig->setAxes(2,3);

  for (int j=0; j<3; j++)
  {
    ax = fig->getAxes(0,j);
    ax->hold(true);
    ax->plot(Time, P_data.row(j), pl_::Color_,pl_::BLUE, pl_::LineStyle_,pl_::SolidLine);
    ax->xlabel("time [s]");
    ax->drawnow();
  }

  for (int j=0; j<3; j++)
  {
    ax = fig->getAxes(1,j);
    ax->hold(true);
    ax->plot(Timed, Pd_data.row(j), pl_::Color_,pl_::GREEN, pl_::LineStyle_,pl_::DashLine);
    ax->xlabel("time [s]");
    ax->drawnow();
  }

  // ======================================================

  std::string temp;
  std::cerr << "Program paused. Press enter to continue...\n";
  std::getline(std::cin, temp);

  pl_::QtPlot::terminate();

  // ===========  Shutdown ROS node  ==================
  ros::shutdown();

  return 0;
}
