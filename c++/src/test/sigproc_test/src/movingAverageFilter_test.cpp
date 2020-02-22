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
#include <sigproc_lib/movingAverageFilter.h>

#include <armadillo>

using namespace as64_;

// signal params
double T;
double dt;
double omega; // signal frequency
double A; // signal amplitude
double noise_std; // noise std

// moving average filter params
int n_samples; // filter window
double init_value; // initial value
double a; // forgetting factor

void loadParams()
{
  ros::NodeHandle nh("~");

  if (!nh.getParam("T",T)) throw std::ios_base::failure("Failed to load param \"T\"...\n");
  if (!nh.getParam("dt",dt)) throw std::ios_base::failure("Failed to load param \"dt\"...\n");
  if (!nh.getParam("omega",omega)) throw std::ios_base::failure("Failed to load param \"omega\"...\n");
  if (!nh.getParam("A",A)) throw std::ios_base::failure("Failed to load param \"A\"...\n");
  if (!nh.getParam("noise_std",noise_std)) throw std::ios_base::failure("Failed to load param \"noise_std\"...\n");

  if (!nh.getParam("n_samples",n_samples)) throw std::ios_base::failure("Failed to load param \"n_samples\"...\n");
  if (!nh.getParam("init_value",init_value)) throw std::ios_base::failure("Failed to load param \"init_value\"...\n");
  if (!nh.getParam("a",a)) throw std::ios_base::failure("Failed to load param \"a\"...\n");
}

int main(int argc, char** argv)
{
  // ===========  Initialize the ROS node  ==================
  ros::init(argc, argv, "movingAverageFilter_test_node");
  std::string path = ros::package::getPath("sigproc_test") + "/";

  loadParams();

  arma::arma_rng::set_seed(0);

  // ===========  Create data  ==================
  int n_data = std::ceil(T/dt);
  arma::rowvec Time(n_data);
  arma::rowvec y_data(n_data), yn_data(n_data), yfilt_data(n_data);

  for (int j=0; j<n_data; j++)
  {
    Time(j) = Time(j) = j*dt;
    y_data(j) = A*std::sin(omega*j*dt);
  }

  // ===========  Add noise  ==================
  yn_data = y_data + noise_std*arma::rowvec().randn(n_data);

  // ===========  Filter  ==================
  spl_::MovingAverageFilter ma_filt(n_samples, init_value, a);
  for (int j=0; j<n_data; j++) yfilt_data(j) = ma_filt.filter(yn_data(j));

  // ===========  Plot results  ==================
  pl_::QtPlot::init();

  pl_::Figure *fig;
  pl_::Axes *ax;
  pl_::Graph *graph;

  // ===========  Plot data  ==================
  fig = pl_::QtPlot::figure("Filtering", {600, 700});
  fig->setAxes(1,1);
  ax = fig->getAxes(0);
  ax->hold(true);
  ax->plot(Time, y_data, pl_::Color_,pl_::LIGHT_BROWN, pl_::LineStyle_,pl_::DashDotLine);
  ax->plot(Time, yn_data, pl_::Color_,pl_::BLUE, pl_::LineStyle_,pl_::SolidLine);
  ax->plot(Time, yfilt_data, pl_::Color_,pl_::MAGENTA, pl_::LineStyle_,pl_::SolidLine);
  ax->xlabel("time [s]", pl_::FontSize_,14);
  ax->legend({"y", "yn", "yfilt"}, pl_::Color_,QColor(40,120,200), pl_::FontSize_,15, pl_::FontWeight_,pl_::DemiBold, pl_::FontFamily_,std::string("Ubuntu"));
  ax->drawnow();

  // ======================================================

  std::string temp;
  std::cerr << "Program paused. Press enter to continue...\n";
  std::getline(std::cin, temp);

  pl_::QtPlot::terminate();

  // ===========  Shutdown ROS node  ==================
  ros::shutdown();

  return 0;
}
