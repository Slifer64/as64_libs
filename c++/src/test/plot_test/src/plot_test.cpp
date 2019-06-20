#include <ros/ros.h>
#include <ros/package.h>
#include <memory>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <exception>
#include <thread>
#include <condition_variable>
#include <mutex>
#include <signal.h>

#include <plot_lib/qt_plot.h>

using namespace as64_;

std::condition_variable qt_init_cond;
bool qt_init_flag;
std::mutex qt_init_mtx;

void runQTthread()
{
  int argc = 0;
  char **argv = 0;
  QApplication app(argc, argv);
  QThread::currentThread()->setPriority(QThread::LowestPriority);
  app.setQuitOnLastWindowClosed(false);
  //QMainWindow *win = new QMainWindow();
  //win->setVisible(false);

  pl_::QtPlot::init();

  {
    std::lock_guard<std::mutex> lck(qt_init_mtx);
    qt_init_flag = true;
    qt_init_cond.notify_one();
  }


  app.exec();

  std::cerr << "[runQTthread]: Finished exec!\n";
  //delete (win);
}

int main(int argc, char** argv)
{
  // ===========  Initialize the ROS node  ==================
  ros::init(argc, argv, "plot_test_node");
  ros::NodeHandle nh_("~");

  std::string path = ros::package::getPath("plot_test") + "/";

  // ===========  Launch Qt Thread  ==================
  bool launch_qt_thread;
  if (!nh_.getParam("launch_qt_thread",launch_qt_thread)) launch_qt_thread = false;
  if (launch_qt_thread)
  {
    qt_init_flag = false;
    std::thread(runQTthread).detach();
    {
      std::unique_lock<std::mutex> lck(qt_init_mtx);
      while (!qt_init_flag) qt_init_cond.wait(lck);
      qt_init_flag = false;
    }
  }


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
  //std::cerr << "[" << __FUNCTION__ << "]: Calling init...\n";
  pl_::QtPlot::init();
  //std::cerr << "[" << __FUNCTION__ << "]: Init DONE!\n";

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
      pl_::Legend *legend_elem = ax->legend({"sim", "demo"}, pl_::Color_,QColor(40,120,200), pl_::FontSize_,15, pl_::FontWeight_,pl_::DemiBold, pl_::FontFamily_,std::string("Ubuntu"));
      legend_elem->setAlignment(Qt::AlignRight | Qt::AlignTop);
      pl_::TextLabel *title_label = ax->title("Cartesian Position [m]", pl_::Color_,QColor(250,50,80), pl_::FontSize_,20, pl_::FontWeight_,pl_::Bold, pl_::FontFamily_,std::string("Ubuntu"));
      // title_label->setText("Cartesian Position [m]");
      // title_label->setProperty(pl_::Color_,QColor(250,50,80), pl_::FontSize_,20, pl_::FontWeight_,pl_::Bold, pl_::FontFamily_,std::string("Ubuntu"));
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
