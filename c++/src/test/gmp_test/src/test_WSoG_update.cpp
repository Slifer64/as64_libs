#include <ros/ros.h>
#include <ros/package.h>
#include <memory>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <map>
#include <exception>

#include <io_lib/io_utils.h>
#include <gmp_lib/WSoG/WSoG.h>
// #include <gmp_test/utils.h>

using namespace as64_;

struct Point
{
  double t;
  double x, x_dot, x_ddot;
  double p, p_dot, p_ddot;

  bool update_pos;
  bool update_vel;
  bool update_accel;

  Point()
  {
    t = 0;
    x = x_dot = x_ddot = 0;
    p = p_dot = p_ddot = 0;
    update_pos = update_vel = update_accel = false;
  }

  void setPos(double p)
  {
    this->p = p;
    update_pos = true;
  }

  void setVel(double p_dot)
  {
    this->p_dot = p_dot;
    update_vel = true;
  }

  void setAccel(double p_ddot)
  {
    this->p_ddot = p_ddot;
    update_accel = true;
  }

  void scale(double ks, double kt, double T, double y0, double y0d)
  {
    t /= kt;

    x = t/T;
    x_dot = 1/T;
    x_ddot = 0;

    if (update_pos) p = ks*(p - y0d) + y0;
    if (update_vel) p_dot *= ks;
    if (update_accel) p_ddot *= ks;
  }

  void save(std::ostream &out) const
  {
    arma::vec data = arma::vec( {t, x, x_dot, x_ddot, p, p_dot, p_ddot, (double)update_pos, (double)update_vel, (double)update_accel} );
    io_::write_mat(data, out);
  }
};

void updateWSoG(std::shared_ptr<gmp_::WSoG> wsog, const Point &s);
void loadParams();

std::string train_data_file;
std::string sim_data_file;
std::string train_method;
int N_kernels;
double kernels_std_scaling;
double ks;
double kt;
int N_points;
std::vector<Point> points;

int main(int argc, char** argv)
{
  // ===========  Initialize the ROS node  ===============
  ros::init(argc, argv, "test_WSoG_cond_node");

  loadParams();

  // ===========  Load training data  ===============
  std::ifstream in(train_data_file, std::ios::in | std::ios::binary);
  if (!in) throw std::runtime_error("Failed to open file \"" + train_data_file + "\"...");

  arma::rowvec Timed;
  arma::rowvec yd_data;
  arma::rowvec dyd_data;
  arma::rowvec ddyd_data;

  io_::read_mat(Timed, in, true);
  io_::read_mat(yd_data, in, true);
  io_::read_mat(dyd_data, in, true);
  io_::read_mat(ddyd_data, in, true);
  in.close();

  double Ts = Timed(1)-Timed(0);

  // ===========  initialize gmp  ===============
  std::shared_ptr<gmp_::WSoG> wsog( new gmp_::WSoG(N_kernels, kernels_std_scaling) );

  arma::wall_clock timer;
  timer.tic();
  std::cout << "WSoG training...\n";
  double offline_train_mse;
  int i_end = Timed.size()-1;
  arma::rowvec x = Timed/Timed(i_end);
  wsog->train(train_method, x, yd_data, &offline_train_mse);
  std::cout << "offline_train_mse = " << offline_train_mse << "\n";
  std::cout << "Elapsed time: " << timer.toc() << " sec\n";

  // ===========  gmp update and simulation  ===============
  double y0d = yd_data(0);
  double ygd = yd_data(i_end);
  double y0 = y0d;
  double yg = ks*(ygd-y0d) + y0;
  double T = Timed(i_end)/kt;
  double x_dot = 1/T;
  arma::rowvec Time = Timed/kt;
  double x_ddot = 0;

  wsog->setStartValue(y0);
  wsog->setFinalValue(yg);

  int N = x.size();
  arma::rowvec y_data(N);
  arma::rowvec dy_data(N);
  arma::rowvec ddy_data(N);

  for (int i=0; i<N_points; i++) points[i].scale(ks, kt, T, y0, y0d);
  for (int i=0; i<N_points; i++) updateWSoG(wsog, points[i]);

  // simulate
  for (int i=0; i<N; i++)
  {
    y_data(i) = wsog->output(x(i));
    dy_data(i) = wsog->outputDot(x(i), x_dot);
    ddy_data(i) = wsog->outputDDot(x(i), x_dot, x_ddot);
  }

  // obtain scaled demo data
  Timed = Timed / kt;
  yd_data = ks*(yd_data-y0d) + y0;
  dyd_data = ks*kt*dyd_data;
  ddyd_data = ks*std::pow(kt,2)*ddyd_data;

  // ===========  write results  ===============
  std::ofstream out(sim_data_file, std::ios::out | std::ios::binary);
  if (!out) throw std::runtime_error("Failed to create file \"" + sim_data_file + "\"...");
  io_::write_mat(Timed, out, true);
  io_::write_mat(yd_data, out, true);
  io_::write_mat(dyd_data, out, true);
  io_::write_mat(ddyd_data, out, true);
  io_::write_mat(Time, out, true);
  io_::write_mat(y_data, out, true);
  io_::write_mat(dy_data, out, true);
  io_::write_mat(ddy_data, out, true);
  io_::write_scalar((long)N_points, out, true);
  for (int i=0; i<N_points; i++) points[i].save(out);
  out.close();

  // ===========  Shutdown ROS node  ==================
  ros::shutdown();

  return 0;
}


void updateWSoG(std::shared_ptr<gmp_::WSoG> wsog, const Point &s)
{
  if (s.update_pos & !s.update_vel& !s.update_accel)
    wsog->updatePos(s.x, s.p);
  else if (!s.update_pos & s.update_vel& !s.update_accel)
    wsog->updateVel(s.x, s.x_dot, s.p_dot);
  else if (!s.update_pos & !s.update_vel& s.update_accel)
    wsog->updateAccel(s.x, s.x_dot, s.x_ddot, s.p_ddot);
  else if (s.update_pos & s.update_vel& !s.update_accel)
    wsog->updatePosVel(s.x, s.x_dot, s.p, s.p_dot);
  else if (s.update_pos & !s.update_vel& s.update_accel)
    wsog->updatePosAccel(s.x, s.x_dot, s.x_ddot, s.p, s.p_ddot);
  else if (!s.update_pos & s.update_vel& s.update_accel)
    wsog->updateVelAccel(s.x, s.x_dot, s.x_ddot, s.p_dot, s.p_ddot);
  else if (s.update_pos & s.update_vel& s.update_accel)
    wsog->updatePosVelAccel(s.x, s.x_dot, s.x_ddot, s.p, s.p_dot, s.p_ddot);

}

void loadParams()
{
  ros::NodeHandle nh_("~");

  // ===========  Read params  ===============
  std::string path = ros::package::getPath("gmp_test") + "/matlab/data/";

  if (!nh_.getParam("train_data_file", train_data_file)) train_data_file = "wsog_train_data.bin";
  if (!nh_.getParam("sim_data_file", sim_data_file)) sim_data_file = "wsog_update_sim_data.bin";
  if (!nh_.getParam("train_method", train_method)) train_method = "LWR";
  if (!nh_.getParam("N_kernels", N_kernels)) N_kernels = 30;
  if (!nh_.getParam("kernels_std_scaling", kernels_std_scaling)) kernels_std_scaling = 2;
  if (!nh_.getParam("ks", ks)) ks = 1.0;
  if (!nh_.getParam("kt", kt)) kt = 1.0;

  train_data_file = path + train_data_file;
  sim_data_file = path + sim_data_file;

  // load points
  if (!nh_.getParam("N_points", N_points)) N_points = 0;
  points.resize(N_points);
  for (int k=0; k<N_points; k++)
  {
    std::string name = "s" + std::to_string(k+1); // find params si
    std::string str;
    if (!nh_.getParam(name, str)) continue; // if didn't find it then go on with next si

    str = str + ","; // add one extra separator at the end to parse also the last name-value pair
    std::map<std::string, double> mp;  // store name-value pairs for t, p, p_dot, p_ddot in si
    int i1 = 0;
    for (int i=0; i<str.length(); i++)
    {
      if (str[i] == ',') // found a separator
      {
        std::string temp = str.substr(i1, i-i1); // extract name-value pair
        i1 = i+1;
        for (int j=0; j<temp.length(); j++) // replace separator ':' in name value-pair with ' '
        {
          if (temp[j] == ':')
          {
            temp[j] = ' ';
            break;
          }
        }
        std::istringstream iss(temp); // read the name and the value from iss
        std::string key;
        double val;
        iss >> key >> val;
        mp[key] = val; // pass them to the map
      }
    }
    auto it = mp.find("t");
    if (it != mp.end()) points[k].t = it->second;
    if ( (it = mp.find("p")) != mp.end()) points[k].setPos(it->second);
    if ( (it = mp.find("p_dot")) != mp.end()) points[k].setVel(it->second);
    if ( (it = mp.find("p_ddot")) != mp.end()) points[k].setAccel(it->second);
  }
}

