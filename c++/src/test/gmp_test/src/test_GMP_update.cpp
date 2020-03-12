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
#include <gmp_lib/GMP/GMP.h>
// #include <gmp_test/utils.h>

using namespace as64_;

#define TEST_GMP_UPDATE_DEBUG_

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

void updateGMP(std::shared_ptr<gmp_::GMP> gmp, const Point &point);
void loadParams();

std::string train_data_file;
std::string sim_data_file;
std::string train_method;
int N_kernels;
double D;
double K;
double kernels_std_scaling;
double ks;
double kt;
int N_points;
std::vector<Point> points;

int main(int argc, char** argv)
{
  #ifdef TEST_GMP_UPDATE_DEBUG_
  try{
  #endif

  // ===========  Initialize the ROS node  ===============
  ros::init(argc, argv, "test_GMP_cond_node");

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
  std::shared_ptr<gmp_::GMP> gmp( new gmp_::GMP(N_kernels, D, K, kernels_std_scaling) );

  arma::wall_clock timer;
  timer.tic();
  std::cout << "GMP training...\n";
  double offline_train_mse;
  int i_end = Timed.size()-1;
  arma::rowvec x = Timed/Timed(i_end);
  gmp->train(train_method, x, yd_data, &offline_train_mse);
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

  gmp->setY0(y0);
  gmp->setGoal(yg);

  int N = x.size();
  arma::rowvec y_data(N);
  arma::rowvec dy_data(N);
  arma::rowvec ddy_data(N);

  for (int i=0; i<N_points; i++) points[i].scale(ks, kt, T, y0, y0d);
  for (int i=0; i<N_points; i++) updateGMP(gmp, points[i]);


  // simulate
  for (int i=0; i<N; i++)
  {
    y_data(i) = gmp->getYd(x(i));
    dy_data(i) = gmp->getYdDot(x(i), x_dot);
    ddy_data(i) = gmp->getYdDDot(x(i), x_dot, x_ddot);
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

  #ifdef TEST_GMP_UPDATE_DEBUG_
  }catch(std::exception &e) { throw std::runtime_error(std::string("[main]: ") + e.what()); }
  #endif
}

void updateGMP(std::shared_ptr<gmp_::GMP> gmp, const Point &point)
{
  #ifdef TEST_GMP_UPDATE_DEBUG_
  try{
  #endif

  std::vector<gmp_::UPDATE_TYPE> type;
  arma::rowvec z;
  std::vector<gmp_::Phase> s;

  gmp_::Phase si(point.x, point.x_dot, point.x_ddot);

  if (point.update_pos)
  {
    type.push_back(gmp_::UPDATE_TYPE::POS);
    z = arma::join_horiz( z, arma::mat({point.p}) );
    s.push_back(si);
  }

  if (point.update_vel)
  {
    type.push_back(gmp_::UPDATE_TYPE::VEL);
    z = arma::join_horiz( z, arma::mat({point.p_dot}) );
    s.push_back(si);
  }

  if (point.update_accel)
  {
    type.push_back(gmp_::UPDATE_TYPE::ACCEL);
    z = arma::join_horiz( z, arma::mat({point.p_ddot}) );
    s.push_back(si);
  }

  gmp->updateWeights(s, z, type);

  #ifdef TEST_GMP_UPDATE_DEBUG_
  }catch(std::exception &e) { throw std::runtime_error(std::string("[updateGMP]: ") + e.what()); }
  #endif

}

void loadParams()
{
  #ifdef TEST_GMP_UPDATE_DEBUG_
  try{
  #endif

  ros::NodeHandle nh_("~");

  // ===========  Read params  ===============
  std::string path = ros::package::getPath("gmp_test") + "/matlab/data/";

  if (!nh_.getParam("train_data_file", train_data_file)) train_data_file = "gmp_train_data.bin";
  if (!nh_.getParam("sim_data_file", sim_data_file)) sim_data_file = "gmp_update_sim_data.bin";
  if (!nh_.getParam("train_method", train_method)) train_method = "LWR";
  if (!nh_.getParam("N_kernels", N_kernels)) N_kernels = 30;
  if (!nh_.getParam("D", D)) D = 50;
  if (!nh_.getParam("K", K)) K = 250;
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

  #ifdef TEST_GMP_UPDATE_DEBUG_
  }catch(std::exception &e) { throw std::runtime_error(std::string("[loadParams]: ") + e.what()); }
  #endif
}

