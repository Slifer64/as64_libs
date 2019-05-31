#include <ros/ros.h>
#include <ros/package.h>
#include <memory>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <exception>

#include <io_lib/io_utils.h>
#include <dmp_lib/DMP/DMP.h>
#include <dmp_lib/DMP/DMP_bio.h>
#include <dmp_lib/GatingFunction/LinGatingFunction.h>
#include <dmp_lib/GatingFunction/ExpGatingFunction.h>
#include <dmp_lib/GatingFunction/SigmoidGatingFunction.h>

#include <dmp_test/utils.h>

using namespace as64_;

int main(int argc, char** argv)
{
  // ===========  Initialize the ROS node  ===============
  ros::init(argc, argv, "dmp_test_node");
  ros::NodeHandle nh_("~");

  // ===========  Read params  ===============
  std::string path = ros::package::getPath("dmp_test") + "/matlab/data/";

  std::string dmp_type;
  std::string gating_type;
  std::string train_data_file;
  std::string sim_data_file;
  double a_z;
  double b_z;
  int N_kernels;
  dmp_::TrainMethod train_method;
  double ks;
  double kt;

  if (!nh_.getParam("dmp_type", dmp_type)) dmp_type = "STD";
  if (!nh_.getParam("gating_type", gating_type)) gating_type = "sigmoid";
  if (!nh_.getParam("train_data_file", train_data_file)) train_data_file = "dmp_std_train_data.bin";
  if (!nh_.getParam("sim_data_file", sim_data_file)) sim_data_file = "dmp_std_sim_data.bin";
  if (!nh_.getParam("a_z", a_z)) a_z = 20;
  if (!nh_.getParam("b_z", b_z)) b_z = a_z/4;
  if (!nh_.getParam("N_kernels", N_kernels)) N_kernels = 60;
  if (!nh_.getParam("ks", ks)) ks = 1.0;
  if (!nh_.getParam("kt", kt)) kt = 1.0;

  std::string train_method_name;
  if (!nh_.getParam("train_method", train_method_name)) train_method_name = "LWR";
  if (train_method_name.compare("LWR")==0) train_method = dmp_::TrainMethod::LWR;
  else if (train_method_name.compare("LS")==0) train_method = dmp_::TrainMethod::LS;
  else throw std::runtime_error("Unsupported train method: \"" + train_method_name + "\"...");

  train_data_file = path + train_data_file;
  sim_data_file = path + sim_data_file;

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

  // ===========  initialize DMP  ===============
  std::shared_ptr<dmp_::CanonicalClock> can_clock_ptr( new dmp_::CanonicalClock());
  std::shared_ptr<dmp_::GatingFunction> shape_attr_gat_ptr;
  if (gating_type.compare("sigmoid")==0) shape_attr_gat_ptr.reset(new dmp_::SigmoidGatingFunction(1.0, 0.5));
  else if (gating_type.compare("lin")==0) shape_attr_gat_ptr.reset(new dmp_::LinGatingFunction(1.0, 0.02));
  else if (gating_type.compare("exp")==0) shape_attr_gat_ptr.reset(new dmp_::ExpGatingFunction(1.0, 0.02));
  else throw std::runtime_error("Unsupported gating type: \"" + gating_type + "\"...");

  std::shared_ptr<dmp_::DMP_> dmp;
  if (dmp_type.compare("STD")==0) dmp.reset(new dmp_::DMP(N_kernels, a_z, b_z, can_clock_ptr, shape_attr_gat_ptr));
  else if (dmp_type.compare("BIO")==0) dmp.reset(new dmp_::DMP_bio(N_kernels, a_z, b_z, can_clock_ptr, shape_attr_gat_ptr));
  else throw std::runtime_error("Unsupported DMP type: \"" + dmp_type + "\"...");

  arma::wall_clock timer;
  timer.tic();
  std::cout << "DMP training...\n";
  double offline_train_mse;
  dmp->train(train_method, Timed, yd_data, dyd_data, ddyd_data, &offline_train_mse);
  std::cout << "offline_train_mse = " << offline_train_mse << "\n";
  std::cout << "Elapsed time: " << timer.toc() << " sec\n";

  // ===========  DMP simulation  ===============
  std::cout << "DMP simulation...\n";
  timer.tic();

  int i_end = Timed.size() - 1;
  double y0 = yd_data(0);
  double ygd = yd_data(i_end);
  double yg = y0 + ks*(ygd - y0);
  double T = kt*Timed(i_end);
  double dt = Ts;

  arma::rowvec Time;
  arma::rowvec y_data;
  arma::rowvec dy_data;
  arma::rowvec ddy_data;
  simulateDMP(dmp, y0, yg, T, dt, Time, y_data, dy_data, ddy_data);

  std::cout << "Elapsed time: " << timer.toc() << " sec\n";

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
  out.close();

  // ===========  Shutdown ROS node  ==================
  ros::shutdown();

  return 0;
}
