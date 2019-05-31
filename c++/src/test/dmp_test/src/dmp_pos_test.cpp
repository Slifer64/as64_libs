#include <ros/ros.h>
#include <ros/package.h>
#include <memory>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <exception>

#include <io_lib/io_utils.h>
#include <dmp_lib/DMP/DMP_pos.h>
#include <dmp_lib/GatingFunction/LinGatingFunction.h>
#include <dmp_lib/GatingFunction/ExpGatingFunction.h>
#include <dmp_lib/GatingFunction/SigmoidGatingFunction.h>

#include <dmp_test/utils.h>

using namespace as64_;

int main(int argc, char** argv)
{
  // ===========  Initialize the ROS node  ===============
  ros::init(argc, argv, "dmp_pos_test_node");
  ros::NodeHandle nh_("~");

  // ===========  Read params  ===============
  std::string path = ros::package::getPath("dmp_test") + "/matlab/data/";


  std::string gating_type;
  std::string train_data_file;
  std::string sim_data_file;
  arma::vec a_z;
  arma::vec b_z;
  arma::uvec N_kernels;
  dmp_::TrainMethod train_method;
  double ks;
  double kt;

  std::string dmp_type;
  dmp_::TYPE dmp_t;
  if (!nh_.getParam("dmp_type", dmp_type)) dmp_type = "STD";
  if (dmp_type.compare("STD")==0) dmp_t = dmp_::TYPE::STD;
  else if (dmp_type.compare("BIO")==0) dmp_t = dmp_::TYPE::BIO;
  else throw std::runtime_error("Unsupported DMP type: \"" + dmp_type + "\"...");

  if (!nh_.getParam("gating_type", gating_type)) gating_type = "sigmoid";
  if (!nh_.getParam("train_data_file", train_data_file)) train_data_file = "dmp_std_train_data.bin";
  if (!nh_.getParam("sim_data_file", sim_data_file)) sim_data_file = "dmp_std_sim_data.bin";

  std::vector<double> a_z_vec;
  if (!nh_.getParam("a_z", a_z_vec)) a_z = arma::vec({20, 20, 20});
  else a_z = a_z_vec;

  std::vector<double> b_z_vec;
  if (!nh_.getParam("b_z", b_z_vec)) b_z = a_z/4;
  else b_z = b_z_vec;

  std::vector<int> N_kernels_vec;
  if (!nh_.getParam("N_kernels", N_kernels_vec)) N_kernels = {30, 30, 30};
  else
  {
    N_kernels.resize(N_kernels_vec.size());
    for (int i=0;i <N_kernels.size(); i++) N_kernels(i) = N_kernels_vec[i];
  }

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
  arma::mat Pd_data;
  arma::mat dPd_data;
  arma::mat ddPd_data;

  io_::read_mat(Timed, in, true);
  io_::read_mat(Pd_data, in, true);
  io_::read_mat(dPd_data, in, true);
  io_::read_mat(ddPd_data, in, true);
  in.close();

  double Ts = Timed(1)-Timed(0);

  // ===========  initialize DMP  ===============
  std::shared_ptr<dmp_::CanonicalClock> can_clock_ptr( new dmp_::CanonicalClock());
  std::shared_ptr<dmp_::GatingFunction> shape_attr_gat_ptr;
  if (gating_type.compare("sigmoid")==0) shape_attr_gat_ptr.reset(new dmp_::SigmoidGatingFunction(1.0, 0.5));
  else if (gating_type.compare("lin")==0) shape_attr_gat_ptr.reset(new dmp_::LinGatingFunction(1.0, 0.02));
  else if (gating_type.compare("exp")==0) shape_attr_gat_ptr.reset(new dmp_::ExpGatingFunction(1.0, 0.02));
  else throw std::runtime_error("Unsupported gating type: \"" + gating_type + "\"...");

  std::shared_ptr<dmp_::DMP_pos> dmp_p;
  dmp_p.reset(new dmp_::DMP_pos(dmp_t, N_kernels, a_z, b_z, can_clock_ptr, shape_attr_gat_ptr));

  arma::wall_clock timer;
  timer.tic();
  std::cout << "DMP training...\n";
  arma::vec offline_train_mse;

  dmp_p->train(train_method, Timed, Pd_data, dPd_data, ddPd_data, &offline_train_mse);
  std::cout << "offline_train_mse = \n" << offline_train_mse << "\n";
  std::cout << "Elapsed time: " << timer.toc() << " sec\n";

  // ===========  DMP simulation  ===============
  std::cout << "DMP simulation...\n";
  timer.tic();

  int i_end = Timed.size() - 1;
  arma::vec P0 = Pd_data.col(0);
  arma::vec Pgd = Pd_data.col(i_end);
  arma::vec Pg = P0 + ks*(Pgd - P0);
  double T = kt*Timed(i_end);
  double dt = Ts;

  arma::rowvec Time;
  arma::mat P_data;
  arma::mat dP_data;
  arma::mat ddP_data;
  simulateDMPpos(dmp_p, P0, Pg, T, dt, Time, P_data, dP_data, ddP_data);

  std::cout << "Elapsed time: " << timer.toc() << " sec\n";

  // ===========  write results  ===============
  std::ofstream out(sim_data_file, std::ios::out | std::ios::binary);
  if (!out) throw std::runtime_error("Failed to create file \"" + sim_data_file + "\"...");

  io_::write_mat(Timed, out, true);
  io_::write_mat(Pd_data, out, true);
  io_::write_mat(dPd_data, out, true);
  io_::write_mat(ddPd_data, out, true);
  io_::write_mat(Time, out, true);
  io_::write_mat(P_data, out, true);
  io_::write_mat(dP_data, out, true);
  io_::write_mat(ddP_data, out, true);
  out.close();

  // ===========  Shutdown ROS node  ==================
  ros::shutdown();

  return 0;
}
