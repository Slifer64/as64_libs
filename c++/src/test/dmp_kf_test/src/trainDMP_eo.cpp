#include <ros/ros.h>
#include <ros/package.h>
#include <memory>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <exception>

#include <io_lib/io_utils.h>
#include <dmp_lib/DMP/DMP_eo.h>
#include <dmp_lib/GatingFunction/LinGatingFunction.h>
#include <dmp_lib/GatingFunction/ExpGatingFunction.h>
#include <dmp_lib/GatingFunction/SigmoidGatingFunction.h>
#include <math_lib/quaternions.h>
#include <dmp_kf_test/utils.h>


using namespace as64_;

int main(int argc, char** argv)
{
  // ===========  Initialize the ROS node  ===============
  ros::init(argc, argv, "train_dmp_eo_node");
  ros::NodeHandle nh_("~");

  // ===========  Load training data  ===============
  std::string path = ros::package::getPath("dmp_kf_test");
  std::string train_data_file = path + "/matlab/data/train/train_data.bin";
  std::ifstream in(train_data_file, std::ios::in | std::ios::binary);
  if (!in) throw std::runtime_error("Failed to open file \"" + train_data_file + "\"...");
  arma::rowvec Timed;
  arma::mat temp; // to skip pos,vel,accel for position when reading data
  arma::mat Qd_data;
  arma::mat vRotd_data;
  arma::mat dvRotd_data;
  io_::read_mat(Timed, in, true);
  io_::read_mat(temp, in, true); // skip pos
  io_::read_mat(temp, in, true); // skip vel
  io_::read_mat(temp, in, true); // skip accel
  io_::read_mat(Qd_data, in, true);
  io_::read_mat(vRotd_data, in, true);
  io_::read_mat(dvRotd_data, in, true);
  in.close();

  double Ts = Timed(1) - Timed(0);

  int n_data = Timed.size();
  int i_end = n_data-1;

  // ===========  initialize DMP  ===============
  arma::vec a_z = 20 * arma::vec().ones(3);
  arma::vec b_z = a_z/4;
  dmp_::TrainMethod train_method = dmp_::TrainMethod::LWR;
  std::shared_ptr<dmp_::CanonicalClock> can_clock_ptr(new dmp_::CanonicalClock());
  std::shared_ptr<dmp_::GatingFunction> shape_attr_gat_ptr( new dmp_::SigmoidGatingFunction(1.0, 0.5));
  // shape_attr_gat_ptr = LinGatingFunction(1.0, 0.01);
  arma::uvec N_kernels = {40, 40, 40};
  std::shared_ptr<dmp_::DMP_eo> dmp_eo(new dmp_::DMP_eo(dmp_::TYPE::STD, N_kernels, a_z, b_z, can_clock_ptr, shape_attr_gat_ptr));

  // ===========  Train DMP  ===============
  std::cout << "DMP_eo training...\n";
  arma::wall_clock timer;
  timer.tic();
  arma::vec offline_train_mse;
  dmp_eo->train(train_method, Timed, Qd_data, vRotd_data, dvRotd_data, &offline_train_mse);
  std::cout << "offline_train_mse =\n" << offline_train_mse << "\n";
  std::cout << "Elapsed time: " << timer.toc() << " sec\n";

  // ===========  save dmp data  ===============
  arma::vec Qgd = Qd_data.col(i_end);
  arma::vec Q0d = Qd_data.col(0);
  double taud = Timed(i_end);
  std::string dmp_data_file = path + "/matlab/data/model/dmp_eo_data.bin";
  std::ofstream out(dmp_data_file, std::ios::out | std::ios::binary);
  if (!out) throw std::runtime_error("Failed to create file \"" + dmp_data_file + "\"...");
  dmp_eo->exportToFile(out);
  io_::write_mat(Qgd, out);
  io_::write_mat(Q0d, out);
  io_::write_scalar(taud, out);
  out.close();

  // ===========  Shutdown ROS node  ==================
  ros::shutdown();

  return 0;
}
