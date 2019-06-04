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
#include <dmp_lib/DMP/DMP_eo.h>
#include <math_lib/quaternions.h>
#include <dmp_kf_test/utils.h>


using namespace as64_;

int main(int argc, char** argv)
{
  // ===========  Initialize the ROS node  ===============
  ros::init(argc, argv, "sim_dmp_node");
  ros::NodeHandle nh_("~");

  std::string path = ros::package::getPath("dmp_kf_test");
  arma::wall_clock timer;

  // ===========  load DMP data  ===============
  std::string dmp_data_file = path + "/matlab/data/dmp_data.bin";
  std::shared_ptr<dmp_::DMP_pos> dmp_p;
  std::shared_ptr<dmp_::DMP_eo> dmp_o;
  arma::vec P0d;
  arma::vec Pgd;
  arma::vec Q0d;
  arma::vec Qgd;
  double taud;
  loadDMPdata(dmp_data_file, &dmp_p, &dmp_o, &Pgd, &P0d, &Qgd, &Q0d, &taud);


  // ===========  DMP simulation  ===============
  double ks = 1.0;
  double kt = 1.0;
  double dt = 0.005;
  arma::vec P0 = P0d;
  arma::vec Pg = P0d + ks*(Pgd-P0d);
  arma::vec Q0 = Q0d;
  arma::vec e0 = ks*math_::quatLog( math_::quatProd( Qgd, math_::quatInv(Q0d) ) );
  arma::vec Qg = math_::quatProd(math_::quatExp(e0), Q0);
  double T = kt*taud;

  arma::rowvec Time;
  arma::mat P_data;
  arma::mat dP_data;
  arma::mat ddP_data;
  arma::mat Q_data;
  arma::mat vRot_data;
  arma::mat dvRot_data;

  std::cout << "DMP simulation...\n";
  timer.tic();
  simulatePosOrientDMP(dmp_p, dmp_o, P0, Q0, Pg, Qg, T, dt,
                       Time, P_data, dP_data, ddP_data, Q_data, vRot_data, dvRot_data);

  std::cout << "Elapsed time: " << timer.toc() << " sec\n";


  // ===========  write results  ===============
  std::string sim_data_file = path + "/matlab/data/sim_data.bin";
  std::ofstream out(sim_data_file, std::ios::out | std::ios::binary);
  if (!out) throw std::runtime_error("Failed to create file \"" + sim_data_file + "\"...");
  io_::write_mat(Time, out, true);
  io_::write_mat(P_data, out, true);
  io_::write_mat(dP_data, out, true);
  io_::write_mat(ddP_data, out, true);
  io_::write_mat(Q_data, out, true);
  io_::write_mat(vRot_data, out, true);
  io_::write_mat(dvRot_data, out, true);
  io_::write_mat(Qg, out, true);
  out.close();


  // ===========  Shutdown ROS node  ==================
  ros::shutdown();

  return 0;
}
