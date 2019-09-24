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
#include <kf_lib/EKF.h>
#include <math_lib/quaternions.h>

#include <dmp_kf_test/utils.h>
#include <dmp_lib/KalmanFilter/DMPpEKFa.h>

#include <plot_lib/qt_plot.h>

using namespace as64_;


int main(int argc, char** argv)
{
  // ===========  Initialize the ROS node  ===============
  ros::init(argc, argv, "DMPpEKFa_test_node");
  ros::NodeHandle nh_("~");

  std::string path = ros::package::getPath("dmp_kf_test");

  arma::arma_rng::set_seed(0);

  double dt = 0.005;

  arma::vec Y0_offset = arma::vec({0,0,0});
  arma::vec pg_offset = arma::vec({0.8, -0.9, 0.7});
  double time_offset = 5;

  arma::mat Qn = 0.001*arma::mat().eye(10,10);
  arma::mat Rn = std::pow(0.1,2)*arma::mat().eye(6,6);
  arma::mat Rn_hat = arma::diagmat(arma::vec( {100, 100, 100, 100, 100, 100} ));

  arma::mat P0 = arma::diagmat(arma::vec( {1,1,1,  1,1,1,  100,100,100,  1000} ));
  double a_p = 1.002;

  arma::mat Mr = 5*arma::mat().eye(3,3);
  arma::mat Dr = 80*arma::mat().eye(3,3);
  arma::mat Kr = 400*arma::mat().eye(3,3);

  // ===========  load DMP data  ===============
  std::string dmp_data_file = path + "/matlab/data/dmp_data.bin";
  std::shared_ptr<dmp_::DMP_pos> dmp_p;
  std::shared_ptr<dmp_::DMP_eo> dmp_o;
  arma::vec P0d;
  arma::vec Yg0;
  arma::vec Q0d;
  arma::vec Qg0;
  double tau0;
  loadDMPdata(dmp_data_file, &dmp_p, &dmp_o, &Yg0, &P0d, &Qg0, &Q0d, &tau0);

  std::shared_ptr<dmp_::CanonicalClock> can_clock_ptr = dmp_p->can_clock_ptr;
  can_clock_ptr->setTau(tau0);

  // DMP simulation
  // set initial values
  int Dim = 3;
  double t = 0.0;
  double x = 0.0;
  double dx = 0.0;
  arma::vec p0 = P0d + Y0_offset;
  arma::vec p = p0;
  arma::vec p_dot = arma::vec().zeros(Dim);
  arma::vec p_ddot = arma::vec().zeros(Dim);
  arma::vec F_ext = arma::vec().zeros(Dim);

  arma::rowvec Time;
  arma::mat p_data, p_dot_data, p_ddot_data;
  arma::mat p_hat_data, p_dot_hat_data, p_ddot_hat_data;
  arma::rowvec x_data;
  arma::mat pg_data;
  arma::rowvec tau_data;
  arma::mat F_data;
  arma::mat Sigma_theta_data;

  arma::vec pg = Yg0 + pg_offset;

  double t_end = tau0 + time_offset;
  double tau = t_end;
  can_clock_ptr->setTau(tau);

  double tau_hat = tau0;
  arma::vec pg_hat = Yg0;
  double x_hat = t/tau_hat;
  arma::vec p_hat = p;
  arma::vec p_dot_hat = p_dot;
  arma::vec p_ddot_hat = arma::vec().zeros(3,1);

  arma::vec theta(10);
  theta.subvec(0,2) = p_dot_hat;
  theta.subvec(3,5) = p_hat;
  theta.subvec(6,8) = pg_hat;
  theta(9) = tau_hat;
  arma::mat P_theta = P0;

  arma::mat Sigma_vn = arma::sqrt(Rn);
  int N_out = 6;

  // Set up EKF object
  dmp_::DMPpEKFa ekf(dmp_p, dt);
  ekf.setProcessNoiseCov(Qn);
  ekf.setMeasureNoiseCov(Rn_hat);
  ekf.setFadingMemoryCoeff(a_p);
  ekf.theta = theta;
  ekf.P = P_theta;

  dmp_p->setY0(p0);

  arma::wall_clock ekf_timer;
  arma::rowvec ekf_time;

  std::cout << "DMP-EKF (discrete) Pos simulation...\n";
  arma::wall_clock timer;
  timer.tic();
  while (true)
  {
    // data logging
    Time = arma::join_horiz(Time, arma::vec({t}));
    p_data = arma::join_horiz(p_data, p);
    p_dot_data = arma::join_horiz(p_dot_data, p_dot);
    p_ddot_data = arma::join_horiz(p_ddot_data, p_ddot);

    p_hat_data = arma::join_horiz(p_hat_data, p_hat);
    p_dot_hat_data = arma::join_horiz(p_dot_hat_data, p_dot_hat);
    p_ddot_hat_data = arma::join_horiz(p_ddot_hat_data, p_ddot_hat);

    pg_data = arma::join_horiz(pg_data, pg_hat);
    tau_data = arma::join_horiz(tau_data, arma::vec({tau_hat}));

    x_data = arma::join_horiz(x_data, arma::vec({x}));
    F_data = arma::join_horiz(F_data, F_ext);
    Sigma_theta_data = arma::join_horiz(Sigma_theta_data, arma::sqrt(arma:: diagvec(P_theta)));

    // DMP simulation
    dmp_p->setTau(tau_hat);
    arma::vec p_ddot_hat = dmp_p->calcYddot(x_hat, p_hat, p_dot_hat, pg_hat);
    dmp_p->setTau(tau);
    p_ddot = dmp_p->calcYddot(x, p, p_dot, pg);

    F_ext = Mr*(p_ddot - p_ddot_hat) + Dr*(p_dot - p_dot_hat) + Kr*(p-p_hat);

    arma::vec Y_out = arma::join_vert(p_dot, p); // + Sigma_vn*arma::vec().randn(N_out);
    // Y_out_hat = p_ddot_hat;

    // Update phase variable
    dx = can_clock_ptr->getPhaseDot(x);

    // Stopping criteria
    double err_p = arma::norm(pg-p)/norm(pg);
    if (err_p <= 0.5e-2 & t>=t_end) break;

    if (t>=t_end)
    {
      std::cerr << "Time limit reached. Stopping simulation...\n";
      break;
    }

    ekf_timer.tic();
    // ========  KF measurement update  ========
    ekf.correct(Y_out);

    // ========  KF time update  ========
    dmp_::DMPpEKFa::StateTransCookie state_cookie(t,p0);
    ekf.predict(static_cast<void *>(&state_cookie));

    ekf_time = arma::join_horiz( ekf_time, arma::vec({ekf_timer.toc()}) );

    theta = ekf.theta;
    P_theta = ekf.P;

    // Numerical integration
    t = t + dt;
    x = x + dx*dt;
    p = p + p_dot*dt;
    p_dot = p_dot + p_ddot*dt;

    p_dot_hat = theta.subvec(0,2);
    p_hat = theta.subvec(3,5);
    pg_hat = theta.subvec(6,8);
    tau_hat = theta(9);

    x_hat = t/tau_hat;
  }
  std::cout << "Elapsed time: " << timer.toc() << " sec\n";

  double ekf_mean_time = arma::mean(ekf_time) * 1000;
  double ekf_std_time = arma::stddev(ekf_time) * 1000;
  double ekf_max_time = arma::max(ekf_time) * 1000;

  std::cerr << "ekf_mean_time = " << ekf_mean_time << " ms\n";
  std::cerr << "ekf_std_time = " << ekf_std_time << " ms\n";
  std::cerr << "ekf_max_time = " << ekf_max_time << " ms\n";

  // ===========  write results  ===============
  std::string sim_data_file = path + "/matlab/data/sim_DMPpEKFa_results.bin";
  std::ofstream out(sim_data_file, std::ios::out | std::ios::binary);
  if (!out) throw std::runtime_error("Failed to create file \"" + sim_data_file + "\"...");
  io_::write_mat(Time, out);
  io_::write_mat(pg, out);
  io_::write_mat(pg_data, out);
  io_::write_scalar(tau, out);
  io_::write_mat(tau_data, out);
  io_::write_mat(Sigma_theta_data, out);
  io_::write_mat(F_data, out);
  io_::write_mat(p_data, out);
  io_::write_mat(p_dot_data, out);
  io_::write_mat(p_hat_data, out);
  io_::write_mat(p_dot_hat_data, out);
  out.close();

  // ===========  Shutdown ROS node  ==================
  ros::shutdown();

  return 0;
}
