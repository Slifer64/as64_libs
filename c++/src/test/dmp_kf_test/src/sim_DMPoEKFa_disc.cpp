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
#include <kf_lib/EKF.h>
#include <math_lib/quaternions.h>

#include <dmp_kf_test/utils.h>
#include <dmp_lib/KalmanFilter/DMPoEKFa.h>

#include <plot_lib/qt_plot.h>

using namespace as64_;

double dt;

arma::vec Q0_offset;
arma::vec Qg_offset;
double time_offset;

arma::mat Qn;
arma::mat Rn;
arma::mat Rn_hat;

arma::mat P0;
double a_p;

arma::mat Mr;
arma::mat Dr;
arma::mat Kr;

void loadParams()
{
  ros::NodeHandle nh_("~");

  if (!nh_.getParam("dt",dt)) throw std::runtime_error("Failed to load param \"dt\"...");

  std::vector<double> temp;
  if (!nh_.getParam("Q0_offset",temp)) throw std::runtime_error("Failed to load param \"Q0_offset\"...");
  Q0_offset = arma::vec(temp);

  if (!nh_.getParam("Qg_offset",temp)) throw std::runtime_error("Failed to load param \"Qg_offset\"...");
  Qg_offset = arma::vec(temp);

  if (!nh_.getParam("time_offset",time_offset)) throw std::runtime_error("Failed to load param \"time_offset\"...");

  if (!nh_.getParam("Qn",temp)) throw std::runtime_error("Failed to load param \"Qn\"...");
  Qn = arma::diagmat(arma::vec(temp));

  if (!nh_.getParam("Rn",temp)) throw std::runtime_error("Failed to load param \"Rn\"...");
  Rn = arma::diagmat(arma::vec(temp));

  if (!nh_.getParam("Rn_hat",temp)) throw std::runtime_error("Failed to load param \"Rn_hat\"...");
  Rn_hat = arma::diagmat(arma::vec(temp));

  if (!nh_.getParam("P0",temp)) throw std::runtime_error("Failed to load param \"P0\"...");
  P0 = arma::diagmat(arma::vec(temp));

  if (!nh_.getParam("a_p",a_p)) throw std::runtime_error("Failed to load param \"a_p\"...");

  if (!nh_.getParam("Mr",temp)) throw std::runtime_error("Failed to load param \"Mr\"...");
  Mr = arma::diagmat(arma::vec(temp));

  if (!nh_.getParam("Dr",temp)) throw std::runtime_error("Failed to load param \"Dr\"...");
  Dr = arma::diagmat(arma::vec(temp));

  if (!nh_.getParam("Kr",temp)) throw std::runtime_error("Failed to load param \"Kr\"...");
  Kr = arma::diagmat(arma::vec(temp));
}

int main(int argc, char** argv)
{
  // ===========  Initialize the ROS node  ===============
  ros::init(argc, argv, "DMPoEKFa_test_node");
  ros::NodeHandle nh_("~");

  std::string path = ros::package::getPath("dmp_kf_test");

  arma::arma_rng::set_seed(0);

  loadParams();

  // ===========  load DMP data  ===============
  std::string dmp_data_file = path + "/matlab/data/model/dmp_eo_data.bin";
  std::ifstream in(dmp_data_file, std::ios::in | std::ios::binary);
  if (!in) throw std::runtime_error("Failed to open file \"" + dmp_data_file + "\"...");
  std::shared_ptr<dmp_::DMP_eo> dmp_o;
  arma::vec Qg0;
  arma::vec Q0d;
  double tau0;
  dmp_o = dmp_::DMP_eo::importFromFile(in);
  io_::read_mat(Qg0, in);
  io_::read_mat(Q0d, in);
  io_::read_scalar(tau0, in);
  in.close();
  // ===========================================

  std::shared_ptr<dmp_::CanonicalClock> can_clock_ptr = dmp_o->can_clock_ptr;
  can_clock_ptr->setTau(tau0);

  // DMP simulation
  // set initial values
  int Dim = 3;
  double t = 0.0;
  double x = 0.0;
  double dx = 0.0;

  arma::vec Q0 = math_::quatProd(math_::quatExp(Q0_offset), Q0d);
  arma::vec Q = Q0;
  arma::vec vRot = arma::vec().zeros(Dim);
  arma::vec dvRot = arma::vec().zeros(Dim);
  arma::vec F_ext = arma::vec().zeros(Dim);

  arma::rowvec Time;
  arma::mat Q_data, vRot_data, dvRot_data;
  arma::mat Q_hat_data, vRot_hat_data, dvRot_hat_data;
  arma::rowvec x_data;
  arma::mat Qg_data;
  arma::rowvec tau_data;
  arma::mat F_data;
  arma::mat Sigma_theta_data;

  arma::vec Qg = math_::quatProd(math_::quatExp(Qg_offset), Qg0);

  double t_end = tau0 + time_offset;
  double tau = t_end;
  can_clock_ptr->setTau(tau);


  double tau_hat = tau0;
  arma::vec Qg_hat = Qg0;
  arma::vec eQg_hat = math_::quatLog(Qg_hat);
  double x_hat = t/tau_hat;
  arma::vec Q_hat = Q;
  arma::vec eQ_hat = math_::quatLog(Q_hat);
  arma::vec vRot_hat = vRot;
  arma::vec dvRot_hat = arma::vec().zeros(3,1);

  arma::vec theta(10);
  theta.subvec(0,2) = vRot_hat;
  theta.subvec(3,5) = eQ_hat;
  theta.subvec(6,8) = eQg_hat;
  theta(9) = tau_hat;
  arma::mat P_theta = P0;

  arma::mat Sigma_vn = arma::sqrt(Rn);
  int N_out = 6;

  // Set up EKF object
  dmp_::DMPoEKFa ekf(dmp_o, dt);
  ekf.setProcessNoiseCov(Qn);
  ekf.setMeasureNoiseCov(Rn_hat);
  ekf.setFadingMemoryCoeff(a_p);
  ekf.theta = theta;
  ekf.P = P_theta;

  dmp_o->setQ0(Q0);

  arma::wall_clock ekf_timer;
  arma::rowvec ekf_time;

  std::cout << "DMP-EKF (discrete) Orient simulation...\n";
  arma::wall_clock timer;
  timer.tic();
  while (true)
  {
    // data logging
    Time = arma::join_horiz(Time, arma::vec({t}));
    Q_data = arma::join_horiz(Q_data, Q);
    vRot_data = arma::join_horiz(vRot_data, vRot);
    dvRot_data = arma::join_horiz(dvRot_data, dvRot);

    Q_hat_data = arma::join_horiz(Q_hat_data, Q_hat);
    vRot_hat_data = arma::join_horiz(vRot_hat_data, vRot_hat);
    dvRot_hat_data = arma::join_horiz(dvRot_hat_data, dvRot_hat);

    Qg_data = arma::join_horiz(Qg_data, Qg_hat);
    tau_data = arma::join_horiz(tau_data, arma::vec({tau_hat}));

    x_data = arma::join_horiz(x_data, arma::vec({x}));
    F_data = arma::join_horiz(F_data, F_ext);
    Sigma_theta_data = arma::join_horiz(Sigma_theta_data, arma::sqrt(arma:: diagvec(P_theta)));

    // DMP simulation
    dmp_o->setTau(tau_hat);
    arma::vec dvRot_hat = dmp_o->calcRotAccel(x_hat, Q_hat, vRot_hat, Qg_hat);
    dmp_o->setTau(tau);
    dvRot = dmp_o->calcRotAccel(x, Q, vRot, Qg);

    F_ext = Mr*(dvRot - dvRot_hat) + Dr*(vRot - vRot_hat) + Kr*math_::quatLog(math_::quatDiff(Q,Q_hat));

    arma::vec Y_out = arma::join_vert(vRot, math_::quatLog(Q)); // + Sigma_vn*arma::vec().randn(N_out);
    // Y_out_hat = dvRot_hat;

    // Update phase variable
    dx = can_clock_ptr->getPhaseDot(x);

    // Stopping criteria
    double err_o = arma::norm(math_::quatLog(math_::quatDiff(Qg,Q)));
    if (err_o <= 0.5e-2 & t>=t_end) break;

    if (t>=t_end)
    {
      std::cerr << "Time limit reached. Stopping simulation...\n";
      break;
    }

    ekf_timer.tic();
    // ========  KF measurement update  ========
    ekf.correct(Y_out);

    // ========  KF time update  ========
    dmp_::DMPoEKFa::StateTransCookie state_cookie(t);
    ekf.predict(static_cast<void *>(&state_cookie));

    ekf_time = arma::join_horiz( ekf_time, arma::vec({ekf_timer.toc()}) );

    theta = ekf.theta;
    P_theta = ekf.P;

    // Numerical integration
    t = t + dt;
    x = x + dx*dt;
    Q = math_::quatProd( math_::quatExp(vRot*dt), Q);
    vRot = vRot + dvRot*dt;

    vRot_hat = theta.subvec(0,2);
    eQ_hat = theta.subvec(3,5);
    Q_hat = math_::quatExp(eQ_hat);
    eQg_hat = theta.subvec(6,8);
    Qg_hat = math_::quatExp(eQg_hat);
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
  std::string sim_data_file = path + "/matlab/data/sim/sim_DMPoEKFa_results.bin";
  std::ofstream out(sim_data_file, std::ios::out | std::ios::binary);
  if (!out) throw std::runtime_error("Failed to create file \"" + sim_data_file + "\"...");
  io_::write_mat(Time, out);
  io_::write_mat(Qg, out);
  io_::write_mat(Qg_data, out);
  io_::write_scalar(tau, out);
  io_::write_mat(tau_data, out);
  io_::write_mat(Sigma_theta_data, out);
  io_::write_mat(F_data, out);
  io_::write_mat(Q_data, out);
  io_::write_mat(vRot_data, out);
  io_::write_mat(Q_hat_data, out);
  io_::write_mat(vRot_hat_data, out);
  out.close();

  // ===========  Shutdown ROS node  ==================
  ros::shutdown();

  return 0;
}
