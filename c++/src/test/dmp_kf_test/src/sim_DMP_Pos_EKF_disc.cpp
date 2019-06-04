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

#include <dmp_kf_test/utils.h>

using namespace as64_;

struct PosMsrCookie
{
  std::shared_ptr<dmp_::DMP_pos> dmp_p;
  double t;
  arma::vec Y;
  arma::vec dY;
  arma::vec Y0;
  arma::vec Yg;
  double tau;

  PosMsrCookie(std::shared_ptr<dmp_::DMP_pos> dmp_p, double t,
               const arma::vec &Y, const arma::vec &dY, const arma::vec &Y0,
               const arma::vec &Yg, double tau)
  {
    this->dmp_p = dmp_p;
    this->t = t;
    this->Y = Y;
    this->dY = dY;
    this->Y0 = Y0;
    this->Yg = Yg;
    this->tau = tau;
  }
};

arma::vec pStateTransFun(const arma::vec &theta, void *cookie)
{
  return theta;
}

arma::vec pMsrFun(const arma::vec &theta, void *cookie)
{
  PosMsrCookie *ck = static_cast<PosMsrCookie *>(cookie);

  arma::vec Yg_hat = theta.subvec(0,2);
  double tau_hat = theta(3);

  double x_hat = ck->t/tau_hat;

  ck->dmp_p->setTau(tau_hat);
  arma::vec Y_out = ck->dmp_p->calcYddot(x_hat, ck->Y, ck->dY, Yg_hat);
  ck->dmp_p->setTau(ck->tau);

  return Y_out;
}


int main(int argc, char** argv)
{
  // ===========  Initialize the ROS node  ===============
  ros::init(argc, argv, "dmp_ekf_pos_test_node");
  ros::NodeHandle nh_("~");

  std::string path = ros::package::getPath("dmp_kf_test");

  double dt = 0.005;

  arma::vec Y0_offset = {0.0, 0.0, 0.0};

  arma::vec Yg_offset = {0.75, -0.09, 0.64};
  double time_offset = 6;

  arma::vec Yg_up_lim = 0.8*arma::vec({1.0, 1.0, 1.0,});
  arma::vec Yg_low_lim = -Yg_up_lim;
  double tau_low_lim = 1.0;
  double tau_up_lim = 30.0; // Inf;

  arma::mat process_noise = arma::diagmat(arma::vec({0.02, 0.02, 0.02, 0.05})); // Q
  double msr_noise = std::pow(0.2,2);
  double msr_noise_hat = 1000;


  arma::mat init_params_variance = arma::diagmat( 1e5*arma::vec({1, 1, 1, 1}) ); // P
  double a_pc = 0.99; // forgetting factor in fading memory KF
  double a_p = 1.002;

  arma::vec theta_low_lim = arma::join_vert(Yg_low_lim, arma::vec({tau_low_lim}) );
  arma::vec theta_up_lim = arma::join_vert(Yg_up_lim, arma::vec({tau_up_lim}) );

  int N_params = theta_low_lim.size();
  arma::mat A_c = arma::join_vert(-arma::mat().eye(N_params, N_params), arma::mat().eye(N_params, N_params));
  arma::vec b_c = arma::join_vert(-theta_low_lim, theta_up_lim);
  bool enable_constraints = true;

  arma::vec num_diff_step = {0.001, 0.001, 0.001, 0.01};

  arma::mat M_r = 5*arma::mat().eye(3,3);

  // ===========  load DMP data  ===============
  std::string dmp_data_file = path + "/matlab/data/dmp_data.bin";
  std::shared_ptr<dmp_::DMP_pos> dmp_p;
  std::shared_ptr<dmp_::DMP_eo> dmp_o;
  arma::vec P0d;
  arma::vec Yg0;
  arma::vec Q0d;
  arma::vec Qgd;
  double tau0;
  loadDMPdata(dmp_data_file, &dmp_p, &dmp_o, &Yg0, &P0d, &Qgd, &Q0d, &tau0);

  int Dim = Yg0.size();
  std::shared_ptr<dmp_::CanonicalClock> can_clock_ptr = dmp_p->can_clock_ptr;
  can_clock_ptr->setTau(tau0);

  // DMP simulation
  // set initial values
  double t = 0.0;
  double x = 0.0;
  double dx = 0.0;
  arma::vec Y0 = P0d + Y0_offset;
  arma::vec Y = Y0;
  arma::vec dY = arma::vec().zeros(Dim);
  arma::vec ddY = arma::vec().zeros(Dim);
  arma::vec F_ext = arma::vec().zeros(Dim);

  arma::rowvec Time;
  arma::mat Y_data;
  arma::mat dY_data;
  arma::mat ddY_data;
  arma::rowvec x_data;
  arma::mat Yg_data;
  arma::rowvec tau_data;
  arma::mat F_data;
  arma::mat Sigma_theta_data;

  arma::vec Yg = Yg0 + Yg_offset;

  double t_end = tau0 + time_offset;
  double tau = t_end;
  can_clock_ptr->setTau(tau);

  double tau_hat = tau0;
  arma::vec Yg_hat = Yg0;
  double x_hat = t/tau_hat;

  int N_out = Yg_hat.size();
  arma::vec Y_out_hat = arma::vec().zeros(N_out);
  arma::vec Y_out = arma::vec().zeros(N_out);

  arma::vec theta = arma::join_vert(Yg_hat, arma::vec({tau_hat}));

  arma::mat P_theta = arma::mat().eye(N_params, N_params) % init_params_variance;
  arma::mat Rn_hat = arma::mat().eye(N_out,N_out) * msr_noise_hat;
  arma::mat Qn = arma::mat().eye(N_params,N_params) % process_noise;

  arma::arma_rng::set_seed(0);
  arma::mat Rn = arma::mat().eye(N_out,N_out) * msr_noise;
  arma::mat Sigma_vn = arma::sqrtmat_sympd(Rn);

  // Set up EKF object
  kf_::EKF ekf(theta, P_theta, N_out, &pStateTransFun, &pMsrFun);
  ekf.setProcessNoiseCov(Qn);
  ekf.setMeasureNoiseCov(Rn_hat);
  // a_p = exp(a_pc*dt);
  ekf.setFadingMemoryCoeff(a_p);

  ekf.enableParamsContraints(enable_constraints);
  ekf.setParamsConstraints(A_c, b_c);
  ekf.setPartDerivStep(num_diff_step);

  // ekf.setStateTransFunJacob(@pStateTransFunJacob);
  // ekf.setMsrFunJacob(@pMsrFunJacob);

  dmp_p->setY0(Y0);

  std::cout << "DMP-EKF (discrete) Pos simulation...\n";
  arma::wall_clock timer;
  timer.tic();
  while (true)
  {
    // data logging
    Time = arma::join_horiz(Time, arma::vec({t}));
    Y_data = arma::join_horiz(Y_data, Y);
    dY_data = arma::join_horiz(dY_data, dY);
    ddY_data = arma::join_horiz(ddY_data, ddY);

    Yg_data = arma::join_horiz(Yg_data, Yg_hat);
    tau_data = arma::join_horiz(tau_data, arma::vec({tau_hat}));

    x_data = arma::join_horiz(x_data, arma::vec({x}));
    F_data = arma::join_horiz(F_data, F_ext);
    Sigma_theta_data = arma::join_horiz(Sigma_theta_data, arma::sqrt(arma::diagvec(P_theta)));

    // DMP simulation
    dmp_p->setTau(tau_hat);
    arma::vec ddY_hat = dmp_p->calcYddot(x_hat, Y, dY, Yg_hat);
    dmp_p->setTau(tau);

    ddY = dmp_p->calcYddot(x, Y, dY, Yg) + Sigma_vn*arma::vec().randn(N_out);


    F_ext = M_r * (ddY - ddY_hat);

    Y_out = ddY;
    // Y_out_hat = ddY_hat;

    // Update phase variable
    dx = can_clock_ptr->getPhaseDot(x);

    // Stopping criteria
    double err_p = arma::norm(Yg-Y)/arma::norm(Yg);
    if (err_p <= 0.5e-2 & t>=t_end) break;

    if (t>=t_end)
    {
      std::cerr << "Time limit reached. Stopping simulation...\n";
      break;
    }

    // ========  KF measurement update  ========
    PosMsrCookie msr_cookie(dmp_p, t, Y, dY, Y0, Yg, tau);
    ekf.correct(Y_out, static_cast<void *>(&msr_cookie));

    // ========  KF time update  ========
    ekf.predict();

    theta = ekf.theta;
    P_theta = ekf.P;

    // Numerical integration
    t = t + dt;
    x = x + dx*dt;
    Y = Y + dY*dt;
    dY = dY + ddY*dt;

    Yg_hat = theta.subvec(0,2);
    tau_hat = theta(3);
    x_hat = t/tau_hat;

  }
  std::cout << "Elapsed time: " << timer.toc() << " sec\n";

  // ===========  write results  ===============
  std::string sim_data_file = path + "/matlab/data/pos_est_results.bin";
  std::ofstream out(sim_data_file, std::ios::out | std::ios::binary);
  if (!out) throw std::runtime_error("Failed to create file \"" + sim_data_file + "\"...");
  io_::write_mat(Time, out);
  io_::write_mat(Yg, out);
  io_::write_mat(Yg_data, out);
  io_::write_scalar(tau, out);
  io_::write_mat(tau_data, out);
  io_::write_mat(Sigma_theta_data, out);
  io_::write_mat(F_data, out);
  io_::write_mat(Y_data, out);
  io_::write_mat(dY_data, out);
  out.close();

  // ===========  Shutdown ROS node  ==================
  ros::shutdown();

  return 0;
}
