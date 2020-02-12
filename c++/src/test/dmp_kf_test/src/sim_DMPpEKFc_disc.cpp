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

arma::mat pStateTransFunJacob(const arma::vec &theta, void *cookie)
{
  int n_params = theta.size();
  arma::mat J = arma::mat().eye(n_params, n_params);
  return J;
}

arma::mat pMsrFunJacob(const arma::vec &theta, void *cookie)
{
  PosMsrCookie *ck = static_cast<PosMsrCookie *>(cookie);

  int n_params = theta.size();
  int i_end = n_params - 1;
  arma::vec Yg_hat = theta.subvec(0,i_end-1);
  double tau_hat = theta(i_end);
  double x_hat = ck->t / tau_hat;

  arma::mat J = ck->dmp_p->getAcellPartDev_g_tau(ck->t, ck->Y, ck->dY, ck->Y0, x_hat, Yg_hat, tau_hat);
  return J;
}

double dt;
arma::vec Y0_offset;
arma::vec Yg_offset;
double time_offset;
arma::vec Yg_up_lim ;
arma::vec Yg_low_lim;
double tau_low_lim;
double tau_up_lim;
arma::mat process_noise;
double msr_noise;
double msr_noise_hat;
arma::mat init_params_variance;
double a_p;
bool enable_constraints;
arma::vec num_diff_step;
bool ekf_use_num_diff;
arma::mat M_r;

void loadParams()
{
  ros::NodeHandle nh_("~");
  if (!nh_.getParam("ekf_use_num_diff",ekf_use_num_diff)) ekf_use_num_diff = false;

  if (!nh_.getParam("dt",dt)) throw std::runtime_error("Failed to load param \"dt\"...");

  std::vector<double> temp;
  if (!nh_.getParam("Y0_offset",temp)) throw std::runtime_error("Failed to load param \"Y0_offset\"...");
  Y0_offset = arma::vec(temp);

  if (!nh_.getParam("Yg_offset",temp)) throw std::runtime_error("Failed to load param \"Yg_offset\"...");
  Yg_offset = arma::vec(temp);

  if (!nh_.getParam("time_offset",time_offset)) throw std::runtime_error("Failed to load param \"time_offset\"...");

  if (!nh_.getParam("Yg_up_lim",temp)) throw std::runtime_error("Failed to load param \"Yg_up_lim\"...");
  Yg_up_lim = arma::vec(temp);

  if (!nh_.getParam("Yg_low_lim",temp)) throw std::runtime_error("Failed to load param \"Yg_low_lim\"...");
  Yg_low_lim = arma::vec(temp);

  if (!nh_.getParam("tau_low_lim",tau_low_lim)) throw std::runtime_error("Failed to load param \"tau_low_lim\"...");
  if (!nh_.getParam("tau_up_lim",tau_up_lim)) throw std::runtime_error("Failed to load param \"tau_up_lim\"...");

  if (!nh_.getParam("process_noise",temp)) throw std::runtime_error("Failed to load param \"process_noise\"...");
  process_noise = arma::diagmat(arma::vec(temp));

  if (!nh_.getParam("msr_noise",msr_noise)) throw std::runtime_error("Failed to load param \"msr_noise\"...");

  if (!nh_.getParam("msr_noise_hat",msr_noise_hat)) throw std::runtime_error("Failed to load param \"msr_noise_hat\"...");

  if (!nh_.getParam("init_params_variance",temp)) throw std::runtime_error("Failed to load param \"init_params_variance\"...");
  init_params_variance = arma::diagmat(arma::vec(temp));

  if (!nh_.getParam("a_p",a_p)) throw std::runtime_error("Failed to load param \"a_p\"...");

  if (!nh_.getParam("enable_constraints",enable_constraints)) throw std::runtime_error("Failed to load param \"enable_constraints\"...");

  if (!nh_.getParam("ekf_num_diff_step",temp)) throw std::runtime_error("Failed to load param \"num_diff_step\"...");
  num_diff_step = arma::vec(temp);

  if (!nh_.getParam("M_r",temp)) throw std::runtime_error("Failed to load param \"M_r\"...");
  M_r = arma::diagmat(arma::vec(temp));
}

int main(int argc, char** argv)
{
  // ===========  Initialize the ROS node  ===============
  ros::init(argc, argv, "dmp_ekf_pos_test_node");
  ros::NodeHandle nh_("~");

  std::string path = ros::package::getPath("dmp_kf_test");

  loadParams();

  arma::vec theta_low_lim = arma::join_vert(Yg_low_lim, arma::vec({tau_low_lim}) );
  arma::vec theta_up_lim = arma::join_vert(Yg_up_lim, arma::vec({tau_up_lim}) );

  int N_params = theta_low_lim.size();
  arma::mat A_c = arma::join_vert(-arma::mat().eye(N_params, N_params), arma::mat().eye(N_params, N_params));
  arma::vec b_c = arma::join_vert(-theta_low_lim, theta_up_lim);

  // ===========  load DMP data  ===============
  std::string dmp_data_file = path + "/matlab/data/model/dmp_pos_data.bin";
  std::ifstream in(dmp_data_file, std::ios::in | std::ios::binary);
  if (!in) throw std::runtime_error("Failed to open file \"" + dmp_data_file + "\"...");
  std::shared_ptr<dmp_::DMP_pos> dmp_p;
  arma::vec P0d;
  arma::vec Yg0;
  double tau0;
  dmp_p = dmp_::DMP_pos::importFromFile(in);
  io_::read_mat(Yg0, in);
  io_::read_mat(P0d, in);
  io_::read_scalar(tau0, in);
  in.close();

  // ========================================

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

  if (!ekf_use_num_diff)
  {
    ekf.setStateTransFunJacob(&pStateTransFunJacob);
    ekf.setMsrFunJacob(&pMsrFunJacob);
  }

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
  std::string sim_data_file = path + "/matlab/data/sim/sim_DMPpEKFc_results.bin";
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
