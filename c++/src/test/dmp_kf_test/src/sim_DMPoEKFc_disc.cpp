#include <ros/ros.h>
#include <ros/package.h>
#include <memory>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <exception>

#include <io_lib/io_utils.h>
#include <dmp_lib/DMP/DMPo.h>
#include <kf_lib/EKF.h>
#include <math_lib/quaternions.h>

#include <dmp_kf_test/utils.h>

using namespace as64_;


struct OrientMsrCookie
{
  std::shared_ptr<dmp_::DMPo> dmp;
  double t;
  arma::vec Y;
  arma::vec dY;

  OrientMsrCookie(std::shared_ptr<dmp_::DMPo> dmp_o, double t, const arma::vec &Y, const arma::vec &dY)
  {
    this->dmp = dmp_o;
    this->t = t;
    this->Y = Y;
    this->dY = dY;
  }
};

arma::vec oStateTransFun(const arma::vec &theta, void *cookie)
{
  return theta;
}

arma::vec oMsrFun(const arma::vec &theta, void *cookie)
{
  OrientMsrCookie *ck = static_cast<OrientMsrCookie *>(cookie);
  arma::vec Yg_hat = theta.subvec(0,2);
  double tau_hat = theta(3);
  double x_hat = ck->t/tau_hat;
  double tau0 = ck->dmp->getTau();
  ck->dmp->setTau(tau_hat);
  arma::vec Y_out = ck->dmp->calcYddot(x_hat, ck->Y, ck->dY, Yg_hat);
  ck->dmp->setTau(tau0); // restore previous value of tau
  return Y_out;
}

arma::mat oStateTransFunJacob(const arma::vec &theta, void *cookie)
{
  int n_params = theta.size();
  arma::mat J = arma::mat().eye(n_params, n_params);
  return J;
}

arma::mat oMsrFunJacob(const arma::vec &theta, void *cookie)
{
  OrientMsrCookie *ck = static_cast<OrientMsrCookie *>(cookie);
  int n_params = theta.size();
  int i_end = n_params - 1;
  arma::vec Yg_hat = theta.subvec(0,i_end-1);
  double tau_hat = theta(i_end);
  double x_hat = ck->t / tau_hat;
  arma::vec Y0 = arma::vec().zeros(Yg_hat.size());
  arma::mat J = ck->dmp->getAcellPartDev_qg_tau(ck->t, ck->Y, ck->dY, Y0, x_hat, Yg_hat, tau_hat);
  return J;
}

double dt;
arma::vec q0_offset;
arma::vec qg_offset;
double time_offset;
double tau_low_lim;
double tau_up_lim;
arma::vec process_noise;
double msr_noise;
double msr_noise_hat;
arma::vec init_params_variance;
double a_p;
arma::vec num_diff_step;
bool ekf_use_num_diff;
bool enable_constraints;
arma::mat M_r;

void loadParams()
{
  ros::NodeHandle nh_("~");

  if (!nh_.getParam("dt",dt)) throw std::runtime_error("Failed to load param \"dt\"...");

  std::vector<double> temp;
  if (!nh_.getParam("q0_offset",temp)) throw std::runtime_error("Failed to load param \"q0_offset\"...");
  q0_offset = arma::vec(temp);

  if (!nh_.getParam("qg_offset",temp)) throw std::runtime_error("Failed to load param \"qg_offset\"...");
  qg_offset = arma::vec(temp);

  if (!nh_.getParam("time_offset",time_offset)) throw std::runtime_error("Failed to load param \"time_offset\"...");

  if (!nh_.getParam("tau_low_lim",tau_low_lim)) throw std::runtime_error("Failed to load param \"tau_low_lim\"...");
  if (!nh_.getParam("tau_up_lim",tau_up_lim)) throw std::runtime_error("Failed to load param \"tau_up_lim\"...");

  if (!nh_.getParam("process_noise",temp)) throw std::runtime_error("Failed to load param \"process_noise\"...");
  process_noise = arma::vec(temp);

  if (!nh_.getParam("msr_noise",msr_noise)) throw std::runtime_error("Failed to load param \"msr_noise\"...");

  if (!nh_.getParam("msr_noise_hat",msr_noise_hat)) throw std::runtime_error("Failed to load param \"msr_noise_hat\"...");

  if (!nh_.getParam("init_params_variance",temp)) throw std::runtime_error("Failed to load param \"init_params_variance\"...");
  init_params_variance = arma::vec(temp);

  if (!nh_.getParam("a_p",a_p)) throw std::runtime_error("Failed to load param \"a_p\"...");

  if (!nh_.getParam("enable_constraints",enable_constraints)) throw std::runtime_error("Failed to load param \"enable_constraints\"...");

  if (!nh_.getParam("ekf_use_num_diff",ekf_use_num_diff)) throw std::runtime_error("Failed to load param \"ekf_use_num_diff\"...");

  if (!nh_.getParam("num_diff_step",temp)) throw std::runtime_error("Failed to load param \"num_diff_step\"...");
  num_diff_step = arma::vec(temp);

  if (!nh_.getParam("M_r",temp)) throw std::runtime_error("Failed to load param \"M_r\"...");
  M_r = arma::diagmat(arma::vec(temp));
}

int main(int argc, char** argv)
{
  // ===========  Initialize the ROS node  ===============
  ros::init(argc, argv, "sim_DMPoEKFc_disc_node");
  ros::NodeHandle nh_("~");

  std::string path = ros::package::getPath("dmp_kf_test");

  loadParams();

  int N_params = 4;
  arma::mat A_c = arma::join_vert(arma::rowvec({0,0,0,-1}), arma::rowvec({0,0,0,1}));
  arma::vec b_c = arma::vec({-tau_low_lim, tau_up_lim});

  // ===========  load DMP data  ===============
  std::string dmp_data_file = path + "/matlab/data/model/dmp_o_data.bin";
  std::ifstream in(dmp_data_file, std::ios::in | std::ios::binary);
  if (!in) throw std::runtime_error("Failed to open file \"" + dmp_data_file + "\"...");
  std::shared_ptr<dmp_::DMPo> dmp_o;
  arma::vec Qg0;
  arma::vec Q0d;
  double tau0;
  dmp_o = dmp_::DMPo::importFromFile(in);
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
  arma::vec Q0 = math_::quatProd(math_::quatExp(q0_offset), Q0d);
  arma::vec Q = Q0;
  arma::vec vRot = arma::vec().zeros(Dim);
  arma::vec dvRot = arma::vec().zeros(Dim);
  arma::vec F_ext = arma::vec().zeros(Dim);
  arma::vec Q1 = dmp_::DMPo::quatTf(Q, Q0);
  arma::vec q = math_::quatLog(Q1);
  arma::vec q_dot = arma::vec().zeros(3);

  arma::rowvec Time;
  arma::mat Q_data;
  arma::mat vRot_data;
  arma::mat dvRot_data;
  arma::rowvec x_data;
  arma::mat Qg_data;
  arma::rowvec tau_data;
  arma::mat F_data;
  arma::mat Sigma_theta_data;

  arma::vec Qg = math_::quatProd(math_::quatExp(qg_offset), Qg0);
  arma::vec qg = dmp_::DMPo::quat2q(Qg, Q0);

  double t_end = tau0 + time_offset;
  double tau = t_end;
  can_clock_ptr->setTau(tau);

  double tau_hat = tau0;
  arma::vec Qg_hat = Qg0;
  arma::vec qg_hat = dmp_::DMPo::quat2q(Qg_hat, Q0);
  double x_hat = t/tau_hat;

  int N_out = vRot.size();
  arma::vec Y_out_hat = arma::vec().zeros(N_out);
  arma::vec Y_out = arma::vec().zeros(N_out);

  arma::vec theta = arma::join_vert(qg_hat, arma::vec({tau_hat}));

  arma::mat P_theta = arma::diagmat(init_params_variance);
  arma::mat Rn_hat = arma::mat().eye(N_out,N_out) * msr_noise_hat;
  arma::mat Qn = arma::diagmat(process_noise);

  arma::arma_rng::set_seed(0);
  arma::mat Rn = arma::mat().eye(N_out,N_out) * msr_noise;
  arma::mat Sigma_vn = arma::sqrtmat_sympd(Rn);

  // Set up EKF object
  kf_::EKF ekf(theta, P_theta, N_out, &oStateTransFun, &oMsrFun);
  ekf.setProcessNoiseCov(Qn);
  ekf.setMeasureNoiseCov(Rn_hat);
  // a_p = exp(a_pc*dt);
  ekf.setFadingMemoryCoeff(a_p);

  ekf.enableParamsContraints(enable_constraints);
  ekf.setParamsConstraints(A_c, b_c);
  ekf.setPartDerivStep(num_diff_step);

  if (!ekf_use_num_diff)
  {
    ekf.setStateTransFunJacob(&oStateTransFunJacob);
    ekf.setMsrFunJacob(&oMsrFunJacob);
  }

  dmp_o->setQ0(Q0);

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

    Qg_data = arma::join_horiz(Qg_data, Qg_hat);
    tau_data = arma::join_horiz(tau_data, arma::vec({tau_hat}));

    x_data = arma::join_horiz(x_data, arma::vec({x}));
    F_data = arma::join_horiz(F_data, F_ext);
    Sigma_theta_data = arma::join_horiz(Sigma_theta_data, arma::sqrt(arma::vectorise(P_theta)));

    // DMP simulation
    dmp_o->setTau(tau_hat);
    arma::vec q_ddot_hat = dmp_o->calcYddot(x_hat, q, q_dot, qg_hat);
    // arma::vec dvRot_hat = dmp_o->calcRotAccel(x_hat, Q, vRot, Qg_hat);
    dmp_o->setTau(tau);

    arma::vec q_ddot = dmp_o->calcYddot(x, q, q_dot, qg) + Sigma_vn*arma::vec().randn(N_out);
    // arma::vec q_ddot = dmp_o->calcYddot(x, q, q_dot, qg);
    // arma::vec dvRot = dmp_o->calcRotAccel(x, Q, vRot, Qg);

    arma::mat JQq = dmp_::DMPo::jacobQq(Q1);
    F_ext = math_::quatProd( 2*JQq*(q_ddot - q_ddot_hat), math_::quatInv(Q1) );
    F_ext = M_r*F_ext.subvec(1,3);

    // arma::vec Fext2 = M_r*(dvRot-dvRot_hat);
    // double Fext_err = arma::norm(F_ext - Fext2);
    // if (Fext_err > 1e-6) std::cerr << "Fext_err = " << Fext_err << "\n";

    Y_out = q_ddot;
    // Y_out_hat = q_ddot_hat;

    // Update phase variable
    dx = can_clock_ptr->getPhaseDot(x);

    // Stopping criteria
    double err_o = arma::norm( math_::quatLog( math_::quatProd(Qg,math_::quatInv(Q)) ) );
    if (err_o <= 0.5e-2 & t>=t_end) break;

    if (t>=t_end)
    {
      std::cerr << "Time limit reached. Stopping simulation...\n";
      break;
    }

    // ========  KF measurement update  ========
    OrientMsrCookie msr_cookie(dmp_o, t, q, q_dot);
    ekf.correct(Y_out, static_cast<void *>(&msr_cookie));

    // ========  KF time update  ========
    ekf.predict();

    theta = ekf.theta;
    P_theta = ekf.P;

    // Numerical integration
    t = t + dt;
    x = x + dx*dt;
    q = q + q_dot*dt;
    q_dot = q_dot + q_ddot*dt;

    Q = dmp_::DMPo::q2quat(q, Q0);
    Q1 = dmp_::DMPo::quatTf(Q, Q0);
    vRot = dmp_::DMPo::qdot2rotVel(q_dot, Q1);
    dvRot = dmp_::DMPo::qddot2rotAccel(q_ddot, vRot, Q1);

    qg_hat = theta.subvec(0,2);
    Qg_hat = dmp_::DMPo::q2quat(qg_hat, Q0);
    tau_hat = theta(3);
    x_hat = t/tau_hat;
  }
  std::cout << "Elapsed time: " << timer.toc() << " sec\n";

  // ===========  write results  ===============
  std::string sim_data_file = path + "/matlab/data/sim/sim_DMPoEKFc_results.bin";
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
  out.close();

  // ===========  Shutdown ROS node  ==================
  ros::shutdown();

  return 0;
}
