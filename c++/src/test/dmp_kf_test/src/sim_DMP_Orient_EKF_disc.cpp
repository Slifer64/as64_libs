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
#include <kf_lib/EKF.h>
#include <math_lib/quaternions.h>

#include <dmp_kf_test/utils.h>

using namespace as64_;

struct OrientStateTransCookie
{
  arma::vec vRot;
  double Ts;

  OrientStateTransCookie(const arma::vec &vRot, double Ts)
  {
    this->vRot = vRot;
    this->Ts = Ts;
  }
};

struct OrientMsrCookie
{
  std::shared_ptr<dmp_::DMP_eo> dmp_o;
  double t;
  arma::vec Q;
  arma::vec vRot;
  arma::vec Q0;
  arma::vec Qg;
  double tau;

  OrientMsrCookie(std::shared_ptr<dmp_::DMP_eo> dmp_o, double t,
               const arma::vec &Q, const arma::vec &vRot, const arma::vec &Q0,
               const arma::vec &Qg, double tau)
  {
    this->dmp_o = dmp_o;
    this->t = t;
    this->Q = Q;
    this->vRot = vRot;
    this->Q0 = Q0;
    this->Qg = Qg;
    this->tau = tau;
  }
};

arma::vec oStateTransFun(const arma::vec &theta, void *cookie)
{
  OrientStateTransCookie *ck = static_cast<OrientStateTransCookie *>(cookie);

  arma::vec theta_next(theta.size());

  arma::vec eo = theta.subvec(0,2);
  arma::vec Qe = math_::quatExp(eo);
  arma::vec rotVel = ck->vRot;
  arma::vec deo = dmp_::DMP_eo::rotVel2deo(rotVel, Qe);

  theta_next.subvec(0,2) = eo + deo*ck->Ts;
  theta_next(3) = theta(3);

  return theta_next;
}

arma::vec oMsrFun(const arma::vec &theta, void *cookie)
{
  OrientMsrCookie *ck = static_cast<OrientMsrCookie *>(cookie);

  arma::vec Q = ck->Q;

  arma::vec eo_hat = theta.subvec(0,2);
  arma::vec Qg_hat = math_::quatProd( math_::quatExp(eo_hat), Q);
  double tau_hat = theta(3);
  double x_hat = ck->t / tau_hat;

  ck->dmp_o->setTau(tau_hat);
  arma::vec Y_out = ck->dmp_o->calcRotAccel(x_hat, Q, ck->vRot, Qg_hat);
  ck->dmp_o->setTau(ck->tau);

  return Y_out;
}


int main(int argc, char** argv)
{
  // ===========  Initialize the ROS node  ===============
  ros::init(argc, argv, "dmp_ekf_orient_test_node");
  ros::NodeHandle nh_("~");

  std::string path = ros::package::getPath("dmp_kf_test");

  double dt = 0.005;

  arma::vec Q0_offset = {1.0, 0.0, 0.0, 0.0};

  arma::vec Qg_offset = {0.9244, 0.1744, 0.2720, 0.2028};
  double time_offset = -2.5;

  double tau_low_lim = 1.0;
  double tau_up_lim = 30.0; // Inf;

  arma::mat process_noise = arma::diagmat(arma::vec({0.02, 0.02, 0.02, 0.05})); // Q
  double msr_noise = std::pow(0.05,2);
  double msr_noise_hat = 10;


  arma::mat init_params_variance = arma::diagmat( arma::vec({1, 1, 1, 1}) ); // P
  double a_pc = 0.99; // forgetting factor in fading memory KF
  double a_p = 1.002;

  int N_params = 4;
  arma::mat A_c = arma::join_vert(arma::rowvec({0,0,0,-1}), arma::rowvec({0,0,0,1}));
  arma::vec b_c = arma::vec({-tau_low_lim, tau_up_lim});
  bool enable_constraints = true;

  arma::vec num_diff_step = {0.001, 0.001, 0.001, 0.01};

  arma::mat M_r = 1*arma::mat().eye(3,3);

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

  std::shared_ptr<dmp_::CanonicalClock> can_clock_ptr = dmp_o->can_clock_ptr;
  can_clock_ptr->setTau(tau0);

  // DMP simulation
  // set initial values
  int Dim = 3;
  double t = 0.0;
  double x = 0.0;
  double dx = 0.0;
  arma::vec Q0 = math_::quatProd(Q0_offset, Q0d);
  arma::vec Q = Q0;
  arma::vec vRot = arma::vec().zeros(Dim);
  arma::vec dvRot = arma::vec().zeros(Dim);
  arma::vec F_ext = arma::vec().zeros(Dim);

  arma::rowvec Time;
  arma::mat Q_data;
  arma::mat vRot_data;
  arma::mat dvRot_data;
  arma::rowvec x_data;
  arma::mat Qg_data;
  arma::rowvec tau_data;
  arma::mat F_data;
  arma::mat Sigma_theta_data;

  arma::vec Qg = math_::quatProd(Qg_offset, Qg0);

  double t_end = tau0 + time_offset;
  double tau = t_end;
  can_clock_ptr->setTau(tau);

  double tau_hat = tau0;
  arma::vec Qg_hat = Qg0;
  arma::vec eo_hat = dmp_::DMP_eo::quat2eo(Q, Qg_hat);
  double x_hat = t/tau_hat;

  int N_out = vRot.size();
  arma::vec Y_out_hat = arma::vec().zeros(N_out);
  arma::vec Y_out = arma::vec().zeros(N_out);

  arma::vec theta = arma::join_vert(eo_hat, arma::vec({tau_hat}));

  arma::mat P_theta = arma::mat().eye(N_params, N_params) % init_params_variance;
  arma::mat Rn_hat = arma::mat().eye(N_out,N_out) * msr_noise_hat;
  arma::mat Qn = arma::mat().eye(N_params,N_params) % process_noise;

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
    arma::vec dvRot_hat = dmp_o->calcRotAccel(x_hat, Q, vRot, Qg_hat);
    dmp_o->setTau(tau);

    dvRot = dmp_o->calcRotAccel(x, Q, vRot, Qg) + Sigma_vn*arma::vec().randn(N_out);

    F_ext = M_r * (dvRot - dvRot_hat);

    Y_out = dvRot;
    // Y_out_hat = dvRot_hat;

    // Update phase variable
    dx = can_clock_ptr->getPhaseDot(x);

    // Stopping criteria
    double err_o = arma::norm(Qg-Q)/arma::norm(Qg);
    if (err_o <= 0.5e-2 & t>=t_end) break;

    if (t>=t_end)
    {
      std::cerr << "Time limit reached. Stopping simulation...\n";
      break;
    }

    // ========  KF measurement update  ========
    OrientMsrCookie msr_cookie(dmp_o, t, Q, vRot, Q0, Qg, tau);
    ekf.correct(Y_out, static_cast<void *>(&msr_cookie));

    // ========  KF time update  ========
    OrientStateTransCookie state_cookie(vRot, dt);
    ekf.predict(static_cast<void *>(&state_cookie));

    theta = ekf.theta;
    P_theta = ekf.P;

    // Numerical integration
    t = t + dt;
    x = x + dx*dt;
    Q = math_::quatProd( math_::quatExp(vRot*dt), Q );
    vRot = vRot + dvRot*dt;

    eo_hat = theta.subvec(0,2);
    Qg_hat = math_::quatProd( math_::quatExp(eo_hat), Q);
    tau_hat = theta(3);
    x_hat = t/tau_hat;

  }
  std::cout << "Elapsed time: " << timer.toc() << " sec\n";

  // ===========  write results  ===============
  std::string sim_data_file = path + "/matlab/data/orient_est_results.bin";
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
