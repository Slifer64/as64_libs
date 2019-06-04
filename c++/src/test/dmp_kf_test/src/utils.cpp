#include <dmp_kf_test/utils.h>
#include <io_lib/io_utils.h>
#include <math_lib/quaternions.h>

void simulatePosOrientDMP(std::shared_ptr<dmp_::DMP_pos> &dmp_p,
                          std::shared_ptr<dmp_::DMP_eo> &dmp_o,
                          const arma::vec &P0, const arma::vec &Q0,
                          const arma::vec &Pg, const arma::vec &Qg, double T, double dt,
                          arma::rowvec &Time, arma::mat &P_data, arma::mat &dP_data, arma::mat &ddP_data,
                          arma::mat &Q_data, arma::mat &vRot_data, arma::mat &dvRot_data)
{
  double t = 0.0;

  double x = 0.0;
  double dx = 0.0;

  arma::vec P = P0;
  arma::vec dP = arma::vec().zeros(3);
  arma::vec ddP = arma::vec().zeros(3);
  arma::vec Q = Q0;
  arma::vec vRot = arma::vec().zeros(3);
  arma::vec dvRot = arma::vec().zeros(3);

  double t_end = T;
  dmp_p->setTau(t_end);
  dmp_o->setTau(t_end);

  dmp_p->setY0(P0);
  dmp_o->setQ0(Q0);

  // simulate
  while (true)
  {
    // data logging
    Time = arma::join_horiz(Time, arma::vec({t}));
    P_data = arma::join_horiz(P_data, P);
    dP_data = arma::join_horiz(dP_data, dP);
    ddP_data = arma::join_horiz(ddP_data, ddP);
    Q_data = arma::join_horiz(Q_data, Q);
    vRot_data = arma::join_horiz(vRot_data, vRot);
    dvRot_data = arma::join_horiz(dvRot_data, dvRot);

    // DMP simulation
    ddP = dmp_p->calcYddot(x, P, dP, Pg);
    dvRot = dmp_o->calcRotAccel(x, Q, vRot, Qg);

    // Update phase variable
    dx = dmp_p->phaseDot(x);

    // Stopping criteria
    if (t>=t_end) break;

    // Numerical integration
    t = t + dt;
    x = x + dx*dt;
    P = P + dP*dt;
    dP = dP + ddP*dt;
    Q = math_::quatProd( math_::quatExp(vRot*dt), Q);
    vRot = vRot + dvRot*dt;
  }

}

void saveDMPdata(const std::string &dmp_data_file,
                 const std::shared_ptr<dmp_::DMP_pos> &dmp_p,
                 const std::shared_ptr<dmp_::DMP_eo> &dmp_o,
                 const arma::vec &Yg0, const arma::vec &Y0,
                 const arma::vec &Qg0, const arma::vec &Q0, double tau0)
{
  std::ofstream out(dmp_data_file, std::ios::out | std::ios::binary);
  if (!out) throw std::runtime_error("Failed to create file \"" + dmp_data_file + "\"...");
  dmp_p->exportToFile(out);
  dmp_o->exportToFile(out);
  io_::write_mat(Yg0, out);
  io_::write_mat(Y0, out);
  io_::write_mat(Qg0, out);
  io_::write_mat(Q0, out);
  io_::write_scalar(tau0, out);
  out.close();
}

void loadDMPdata(const std::string &dmp_data_file,
                 std::shared_ptr<dmp_::DMP_pos> *dmp_p,
                 std::shared_ptr<dmp_::DMP_eo> *dmp_o,
                 arma::vec *Yg0, arma::vec *Y0,
                 arma::vec *Qg0, arma::vec *Q0, double *tau0)
{
  std::ifstream in(dmp_data_file, std::ios::in | std::ios::binary);
  if (!in) throw std::runtime_error("Failed to open file \"" + dmp_data_file + "\"...");

  arma::vec Yg0_2;
  arma::vec Y0_2;
  arma::vec Qg0_2;
  arma::vec Q0_2;
  double tau0_2;

  std::shared_ptr<dmp_::DMP_pos> dmp_p_2 = dmp_::DMP_pos::importFromFile(in);
  std::shared_ptr<dmp_::DMP_eo> dmp_o_2 = dmp_::DMP_eo::importFromFile(in);
  io_::read_mat(Yg0_2, in);
  io_::read_mat(Y0_2, in);
  io_::read_mat(Qg0_2, in);
  io_::read_mat(Q0_2, in);
  io_::read_scalar(tau0_2, in);
  in.close();

  if (dmp_p) *dmp_p = dmp_p_2;
  if (dmp_o) *dmp_o = dmp_o_2;
  if (Yg0) *Yg0 = Yg0_2;
  if (Y0) *Y0 = Y0_2;
  if (Qg0) *Qg0 = Qg0_2;
  if (Q0) *Q0 = Q0_2;
  if (tau0) *tau0 = tau0_2;
}