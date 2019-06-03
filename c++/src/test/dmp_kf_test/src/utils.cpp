#include <dmp_kf_test/utils.h>

#include <math_lib/quaternions.h>

void simulatePosOrientDMP(std::shared_ptr<dmp_::DMP_pos> &dmp_p,
                          std::shared_ptr<dmp_::DMP_eo> &dmp_o,
                          const arma::vec &P0, const arma::vec &Q0,
                          const arma::vec &Pg, const arma::vec &Qg, double T, double dt,
                          arma::rowvec &Time, arma::mat &P_data, arma::mat &dP_data, arma::mat &ddP_data,
                          arma::mat &Q_data, arma::mat &vRot_data, arma::mat &dvRot_data)
{

  // set initial values
  std::shared_ptr<dmp_::CanonicalClock> can_clock_ptr = dmp_p->can_clock_ptr;

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
  can_clock_ptr->setTau(t_end);

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
    dx = can_clock_ptr->getPhaseDot(x);

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