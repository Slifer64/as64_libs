#include <dmp_test/utils.h>



void simulateDMP(std::vector<std::shared_ptr<dmp_::DMP_>> &dmp, const arma::vec &y0,
                 const arma::vec &yg, double T, double dt,
                 arma::mat &Time, arma::mat &Y_data, arma::mat &dY_data, arma::mat &ddY_data)
{
  std::shared_ptr<dmp_::CanonicalClock> can_clock_ptr = dmp[0]->can_clock_ptr;

  int Dim = dmp.size();
  double x = 0.0;
  double dx = 0.0;
  arma::vec ddy = arma::vec().zeros(Dim);
  arma::vec dy = arma::vec().zeros(Dim);
  arma::vec y = y0;
  double t = 0.0;
  arma::vec dz = arma::vec().zeros(Dim);
  arma::vec z = arma::vec().zeros(Dim);

  double t_end = T;
  can_clock_ptr->setTau(t_end);

  for (int i=0; i<Dim; i++) dmp[i]->setY0(y0(i));

  // simulate
  while (true)
  {
    // data logging
    Time = arma::join_horiz(Time, arma::vec({t}));
    Y_data = arma::join_horiz(Y_data, y);
    dY_data = arma::join_horiz(dY_data, dy);
    ddY_data = arma::join_horiz(ddY_data, ddy);
    // x_data = arma::join_horiz(x_data, arma::vec({x}));

    // DMP simulation
    for (int i=0; i<Dim; i++)
    {
      double y_c = 0.0;
      double z_c = 0.0;
      dmp[i]->update(x, y(i), z(i), yg(i), y_c, z_c);
      dy(i) = dmp[i]->getYdot();
      dz(i) = dmp[i]->getZdot();
      ddy(i) = dmp[i]->getYddot();
    }

    // Update phase variable
    dx = can_clock_ptr->getPhaseDot(x);

    // Stopping criteria
    if (t>=t_end) break;

    // Numerical integration
    t = t + dt;
    x = x + dx*dt;
    y = y + dy*dt;
    z = z + dz*dt;
  }

}


void simulateDMP(std::shared_ptr<dmp_::DMP_> &dmp, double y0, double yg, double T, double dt,
                 arma::rowvec &Time, arma::rowvec &y_data, arma::rowvec &dy_data, arma::rowvec &ddy_data)
{
  std::vector<std::shared_ptr<dmp_::DMP_>> dmp_array(1);
  dmp_array[0] = dmp;

  simulateDMP(dmp_array, arma::vec({y0}), arma::vec({yg}), T, dt, Time, y_data, dy_data, ddy_data);

}


void simulateDMPpos(std::shared_ptr<dmp_::DMP_pos> &dmp_p, const arma::vec &P0, const arma::vec &Pg,
                    double T, double dt, arma::rowvec &Time, arma::mat &P_data, arma::mat &dP_data, arma::mat &ddP_data)
{
  // set initial values
  double t = 0.0;

  double x = 0.0;
  double dx = 0.0;

  arma::vec P = P0;
  arma::vec dP = arma::vec().zeros(3);
  arma::vec ddP = arma::vec().zeros(3);

  double t_end = T;
  dmp_p->setTau(t_end);

  dmp_p->setY0(P0);

  // simulate
  while (true)
  {
    // data logging
    Time = arma::join_horiz(Time, arma::vec({t}));
    P_data = arma::join_horiz(P_data, P);
    dP_data = arma::join_horiz(dP_data, dP);
    ddP_data = arma::join_horiz(ddP_data, ddP);

    // DMP simulation
    ddP = dmp_p->calcYddot(x, P, dP, Pg);

    // Update phase variable
    dx = dmp_p->phaseDot(x);

    // Stopping criteria
    if (t>=t_end) break;

    // Numerical integration
    t = t + dt;
    x = x + dx*dt;
    P = P + dP*dt;
    dP = dP + ddP*dt;
  }

}


void simulateDMPeo_in_eo_space(std::shared_ptr<dmp_::DMP_eo> &dmp_o, const arma::vec &Q0, const arma::vec &Qg,
                               double T, double dt, arma::rowvec &Time, arma::mat &Q_data, arma::mat &vRot_data, arma::mat &dvRot_data)
{
  double t_end = T;
  dmp_o->setTau(t_end);

  double t = 0.0;
  double x = 0.0;
  double dx = 0.0;
  arma::vec Q = Q0;
  arma::vec Q_prev = Q;
  arma::vec rotVel = arma::vec().zeros(3);
  arma::vec rotAccel = arma::vec().zeros(3);
  // arma::vec rotAccel2 = arma::vec().zeros(3);
  arma::vec eo = dmp_::DMP_eo::quat2eo(Q0, Qg);
  arma::vec deo = arma::vec().zeros(3);
  arma::vec dy = arma::vec().zeros(3);
  arma::vec dz = arma::vec().zeros(3);

  dmp_o->setQ0(Q0);
  dmp_o->setQg(Qg);
  arma::vec y = dmp_o->getY(Q);
  arma::vec z = dmp_o->getZ(rotVel, Q);

  // simulate
  while (true)
  {
    // data logging
    Time = arma::join_horiz(Time, arma::vec({t}));
    Q_data = arma::join_horiz(Q_data, Q);
    vRot_data = arma::join_horiz(vRot_data, rotVel);
    dvRot_data = arma::join_horiz(dvRot_data, rotAccel);

    double tau_dot = 0;
    arma::vec yc_dot = arma::vec().zeros(3);

    // DMP simulation
    arma::vec yc = arma::vec().zeros(3);
    arma::vec zc = arma::vec().zeros(3);

    dmp_o->update(x, y, z, yc, zc);

    dy = dmp_o->getYdot();
    dz = dmp_o->getZdot();
    rotAccel = dmp_o->getRotAccel(Q, tau_dot, yc_dot);
    // rotAccel2 = dmp_o->calcRotAccel(x, Q, rotVel, Qg);

    // Update phase variable
    dx = dmp_o->phaseDot(x);

    // Stopping criteria
    if (t>1.5*t_end)
    {
      std::cerr << "Time limit reached... Stopping simulation!\n";
      break;
    }

    eo = dmp_::DMP_eo::quat2eo(Q, Qg);
    if (t>=t_end && norm(eo)<0.02) break;

    // Numerical integration
    t = t + dt;
    x = x + dx*dt;
    y = y + dy*dt;
    z = z + dz*dt;

    eo = y;
    dy = z/dmp_o->getTau();
    deo = dy;

    Q_prev = Q;
    Q = dmp_::DMP_eo::eo2quat(eo, Qg);
    if (arma::dot(Q_prev,Q)<0) Q = -Q;

    arma::vec Qe = dmp_::DMP_eo::quatError(Q, Qg);
    rotVel = dmp_::DMP_eo::deo2rotVel(deo, Qe);
  }

}


void simulateDMPeo_in_quat_space(std::shared_ptr<dmp_::DMP_eo> &dmp_o, const arma::vec &Q0, const arma::vec &Qg,
                                 double T, double dt, arma::rowvec &Time, arma::mat &Q_data, arma::mat &vRot_data, arma::mat &dvRot_data)
{

  double t_end = T;
  double tau = t_end;
  dmp_o->setTau(tau);

  double t = 0.0;
  double x = 0.0;
  double dx = 0.0;
  arma::vec Q = Q0;
  arma::vec rotVel = arma::vec().zeros(3);
  arma::vec rotAccel = arma::vec().zeros(3);

  double tau_dot = 0;

  dmp_o->setQ0(Q0);
  dmp_o->setQg(Qg);

  // simulate
  while (true)
  {
    // data logging
    Time = arma::join_horiz(Time, arma::vec({t}));
    Q_data = arma::join_horiz(Q_data, Q);
    vRot_data = arma::join_horiz(vRot_data, rotVel);
    dvRot_data = arma::join_horiz(dvRot_data, rotAccel);

    // DMP simulation
    arma::vec yc = arma::vec().zeros(3);
    arma::vec zc = arma::vec().zeros(3);
    double tau_dot = 0;
    arma::vec yc_dot = arma::vec().zeros(3);
    rotAccel = dmp_o->calcRotAccel(x, Q, rotVel, Qg, tau_dot, yc, zc, yc_dot);

    // Update phase variable
    dx = dmp_o->phaseDot(x);

    // Stopping criteria
    if (t>1.5*t_end)
    {
      std::cerr << "Time limit reached... Stopping simulation!\n";
      break;
    }

    arma::vec eo = dmp_::DMP_eo::quat2eo(Q, Qg);
    if (t>=t_end & arma::norm(eo)<0.02) break;

    // Numerical integration
    t = t + dt;
    tau = tau + tau_dot*dt;
    x = x + dx*dt;
    Q = dmp_::quatProd( dmp_::quatExp(rotVel*dt), Q);
    rotVel = rotVel + rotAccel*dt;
  }
}