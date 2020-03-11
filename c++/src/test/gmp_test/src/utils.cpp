#include <gmp_test/utils.h>



void simulategmp(std::vector<std::shared_ptr<gmp_::gmp_>> &gmp, const arma::vec &y0,
                 const arma::vec &yg, double T, double dt,
                 arma::mat &Time, arma::mat &Y_data, arma::mat &dY_data, arma::mat &ddY_data)
{
  std::shared_ptr<gmp_::CanonicalClock> can_clock_ptr = gmp[0]->can_clock_ptr;

  int Dim = gmp.size();
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

  for (int i=0; i<Dim; i++) gmp[i]->setY0(y0(i));

  // simulate
  while (true)
  {
    // data logging
    Time = arma::join_horiz(Time, arma::vec({t}));
    Y_data = arma::join_horiz(Y_data, y);
    dY_data = arma::join_horiz(dY_data, dy);
    ddY_data = arma::join_horiz(ddY_data, ddy);
    // x_data = arma::join_horiz(x_data, arma::vec({x}));

    // gmp simulation
    for (int i=0; i<Dim; i++)
    {
      double y_c = 0.0;
      double z_c = 0.0;
      gmp[i]->update(x, y(i), z(i), yg(i), y_c, z_c);
      dy(i) = gmp[i]->getYdot();
      dz(i) = gmp[i]->getZdot();
      ddy(i) = gmp[i]->getYddot();
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


void simulategmp(std::shared_ptr<gmp_::gmp_> &gmp, double y0, double yg, double T, double dt,
                 arma::rowvec &Time, arma::rowvec &y_data, arma::rowvec &dy_data, arma::rowvec &ddy_data)
{
  std::vector<std::shared_ptr<gmp_::gmp_>> gmp_array(1);
  gmp_array[0] = gmp;

  simulategmp(gmp_array, arma::vec({y0}), arma::vec({yg}), T, dt, Time, y_data, dy_data, ddy_data);

}


void simulategmppos(std::shared_ptr<gmp_::gmp_pos> &gmp_p, const arma::vec &P0, const arma::vec &Pg,
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
  gmp_p->setTau(t_end);

  gmp_p->setY0(P0);

  // simulate
  while (true)
  {
    // data logging
    Time = arma::join_horiz(Time, arma::vec({t}));
    P_data = arma::join_horiz(P_data, P);
    dP_data = arma::join_horiz(dP_data, dP);
    ddP_data = arma::join_horiz(ddP_data, ddP);

    // gmp simulation
    ddP = gmp_p->calcYddot(x, P, dP, Pg);

    // Update phase variable
    dx = gmp_p->phaseDot(x);

    // Stopping criteria
    if (t>=t_end) break;

    // Numerical integration
    t = t + dt;
    x = x + dx*dt;
    P = P + dP*dt;
    dP = dP + ddP*dt;
  }

}



void simulategmpo_in_log_space(std::shared_ptr<gmp_::gmpo> &gmp_o, const arma::vec &Q0, const arma::vec &Qg,
                               double T, double dt, arma::rowvec &Time, arma::mat &Q_data, arma::mat &vRot_data, arma::mat &dvRot_data)
{
  double t_end = T;

  double t = 0.0;
  double x = 0.0;
  double dx = 0.0;
  arma::vec Q = Q0;
  arma::vec Q_prev = Q;
  arma::vec rotVel = arma::vec().zeros(3);
  arma::vec rotAccel = arma::vec().zeros(3);
  arma::vec q = gmp_::gmpo::quat2q(Q, Q0);
  arma::vec qdot = arma::vec().zeros(3);
  arma::vec dy = arma::vec().zeros(3);
  arma::vec dz = arma::vec().zeros(3);

  gmp_o->setQ0(Q0);
  gmp_o->setTau(t_end);

  arma::vec y = gmp_o->getY(Q);
  arma::vec z = gmp_o->getZ(rotVel, Q);
  arma::vec g = gmp_o->quat2q(Qg, Q0);

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

    // gmp simulation
    arma::vec yc = arma::vec().zeros(3);
    arma::vec zc = arma::vec().zeros(3);

    gmp_o->update(x, y, z, g, yc, zc);

    dy = gmp_o->getYdot();
    dz = gmp_o->getZdot();
    rotAccel = gmp_o->getRotAccel(Q, tau_dot, yc_dot);

    // Update phase variable
    dx = gmp_o->phaseDot(x);

    // Stopping criteria
    if (t>1.5*t_end)
    {
      std::cerr << "Time limit reached... Stopping simulation!\n";
      break;
    }

    arma::vec eo = gmp_::quatLog(gmp_::quatProd(Qg, gmp_::quatInv(Q)));
    if (t>=t_end && norm(eo)<0.02) break;

    // Numerical integration
    t = t + dt;
    x = x + dx*dt;
    y = y + dy*dt;
    z = z + dz*dt;

    q = y;
    dy = z/gmp_o->getTau();
    qdot = dy;

    Q_prev = Q;
    Q = gmp_::gmpo::q2quat(q, Q0);
    if (arma::dot(Q_prev,Q)<0) Q = -Q;

    arma::vec Q1 = gmp_::gmpo::quatTf(Q, Q0);
    rotVel = gmp_::gmpo::qdot2rotVel(qdot, Q1);
  }

}


void simulategmpo_in_quat_space(std::shared_ptr<gmp_::gmpo> &gmp_o, const arma::vec &Q0, const arma::vec &Qg,
                                 double T, double dt, arma::rowvec &Time, arma::mat &Q_data, arma::mat &vRot_data, arma::mat &dvRot_data)
{
  double t_end = T;
  double tau = t_end;

  double t = 0.0;
  double x = 0.0;
  double dx = 0.0;
  arma::vec Q = Q0;
  arma::vec rotVel = arma::vec().zeros(3);
  arma::vec rotAccel = arma::vec().zeros(3);

  double tau_dot = 0;

  gmp_o->setQ0(Q0);
  gmp_o->setTau(tau);

  // simulate
  while (true)
  {
    // data logging
    Time = arma::join_horiz(Time, arma::vec({t}));
    Q_data = arma::join_horiz(Q_data, Q);
    vRot_data = arma::join_horiz(vRot_data, rotVel);
    dvRot_data = arma::join_horiz(dvRot_data, rotAccel);

    // gmp simulation
    arma::vec yc = arma::vec().zeros(3);
    arma::vec zc = arma::vec().zeros(3);
    double tau_dot = 0;
    arma::vec yc_dot = arma::vec().zeros(3);
    rotAccel = gmp_o->calcRotAccel(x, Q, rotVel, Qg, tau_dot, yc, zc, yc_dot);

    // Update phase variable
    dx = gmp_o->phaseDot(x);

    // Stopping criteria
    if (t>1.5*t_end)
    {
      std::cerr << "Time limit reached... Stopping simulation!\n";
      break;
    }

    arma::vec eo = gmp_::quatLog(gmp_::quatProd(Qg, gmp_::quatInv(Q)));
    if (t>=t_end & arma::norm(eo)<0.02) break;

    // Numerical integration
    t = t + dt;
    tau = tau + tau_dot*dt;
    x = x + dx*dt;
    Q = gmp_::quatProd( gmp_::quatExp(rotVel*dt), Q);
    rotVel = rotVel + rotAccel*dt;

    gmp_o->setTau(tau); // set tau, if it changes online
  }
}


void simulategmpeo_in_eo_space(std::shared_ptr<gmp_::gmp_eo> &gmp_o, const arma::vec &Q0, const arma::vec &Qg,
                               double T, double dt, arma::rowvec &Time, arma::mat &Q_data, arma::mat &vRot_data, arma::mat &dvRot_data)
{
  double t_end = T;
  gmp_o->setTau(t_end);

  double t = 0.0;
  double x = 0.0;
  double dx = 0.0;
  arma::vec Q = Q0;
  arma::vec Q_prev = Q;
  arma::vec rotVel = arma::vec().zeros(3);
  arma::vec rotAccel = arma::vec().zeros(3);
  arma::vec eo = gmp_::gmp_eo::quat2eo(Q0, Qg);
  arma::vec deo = arma::vec().zeros(3);
  arma::vec dy = arma::vec().zeros(3);
  arma::vec dz = arma::vec().zeros(3);

  gmp_o->setQ0(Q0);
  gmp_o->setQg(Qg);
  arma::vec y = gmp_o->getY(Q);
  arma::vec z = gmp_o->getZ(rotVel, Q);

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

    // gmp simulation
    arma::vec yc = arma::vec().zeros(3);
    arma::vec zc = arma::vec().zeros(3);

    gmp_o->update(x, y, z, yc, zc);

    dy = gmp_o->getYdot();
    dz = gmp_o->getZdot();
    rotAccel = gmp_o->getRotAccel(Q, tau_dot, yc_dot);
    // rotAccel2 = gmp_o->calcRotAccel(x, Q, rotVel, Qg);

    // Update phase variable
    dx = gmp_o->phaseDot(x);

    // Stopping criteria
    if (t>1.5*t_end)
    {
      std::cerr << "Time limit reached... Stopping simulation!\n";
      break;
    }

    eo = gmp_::gmp_eo::quat2eo(Q, Qg);
    if (t>=t_end && norm(eo)<0.02) break;

    // Numerical integration
    t = t + dt;
    x = x + dx*dt;
    y = y + dy*dt;
    z = z + dz*dt;

    eo = y;
    dy = z/gmp_o->getTau();
    deo = dy;

    Q_prev = Q;
    Q = gmp_::gmp_eo::eo2quat(eo, Qg);
    if (arma::dot(Q_prev,Q)<0) Q = -Q;

    arma::vec Qe = gmp_::gmp_eo::quatError(Q, Qg);
    rotVel = gmp_::gmp_eo::deo2rotVel(deo, Qe);
  }

}


void simulategmpeo_in_quat_space(std::shared_ptr<gmp_::gmp_eo> &gmp_o, const arma::vec &Q0, const arma::vec &Qg,
                                 double T, double dt, arma::rowvec &Time, arma::mat &Q_data, arma::mat &vRot_data, arma::mat &dvRot_data)
{

  double t_end = T;
  double tau = t_end;
  gmp_o->setTau(tau);

  double t = 0.0;
  double x = 0.0;
  double dx = 0.0;
  arma::vec Q = Q0;
  arma::vec rotVel = arma::vec().zeros(3);
  arma::vec rotAccel = arma::vec().zeros(3);

  double tau_dot = 0;

  gmp_o->setQ0(Q0);
  gmp_o->setQg(Qg);

  // simulate
  while (true)
  {
    // data logging
    Time = arma::join_horiz(Time, arma::vec({t}));
    Q_data = arma::join_horiz(Q_data, Q);
    vRot_data = arma::join_horiz(vRot_data, rotVel);
    dvRot_data = arma::join_horiz(dvRot_data, rotAccel);

    // gmp simulation
    arma::vec yc = arma::vec().zeros(3);
    arma::vec zc = arma::vec().zeros(3);
    double tau_dot = 0;
    arma::vec yc_dot = arma::vec().zeros(3);
    rotAccel = gmp_o->calcRotAccel(x, Q, rotVel, Qg, tau_dot, yc, zc, yc_dot);

    // Update phase variable
    dx = gmp_o->phaseDot(x);

    // Stopping criteria
    if (t>1.5*t_end)
    {
      std::cerr << "Time limit reached... Stopping simulation!\n";
      break;
    }

    arma::vec eo = gmp_::gmp_eo::quat2eo(Q, Qg);
    if (t>=t_end & arma::norm(eo)<0.02) break;

    // Numerical integration
    t = t + dt;
    tau = tau + tau_dot*dt;
    x = x + dx*dt;
    Q = gmp_::quatProd( gmp_::quatExp(rotVel*dt), Q);
    rotVel = rotVel + rotAccel*dt;
  }
}