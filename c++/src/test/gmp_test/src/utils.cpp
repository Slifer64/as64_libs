#include <gmp_test/utils.h>

void simulateGMP(std::vector<std::shared_ptr<gmp_::GMP>> &gmp, const arma::vec &y0,
                 const arma::vec &yg, double T, double dt,
                 arma::mat &Time, arma::mat &Y_data, arma::mat &dY_data, arma::mat &ddY_data)
{

  int Dim = gmp.size();
  arma::vec y = y0;
  arma::vec dy = arma::vec().zeros(Dim);
  arma::vec ddy = arma::vec().zeros(Dim);
  arma::vec z = arma::vec().zeros(Dim);
  arma::vec dz = arma::vec().zeros(Dim);

  double t = 0.0;
  double t_end = T;
  double tau = t_end;
  double x = 0;
  double x_dot = 1/tau;
  double x_ddot = 0;
  gmp_::Phase s(x, x_dot, x_ddot);

  for (int i=0; i<Dim; i++)
  {
    gmp[i]->setY0(y0(i));
    gmp[i]->setGoal(yg(i));
  }

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
      double yc = 0.0;
      double zc = 0.0;
      gmp[i]->update(s, y(i), z(i), yc, zc);
      dy(i) = gmp[i]->getYdot();
      dz(i) = gmp[i]->getZdot();
      double yc_dot = 0.0;
      ddy(i) = gmp[i]->getYddot(yc_dot);
    }

    // Update phase variable
    x_dot = 1/tau;

    // Stopping criteria
    if (t>=t_end) break;

    // Numerical integration
    t = t + dt;
    x = x + x_dot*dt;
    y = y + dy*dt;
    z = z + dz*dt;

    s = gmp_::Phase(x, x_dot, x_ddot);
  }

}


void simulateGMP(std::shared_ptr<gmp_::GMP> gmp, double y0, double yg, double T, double dt,
                 arma::rowvec &Time, arma::rowvec &y_data, arma::rowvec &dy_data, arma::rowvec &ddy_data)
{
  std::vector<std::shared_ptr<gmp_::GMP>> gmp_array(1);
  gmp_array[0] = gmp;

  simulateGMP(gmp_array, arma::vec({y0}), arma::vec({yg}), T, dt, Time, y_data, dy_data, ddy_data);

}

void simulateGMP_nDoF(std::shared_ptr<gmp_::GMP_nDoF> &gmp, const arma::vec &y0,
                 const arma::vec &yg, double T, double dt,
                 arma::mat &Time, arma::mat &Y_data, arma::mat &dY_data, arma::mat &ddY_data)
{

  int Dim = gmp->length();
  arma::vec y = y0;
  arma::vec dy = arma::vec().zeros(Dim);
  arma::vec ddy = arma::vec().zeros(Dim);
  arma::vec z = arma::vec().zeros(Dim);
  arma::vec dz = arma::vec().zeros(Dim);

  double t = 0.0;
  double t_end = T;
  double tau = t_end;
  double x = 0;
  double x_dot = 1/tau;
  double x_ddot = 0;
  gmp_::Phase s(x, x_dot, x_ddot);

  gmp->setY0(y0);
  gmp->setGoal(yg);

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
    arma::vec yc = arma::vec({0.0});
    arma::vec zc = arma::vec().zeros(Dim);
    gmp->update(s, y, z, yc, zc);
    dy = gmp->getYdot();
    dz = gmp->getZdot();
    arma::vec yc_dot = arma::vec({0.0});
    ddy = gmp->getYddot(yc_dot);

    // Update phase variable
    x_dot = 1/tau;

    // Stopping criteria
    if (t>=t_end) break;

    // Numerical integration
    t = t + dt;
    x = x + x_dot*dt;
    y = y + dy*dt;
    z = z + dz*dt;

    s = gmp_::Phase(x, x_dot, x_ddot);
  }

}
