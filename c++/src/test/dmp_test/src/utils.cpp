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

  int iters = 0;

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
    iters++;
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