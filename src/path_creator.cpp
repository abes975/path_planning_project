#include "path_creator.hpp"

using namespace std;

/* Use waypoint map information and fit a cubic polinomial cuve in order to
* connect all waypoints in regards to the waypoints_s
*/
void PathCreator::interpolate(const vector<double>& waypoints_x, const vector<double>& waypoints_y,
  const vector<double>& waypoints_s, const vector<double>&waypoints_dx,
  const vector<double>& waypoints_dy) {
  _smooth_x.set_points(waypoints_s, waypoints_x);
  _smooth_y.set_points(waypoints_s, waypoints_y);
  _smooth_dx.set_points(waypoints_s, waypoints_dx);
  _smooth_dy.set_points(waypoints_s, waypoints_dy);
}

/* Convert to cartesian point using interpolated cubic curve */
vector<double> PathCreator::convert_to_cartesian(const double& s, const double& d) const
{
  const double mod_s = fmod(s, _loop_distance);
  const double x = _smooth_x(mod_s);
  const double y = _smooth_y(mod_s);
  const double dx = _smooth_dx(mod_s);
  const double dy = _smooth_dy(mod_s);

  x += dx * d;
  y += dy * d;

  return {x, y};
}

XYPoints PathCreator::create_path(JMT jmt_s, JMT jmt_d, const double timestep, const int n) const {

  vector<double> xs;
  vector<double> ys;
  vector<double> p;

  for (int i = 0; i < n; i++) {

    double s = jmt_s.get(i * t);
    double d = jmt_d.get(i * t);

    vector<double> p = this->convert_sd_to_xy(s, d);

    xs.emplace_back(p[0]);
    ys.emplace_back(p[1]);
  }

  XYPoints path = {xs, ys, n};

  return path;
}
