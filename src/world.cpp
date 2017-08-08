#include "world.hpp"
#include <iostream>
#include <cmath>
#include <vector>

//#include "Eigen-3.3/Eigen/Core"
//#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;



using namespace std;

/* Use waypoint map information and fit a cubic polinomial cuve in order to
* connect all waypoints in regards to the waypoints_s
*/
void World::interpolate(const vector<double>& waypoints_x, const vector<double>& waypoints_y,
  const vector<double>& waypoints_s, const vector<double>&waypoints_dx,
  const vector<double>& waypoints_dy) {
  _smooth_x.set_points(waypoints_s, waypoints_x);
  _smooth_y.set_points(waypoints_s, waypoints_y);
  _smooth_dx.set_points(waypoints_s, waypoints_dx);
  _smooth_dy.set_points(waypoints_s, waypoints_dy);
}

/* Convert to cartesian point using interpolated cubic curve */
vector<double> World::convert_to_cartesian(const double& s, const double& d) const
{
  const double mod_s = fmod(s, _loop_distance);
  double x = _smooth_x(mod_s);
  double y = _smooth_y(mod_s);
  const double dx = _smooth_dx(mod_s);
  const double dy = _smooth_dy(mod_s);

  x += dx * d;
  y += dy * d;

  return {x, y};
}

vector <double> World::calculate_other_cars_velocity(vector<double> sensor_fusion) {
  vector<double> result;
  double vx = sensor_fusion[3];
  double vy = sensor_fusion[4];
  double s = sensor_fusion[5];
  double d = sensor_fusion[6];

  double yaw = atan2(vy, vx);
  vector<double> catesian_t0 = convert_to_cartesian(s, d);
  // Get next interval x, y position on spline curve...
  vector<double> catesian_t1 = convert_to_cartesian(s + 1, d + 1);

  double lane_orientation = atan2(catesian_t1[1] - catesian_t0[1], catesian_t1[0] - catesian_t0[0]);
  if(lane_orientation < 0)
    lane_orientation += 2 * M_PI;
  double local_yaw = yaw - lane_orientation;
  double v = sqrt(pow(vx,2) + pow(vy,2));
  double speed_s = v * cos(local_yaw);
  double speed_d = d * sin(local_yaw);
  result.push_back(speed_s);
  result.push_back(speed_d);
  return result;
}


Eigen::VectorXd World::jerk_minimizer_trajectory(const vector<double>& start, const vector<double>& end, const double time_limit)
{
  const double t1 = time_limit;
  const double t2 = t1 * t1;
  const double t3 = t2 * t1;
  const double t4 = t3 * t1;
  const double t5 = t4 * t1;

  Eigen::VectorXd result = VectorXd(6);

  MatrixXd A = MatrixXd(3, 3);
  A <<  t3,     t4,      t5,
        3 * t2, 4 * t3,  5 * t4,
        6 * t1, 12 * t2, 20 * t3;

  MatrixXd B = MatrixXd(3,1);
  B << end[0] - (start[0] + start[1] * t1 + .5 * start[2] * t2),
       end[1] - (start[1] + start[2]* t1),
       end[2] - start[2];

  MatrixXd Ai = A.inverse();

  VectorXd C = Ai * B;

  result << start[0],
            start[1],
            .5 * start[2],
            C.data()[0],
            C.data()[1],
            C.data()[2];

  return result;
}

Path World::create_path(const vector<double>& start_s, const vector<double>& end_s,
                        const vector<double>& start_d, const vector<double>& end_d,
                        const double time_limit, const int n) {
  vector<double> x;
  vector<double> y;

  const double t0 = 1.0;
  const double t1 = time_limit;
  const double t2 = t1 * t1;
  const double t3 = t2 * t1;
  const double t4 = t3 * t1;
  const double t5 = t4 * t1;

  Eigen::VectorXd T = VectorXd(6);
  T << t0, t1, t2, t3, t4, t5;

  Eigen::VectorXd C_s = jerk_minimizer_trajectory(start_s, end_s, time_limit);
  Eigen::VectorXd C_d = jerk_minimizer_trajectory(start_d, end_d, time_limit);

  for (int i = 0; i < n; i++) {
    double s =  T.transpose() * C_s;
    double d = T.transpose() * C_d;
    vector<double> cartesian = convert_to_cartesian(s, d);
    x.push_back(cartesian[0]);
    y.push_back(cartesian[1]);
  }

  Path path;
  path.x = x;
  path.y = y;
  path.n = n;
  return path;
}
