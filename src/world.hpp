#ifndef __WORLD_HPP__
#define __WORLD_HPP__
#include <vector>
#include "spline.h"
#include "Eigen-3.3/Eigen/Dense"

using namespace std;
using namespace tk;
using namespace Eigen;

struct Path {
  vector<double> x;
  vector<double> y;
  int n;
};

class World
{
  private:
    double _loop_distance;
    spline _smooth_x;
    spline _smooth_y;
    spline _smooth_dx;
    spline _smooth_dy;
    vector<double> convert_to_cartesian(const double& s, const double& d) const;
    Eigen::VectorXd jerk_minimizer_trajectory(const vector<double>& start, const vector<double>& end, const double time_limit);

  public:
    World(double distance): _loop_distance(distance) {};
    void interpolate(const vector<double>& waypoints_x, const vector<double>& waypoints_y,
      const vector<double>& waypoints_s, const vector<double>&waypoints_dx,
      const vector<double>& waypoints_dy);
    Path create_path(const vector<double>& start_s, const vector<double>& end_s,
        const vector<double>& start_d, const vector<double>& end_d, const double t, int n);
    vector<double> calculate_other_cars_velocity(vector<double> sensor_fusion);

};

#endif
