#ifndef __PATH_CREATOR_HPP__
#define __PATH_CREATOR_HPP__
#include <vector>
#include "spline.h"

using namespace std;
using namespace tk;

struct XYPoints {
  vector<double> x;
  vector<double> y;
  int n;
}

class PathCreator
{
  private:
    double _loop_distance;
    spline _smooth_x;
    spline _smooth_y;
    spline _smooth_dx;
    spline _smooth_dy;
    vector<double> convert_to_cartesian(const double& s, const double& d) const;
    vector<double> jerk_minimizer_trajectory(vector<double> start, vector<double> end, double time_limit);

  public:
    PathCreator(double distance): _loop_distance(distance) {};
    void interpolate(const vector<double>& waypoints_x, const vector<double>& waypoints_y,
      const vector<double>& waypoints_s, const vector<double>&waypoints_dx,
      const vector<double>& waypoints_dy);
    XYPoints PathCreator::create_path(JMT jmt_s, JMT jmt_d, const double t, const int n) const;
    

};

#endif
