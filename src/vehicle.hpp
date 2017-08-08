#ifndef __VEHICLE_HPP__
#define __VEHICLE_HPP__

#include <iostream>
#include <random>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <unordered_map>
#include <string>
#include <iterator>
#include "behaviour_planner.hpp"

using namespace std;

class Vehicle {
private:
  int _buffer_distance = 6;
  double _s;
  double _d;
  double _v_s;
  double _v_d;
  // Previous value...used to get acceleration
  double _pv_s;
  double _pv_d;
  double _a_s;
  double _a_d;
  double _target_speed;
  int _lane;
  //BehaviourPlanner::State state;
  vector <int> _start_s;
  vector <int> _end_s;
  vector <int> _start_d;
  vector <int> _end_d;

  //vector<double> state_at(Vehicle&v , double t);

public:

  /**
  * Constructor
  */
  Vehicle();
  Vehicle(double s, double d, double v_s, double v_d);

  void set_target_speed(double target_speed);
  void update_position(double cur_s, double cur_d, double speed_s, double speed_d, double t = 0.2);

  vector<double> state_at(double t);


  //void update_position(double cur_s, double cur_d, double speed_s, double speed_d, double t);
  //void update_state(unordered_map<int, Vehicle>& other_cars, int horizon);
  //void realize_state(vector<vector<double> >& predictions);

  // string Vehicle::to_string() const;
  //

  //
  // string display();
  //
  //
  //
  // vector<vector<double>> increment(int horizon);
  // //
  // //
  // // bool collides_with(Vehicle other, int at_time);
  // //
  // // collider will_collide_with(Vehicle other, int timesteps);
  // //
  // // void realize_state(map<int, vector < vector<int> > > predictions);
  // //
  // void realize_constant_speed();
  //
  // int _max_accel_for_lane(vector<vector<double>>& predictions, int lane, int s);
  //
  // void realize_keep_lane(vector<vector<double>>& predictions);
  //
  // void realize_lane_change(vector<vector<double>>&, string direction);
  //
  // void realize_prep_lane_change(vector<vector<double>>&, string direction);
  //
  // vector<vector<double> > generate_future_positions(unordered_map<int, Vehicle>, int horizon);


};

#endif
