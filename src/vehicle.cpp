#ifndef VEHICLE_HPP
#define VEHICLE_HPP
#include <iostream>
#include <random>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <map>
#include <string>
#include <iterator>

using namespace std;

class Vehicle {
public:
  int L = 1;
  int preferred_buffer = 6; // impacts "keep lane" behavior.
  int lane;
  int s;
  int v;
  int a;
  int target_speed;
  int lanes_available;
  int max_acceleration;
  int goal_lane;
  int goal_s;
  string state;
  bool can_change_line;
  int delta;

  /**
  * Constructor
  */
  Vehicle(int lane, int s, int v, int a);

  /**
  * Destructor
  */
  virtual ~Vehicle();
};
