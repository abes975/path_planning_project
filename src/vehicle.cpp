#include <iostream>
#include "vehicle.hpp"
#include <iostream>
#include <math.h>
#include <map>
#include <string>
#include <iterator>

/**
 * Initializes Vehicle
 */
Vehicle::Vehicle(int s, int d, int speed, int a, int tgt_speed)
{
    this->s = s;
    this->d = d;
    this->v = speed;
    this->a = a;
    this->target_speed = tgt_speed;
}


void Vehicle::update_state(map<int,vector < vector<int> > > predictions)
{
    state = get_next_state(state, predictions);
}


string Vehicle::to_string()
{
	ostringstream oss;

	oss << "s: " << this->s << "\n";
    oss << "d: " << this->d << "\n";
    oss << "v: " << this->v << "\n";
    oss << "a: " << this->a << "\n";

    return oss.str();
}

void Vehicle::increment(int dt = 1)
{
	this->s += this->v * dt;
    this->v += this->a * dt;
}

vector<int> Vehicle::state_at(int t)
{
  	/*
    Predicts state of vehicle in t seconds (assuming constant acceleration)
    */
    int s = this->s + this->v * t + this->a * t * t / 2;
    int v = this->v + this->a * t;
    return {this->lane, s, v, this->a};
}

// bool Vehicle::collides_with(Vehicle other, int at_time)
// {
// 	/*
//     Simple collision detection.
//     */
//     vector<int> check1 = state_at(at_time);
//     vector<int> check2 = other.state_at(at_time);
//     return (check1[0] == check2[0]) && (abs(check1[1]-check2[1]) <= L);
// }
//
// Vehicle::collider Vehicle::will_collide_with(Vehicle other, int timesteps)
// {
// 	Vehicle::collider collider_temp;
// 	collider_temp.collision = false;
// 	collider_temp.time = -1;
//
// 	for (int t = 0; t < timesteps+1; t++) {
// 	 if( collides_with(other, t) ){
// 			collider_temp.collision = true;
// 			collider_temp.time = t;
//     	return collider_temp;
//   	}
// 	}
// return collider_temp;
// }


vector<vector<int> > Vehicle::generate_predictions(int horizon = 10)
{
  vector<vector<int> > predictions;
  for( int i = 0; i < horizon; i++) {
    vector<int> check1 = state_at(i);
    vector<int> lane_s = {check1[0], check1[1]};
    predictions.push_back(lane_s);
  }
  return predictions;
}
