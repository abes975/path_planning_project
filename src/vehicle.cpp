#include <iostream>
#include <iostream>
#include <math.h>
#include <unordered_map>
#include <vector>
#include <string>
#include <iterator>
#include <algorithm>
#include "vehicle.hpp"
//#include "behaviour_planner.hpp"


/**
 * Initializes Vehicle
 */
Vehicle::Vehicle()
{
  _d = 0;
  _s = 0;
  _pv_s = 0;
  _pv_d = 0;
  _v_s = 0;
  _v_d = 0;
  _a_s = 0;
  _a_d = 0;
  _target_speed = 0;
}

void Vehicle::set_target_speed(double target_speed)
{
  _target_speed = target_speed;
}

Vehicle::Vehicle(double s, double d, double v_s, double v_d)
{
    _s = s;
    _d = d;
    _pv_s = _v_s;
    _pv_d = _v_d;
    _v_s = v_s;
    _v_d = v_d;
    _a_s = 0; // SAMPLING_TIME;
    _a_d = 0; // SAMPLING_TIME;
}

void Vehicle::update_position(double s, double d, double v_s, double v_d, double t)
{
  _s = s;
  _d = d;
  _pv_s = _v_s;
  _pv_d = _v_d;
  _v_s = v_s;
  _v_d = v_d;
  // Have to divide by time here delta v / t gives acceleration...
  _a_s = (_v_s - _pv_s) / t;
  _a_d = (_v_d - _pv_d) / t;
  //cout << "Acceleration _a_s = " << _a_s << " a_d = " << _a_d << endl;
}


// /*
//  Move vehicle in t seconds (assuming constant acceleration)
// */
// void Vehicle::update_position(double cur_s, double cur_d, double v_s, double v_d, double t)
// {
//   _s += _v_s * t + _a_s * t * t / 2;
//   _d += _v_d * t + _a_d * t * t / 2;
//   cout << "Vehicle is in line " << (int) (_d / 4) << " at position = " << _s << endl;
// }

/*
Predicts state of vehicle in t seconds (assuming constant acceleration)
*/
vector<double> Vehicle::state_at(double t)
{
    double s = _s + _v_s * t + _a_s * t * t / 2;
    double d = _d + _v_d * t + _a_d * t * t / 2;
    double v_s = _v_s + _a_s * t;
    double v_d = _v_d + _a_d * t;
    return {s, d, v_s, v_d, _a_s, _a_d};
}

#if 0
void Vehicle::update_state(unordered_map<int, Vehicle >& other_cars, int horizon)
{
     if(horizon < 1)
       horizon = 1;

     double cost;
     double min_cost = std::numeric_limits<double>::max();
     BehaviourPlanner behaviour_planner;
     vector<vector<double>> predictions = generate_future_positions(other_cars, horizon);

     vector<BehaviourPlanner::State> states = behaviour_planner.get_valid_next_states(state);
     for(int i = 0; i < states.size(); i++) {
         cost = 0;
         // create copy of our vehicle
         Vehicle experiment = *this;
         experiment.state = states[i];
         //cout << "Evaluating state: " << states[i] <<  endl;

         //experiment.realize_state(predictions);
         // Move the vehicle in the future....
         vector<vector<double> > future = experiment.increment(horizon);
         // Finally get a cost...
         /*
         cost = behaviour_planner.calculate_total_cost(experiment, future, predictions);
         //cout << "\tTotal cost for state " << experiment.state << " = " << cost << " curren min cost " << min_cost << " for state " << this->state << endl;
         if(cost < min_cost) {
           min_cost = cost;
           //cout << "\tNEW min cost is " << min_cost << " for state " << experiment.state << endl;
            this->state = experiment.state;
           this->start = future[1];
           this->end = future.back();
         }
         */
     }
     cout << "NEXT STATE WILL BE " << this->state << endl << endl;
}
#endif



//
// void Vehicle::update_position(double cur_s, double cur_d, double speed_s, double speed_d, double t)
// {
//   s += speed_s * t + a * t * t / 2;
//   d += speed_d * t;
//   cout << "Vehicle is in line " << (int) (d / 4) << " at position = " << s << endl;
// }

//
// void Vehicle::update_position(double cur_s, double cur_d, double speed_s, double speed_d, double t)
// {
//   s = cur_s + speed_s * t;
//   d = cur_d + speed_d * t;
//   cout << "Vehicle is in line " << (int) (d / 4) << " at position = " << s << endl;
// }


// vector<double> Vehicle::state_at(Vehicle& v, double t)
// {
//   s += v_s * t;
//   d += v_d * t;
//   lane = (int) (d/4);
//   cout << "Vehicle is in line " << lane << " at position = " << s << endl;
//   return {s, d};
// }



// // don't know what to do with this....let's wait and see
// void filter_prediction_by_lane(const map<int, vector< vector<int> >>& pred, int lane, map<int, vector< vector<int> >>& filtered)
// {
//   map<int, vector< vector<int> >>::const_iterator cit;
//   for(cit = pred.begin(); cit!= pred.end(); ++cit) {
//     if (cit->first != -1) {
//       const vector< vector<int> >& data = cit->second;
//       for(int i = 0; i < data.size(); i++)
//           if(data[i][0] == lane) {
//             filtered[cit->first] = data;
//             break;
//           }
//     }
//   }
// }


//
// vector<vector<double>> Vehicle::increment(int horizon)
// {
//   vector<vector<double> > res(horizon, vector<double>(0));
//   vector<double> r;
//   double old_s = this->s;
//   double old_d = this->d;
//   while (horizon) {
//     this->s += this->v_s * 0.2;
//     this->d += this->v_d * 0.2;
//     r.push_back(this->s);
//     r.push_back(this->d);
//     res.push_back(r);
//     horizon--;
//   }
//   this->s = old_s;
//   this->d = old_d;
// }
//
// # if 0
//
// string Vehicle::display()
// {
// 	ostringstream oss;
//
//   oss << "s:    " << this->s << "\n";
//   oss << "lane: " << this->lane << "\n";
//   oss << "v:    " << this->v << "\n";
//   oss << "a:    " << this->a << "\n";
//
//   return oss.str();
// }
//
//
//
// vector<int> Vehicle::state_at(int t)
// {
//   	/*
//     Predicts state of vehicle in t seconds (assuming constant acceleration)
//     */
//     int s = this->s + this->v * t + this->a * t * t / 2;
//     int v = this->v + this->a * t;
//     return {this->lane, s, v, this->a};
// }
//
// // bool Vehicle::collides_with(Vehicle other, int at_time)
// // {
// // 	/*
// //     Simple collision detection.
// //     */
// //     vector<int> check1 = state_at(at_time);
// //     vector<int> check2 = other.state_at(at_time);
// //     return (check1[0] == check2[0]) && (abs(check1[1]-check2[1]) <= L);
// // }
// //
// // Vehicle::collider Vehicle::will_collide_with(Vehicle other, int timesteps)
// // {
// // 	Vehicle::collider collider_temp;
// // 	collider_temp.collision = false;
// // 	collider_temp.time = -1;
// //
// // 	for (int t = 0; t < timesteps+1; t++) {
// // 	 if( collides_with(other, t) ){
// // 			collider_temp.collision = true;
// // 			collider_temp.time = t;
// //     	return collider_temp;
// //   	}
// // 	}
// // return collider_temp;
// // }
//
// #endif
//
// void Vehicle::realize_state(vector<vector<double> >& predictions)
// {
// 	/*
//     Given a state, realize it by adjusting acceleration and lane.
//     Note - lane changes happen instantaneously.
//     */
//     BehaviourPlanner::State state = state;
//     if(state == BehaviourPlanner::State::CONSTANT_SPEED) {
//     	realize_constant_speed();
//     } else if(state == BehaviourPlanner::State::KEEP_LANE) {
//     	realize_keep_lane(predictions);
//     } else if(state == BehaviourPlanner::State::CHANGE_LANE_LEFT) {
//     	realize_lane_change(predictions, "L");
//     } else if(state == BehaviourPlanner::State::CHANGE_LANE_RIGHT) {
//     	realize_lane_change(predictions, "R");
//     } else if(state == BehaviourPlanner::State::PREPARE_CHANGE_LANE_LEFT) {
//     	realize_prep_lane_change(predictions, "L");
//     } else if(state == BehaviourPlanner::State::CHANGE_LANE_RIGHT) {
//     	realize_prep_lane_change(predictions, "R");
//     }
// }
//
// void Vehicle::realize_constant_speed()
// {
// 	a = 0;
// }
//
// int Vehicle::_max_accel_for_lane(vector<vector<double> >& predictions, int lane, int ss)
// {
//   int delta_v_til_target = target_speed - v_s;
//   int max_acc = min(max_acceleration, delta_v_til_target);
//
//   map<vector<double>>::iterator it = predictions.begin();
//   vector<vector<vector<double> > > in_front;
//   while(it != predictions.end()) {
//     vector<vector<double> > v = it->second;
//     if((v[0][0] == lane) && (v[0][1] > ss)) {
//   	 in_front.push_back(v);
//     }
//     it++;
//   }
//
//   if(in_front.size() > 0) {
//   	int min_s = 1000;
//   	vector<vector<int>> leading = {};
//   	for(int i = 0; i < in_front.size(); i++) {
//   		if((in_front[i][0][1] - ss) < min_s) {
//             min_s = (in_front[i][0][1]-ss);
//             leading = in_front[i];
//   		}
//   	}
//   	int next_pos = leading[1][1];
//   	int my_next = ss + this->v;
//   	int separation_next = next_pos - my_next;
//   	int available_room = separation_next - preferred_buffer;
//   	max_acc = min(max_acc, available_room);
//   }
//   return max_acc;
// }
//
// void Vehicle::realize_keep_lane(vector<vector<double> >& predictions)
// {
// 	this->a = _max_accel_for_lane(predictions, this->lane, this->s);
// }
//
// void Vehicle::realize_lane_change(vector<vector<double> >&, string direction)
// {
// 	int delta = -1;
//   if (direction.compare("L") == 0) {
//   	delta = 1;
//   }
//   this->lane += delta;
//   int lane = this->lane;
//   int s = this->s;
//   this->a = _max_accel_for_lane(predictions, lane, s);
// }
//
// void Vehicle::realize_prep_lane_change(vector<vector<double> >&, string direction)
// {
//
//   int delta = -1;
//   if (direction.compare("L") == 0) {
//   	delta = 1;
//   }
//   int lane = this->lane + delta;
//
//
//   map<int, vector<vector<int> > >::iterator it = predictions.begin();
//   vector<vector<vector<int> > > at_behind;
//   while(it != predictions.end()) {
//     int v_id = it->first;
//     vector<vector<int> > v = it->second;
//
//     if((v[0][0] == lane) && (v[0][1] <= this->s)) {
//       at_behind.push_back(v);
//     }
//     it++;
//   }
//
//   if(at_behind.size() > 0) {
//   	int max_s = -1000;
//   	vector<vector<int> > nearest_behind = {};
//   	for(int i = 0; i < at_behind.size(); i++) {
//       if((at_behind[i][0][1]) > max_s) {
//         max_s = at_behind[i][0][1];
//   			nearest_behind = at_behind[i];
//   		}
//   	}
//   	int target_vel = nearest_behind[1][1] - nearest_behind[0][1];
//   	int delta_v = this->v - target_vel;
//   	int delta_s = this->s - nearest_behind[0][1];
//   	if(delta_v != 0) {
// 		  int time = -2 * delta_s/delta_v;
//   		int a;
//   		if (time == 0) {
//   			a = this->a;
//   		} else {
//         a = delta_v/time;
//   		}
//   		if(a > this->max_acceleration) {
//   			a = this->max_acceleration;
//   		}
//   		if(a < -this->max_acceleration) {
//         a = -this->max_acceleration;
//   		}
//   		this->a = a;
//   	}	else {
// 		  int my_min_acc = max(-this->max_acceleration,-delta_s);
//   		this->a = my_min_acc;
//   	}
//   }
//   else
//     this->can_change_line = true;
//     this->delta = delta;
// }
//
// vector<vector<int> > Vehicle::generate_future_positions(unordered_map<int, Vehicle> other_cars, int horizon = 10)
// {
//   vector<vector<double> > predictions;
//   if(horizon < 1)
//     horizon = 1;
//   // Prediction at position 0 is current state!!!
//   unordered_map<int, Vehicle>::const_iterator cit;
//   for(cit = other_cars.begin(); cit != other_cars.end(); ++cit) {
//     for( int i = 0; i <= horizon; i++) {
//       vector<double> check1 = state_at(cit->second, i);
//       //vector<int> lane_s = {check1[0], check1[1]};
//       predictions.push_back(check1);
//     }
//   }
//   return predictions;
// }
