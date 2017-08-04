#include "behaviour_planner.hpp"

vector<string> valid_next_states(string& state)
{
    vector<string> valid_states;
    if(state == "CS") {
        valid_states.push_back("KL");
    } else if (state == "KL") {
        valid_states.push_back("LK");
        valid_states.push_back("PLCL");
        valid_states.push_back("PLCR");
    } else if (state == "PLCL") {
        valid_states.push_back("PLCL");
        valid_states.push_back("KL");
        valid_states.push_back("LCL");
    } else if (state == "PLCR") {
        valid_states.push_back("PLCR");
        valid_states.push_back("KL");
        valid_states.push_back("LCR");
    } else if (state == "LCR") {
        valid_states.push_back("LRC");
        valid_states.push_back("KL");
    } else if (state == "LCL") {
        valid_states.push_back("LCL");
        valid_states.push_back("KL");
    }
    return valid_states;
}

string get_next_state(string state, const map<int,vector < vector<int> >>& predictions)
{
    vector<string> states = valid_next_states(state);
    for(int i = 0; i < states.size(); i++) {
        cout << states[i] << " ";
    }
    cout << endl;
    return state;
}

void Vehicle::realize_state(map<int,vector < vector<int> > > predictions)
{
	/*
    Given a state, realize it by adjusting acceleration and lane.
    Note - lane changes happen instantaneously.
    */
    string state = this->state;
    if(state.compare("CS") == 0) {
    	realize_constant_speed();
    } else if(state.compare("KL") == 0) {
    	realize_keep_lane(predictions);
    } else if(state.compare("LCL") == 0) {
    	realize_lane_change(predictions, "L");
    } else if(state.compare("LCR") == 0) {
    	realize_lane_change(predictions, "R");
    } else if(state.compare("PLCL") == 0) {
    	realize_prep_lane_change(predictions, "L");
    } else if(state.compare("PLCR") == 0) {
    	realize_prep_lane_change(predictions, "R");
    }
}

void Vehicle::realize_constant_speed()
{
	a = 0;
}

int Vehicle::_max_accel_for_lane(map<int,vector<vector<int> > > predictions, int lane, int s)
{
	int delta_v_til_target = target_speed - v;
  int max_acc = min(max_acceleration, delta_v_til_target);

  map<int, vector<vector<int> > >::iterator it = predictions.begin();
  vector<vector<vector<int> > > in_front;
  while(it != predictions.end()) {
    int v_id = it->first;
    vector<vector<int> > v = it->second;

    if((v[0][0] == lane) && (v[0][1] > s)) {
  	 in_front.push_back(v);
    }
    it++;
  }

  if(in_front.size() > 0) {
  	int min_s = 1000;
  	vector<vector<int>> leading = {};
  	for(int i = 0; i < in_front.size(); i++) {
  		if((in_front[i][0][1]-s) < min_s) {
        min_s = (in_front[i][0][1]-s);
        leading = in_front[i];
  		}
  	}
  	int next_pos = leading[1][1];
  	int my_next = s + this->v;
  	int separation_next = next_pos - my_next;
  	int available_room = separation_next - preferred_buffer;
  	max_acc = min(max_acc, available_room);
  }
  return max_acc;
}

void Vehicle::realize_keep_lane(map<int,vector< vector<int> > > predictions)
{
	this->a = _max_accel_for_lane(predictions, this->lane, this->s);
}

void Vehicle::realize_lane_change(map<int,vector< vector<int> > > predictions, string direction)
{
	int delta = -1;
  if (direction.compare("L") == 0) {
  	delta = 1;
  }
  this->lane += delta;
  int lane = this->lane;
  int s = this->s;
  this->a = _max_accel_for_lane(predictions, lane, s);
}

void Vehicle::realize_prep_lane_change(map<int,vector<vector<int> > > predictions, string direction)
{
  int delta = -1;
  if (direction.compare("L") == 0) {
  	delta = 1;
  }
  int lane = this->lane + delta;

  map<int, vector<vector<int> > >::iterator it = predictions.begin();
  vector<vector<vector<int> > > at_behind;
  while(it != predictions.end()) {
    int v_id = it->first;
    vector<vector<int> > v = it->second;

    if((v[0][0] == lane) && (v[0][1] <= this->s)) {
      at_behind.push_back(v);
    }
    it++;
  }
  if(at_behind.size() > 0) {
  	int max_s = -1000;
  	vector<vector<int> > nearest_behind = {};
  	for(int i = 0; i < at_behind.size(); i++) {
      if((at_behind[i][0][1]) > max_s) {
        max_s = at_behind[i][0][1];
  			nearest_behind = at_behind[i];
  		}
  	}
  	int target_vel = nearest_behind[1][1] - nearest_behind[0][1];
  	int delta_v = this->v - target_vel;
  	int delta_s = this->s - nearest_behind[0][1];
  	if(delta_v != 0) {
		  int time = -2 * delta_s/delta_v;
  		int a;
  		if (time == 0) {
  			a = this->a;
  		} else {
        a = delta_v/time;
  		}
  		if(a > this->max_acceleration) {
  			a = this->max_acceleration;
  		}
  		if(a < -this->max_acceleration) {
        a = -this->max_acceleration;
  		}
  		this->a = a;
  	}	else {
		  int my_min_acc = max(-this->max_acceleration,-delta_s);
  		this->a = my_min_acc;
  	}
  }
}
