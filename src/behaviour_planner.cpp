#include "behaviour_planner.hpp"

vector<BehaviourPlanner::State> BehaviourPlanner::get_valid_next_states(BehaviourPlanner::State& current_state)
{
    vector<BehaviourPlanner::State> valid_states;
    switch(current_state) {
      case BehaviourPlanner::State::BEGIN:
        valid_states.push_back(BehaviourPlanner::State::KEEP_LANE);
        break;
      case BehaviourPlanner::State::KEEP_LANE:
        valid_states.push_back(BehaviourPlanner::State::KEEP_LANE;
        valid_states.push_back(BehaviourPlanner::State::PREPARE_CHANGE_LANE_LEFT;
        valid_states.push_back(BehaviourPlanner::State::PREPARE_CHANGE_LANE_RIGHT;
        break;
      case BehaviourPlanner::State::PREPARE_CHANGE_LANE_LEFT:
        valid_states.push_back(BehaviourPlanner::State::KEEP_LANE;
        valid_states.push_back(BehaviourPlanner::State::PREPARE_CHANGE_LANE_LEFT;
        valid_states.push_back(BehaviourPlanner::State::CHANGE_LANE_LEFT;
        break;
      case BehaviourPlanner::State::PREPARE_CHANGE_LANE_RIGHT:
        valid_states.push_back(BehaviourPlanner::State::KEEP_LANE;
        valid_states.push_back(BehaviourPlanner::State::PREPARE_CHANGE_LANE_RIGHT;
        valid_states.push_back(BehaviourPlanner::State::CHANGE_LANE_RIGHT;
        break;
      case BehaviourPlanner::State::PREPARE_CHANGE_LANE_RIGHT:
        valid_states.push_back(BehaviourPlanner::State::KEEP_LANE;
        valid_states.push_back(BehaviourPlanner::State::PREPARE_CHANGE_LANE_RIGHT;
        valid_states.push_back(BehaviourPlanner::State::PREPARE_CHANGE_LANE_LEFT;
        break;
      case BehaviourPlanner::State::PREPARE_CHANGE_LANE_LEFT:
        valid_states.push_back(BehaviourPlanner::State::KEEP_LANE;
        valid_states.push_back(BehaviourPlanner::State::PREPARE_CHANGE_LANE_RIGHT;
        valid_states.push_back(BehaviourPlanner::State::PREPARE_CHANGE_LANE_LEFT;
        break;
      default:
        throw std::runtime_error(string("Unkown state passed...cannot continue");
    }
    return valid_states;
}

#if 0

void BehaviourPlanner::realize_state(Vehicle& my_car, map<int,vector < vector<int> > > predictions)
{
	/*
    Given a state, realize it by adjusting acceleration and lane.
    Note - lane changes happen instantaneously.
    */
    BehaviourPlanner::State state = my_car.state;
    if(state == BehaviourPlanner::State::CONSTANT_SPEED) {
    	my_car.realize_constant_speed();
    } else if(state == BehaviourPlanner::State::KEEP_LANE) {
    	my_car.realize_keep_lane(predictions);
    } else if(state == BehaviourPlanner::State::CHANGE_LANE_LEFT) == 0) {
    	my_car.realize_lane_change(predictions, "L");
    } else if(state == BehaviourPlanner::State::CHANGE_LANE_RIGHT) {
    	my_car.realize_lane_change(predictions, "R");
    } else if(state == BehaviourPlanner::State::PREPARE_CHANGE_LANE_LEFT) {
    	my_car.realize_prep_lane_change(predictions, "L");
    } else if(state == BehaviourPlanner::State::CHANGE_LANE_RIGHT) {
    	my_car.realize_prep_lane_change(predictions, "R");
    }
}

struct cost_data
{
  	int proposed_lane;
		double avg_speed;
		int max_accel;
		double rms_acceleration;
		int closest_approach;
		int end_distance_to_goal;
		int end_lanes_from_goal;
		bool collides;
    int collision_time;
};


double change_lane_cost(const Vehicle& v, struct cost_data& data)
{
	int proposed_lanes = data.end_lanes_from_goal;
  // VEry bad name...
	int cur_lanes = data.proposed_lane;
	double cost = 0.0;
	if (proposed_lanes > cur_lanes)
		cost = abs(proposed_lanes- cur_lanes) * 10000;
	if (proposed_lanes < cur_lanes)
	 	cost = -10000 * abs(proposed_lanes- cur_lanes);
	if (cost != 0)
	//	cout << "!!cost for lane change is " << cost << endl;
  //cout << "Change lane cost = " << cost << endl;
	return cost;
}

double out_of_the_road_cost(const Vehicle& v, struct cost_data& data)
{
  double cost = 0;
  if (data.proposed_lane < 0 || data.proposed_lane > 3)
    cost = 1000000;
  cout << "Out of the road cost " << cost << endl;
  return cost;
}


double distance_from_goal_lane(const Vehicle& v, struct cost_data& data)
{
	int distance = abs(data.end_distance_to_goal);
	distance = max(distance, 1);
  //cout << "dentro costo distanza distance from goal = " << distance << " avg_speed = " << data.avg_speed << endl;

	double time_to_goal = double(distance) / data.avg_speed;
	int lanes = data.end_lanes_from_goal;
	double multiplier = double(5 * lanes) / time_to_goal;
  //cout << "dentro costo distanza Ma anche qui come cazzo " << lanes << " time to goal " << time_to_goal << " multiplier = " << multiplier << endl;
	double cost = multiplier * 100000;
  cout << "distance goal lane cost = " << cost << endl;
	return cost;
}

double inefficiency_cost(const Vehicle& v, struct cost_data& data)
{
	double speed = data.avg_speed;
	double target_speed = v.target_speed;
  //cout << " trajectory_avg_speed = " << speed << " target_speed = " << target_speed << endl;
	int diff = target_speed - speed;
	double pct = double (diff) / target_speed;
	double multiplier = pow(pct,2);
	double cost = multiplier * 1000;
  cout << "inefficenciency cost = " << cost << endl;
  return cost;
}

double collision_cost(const Vehicle& v, struct cost_data& data)
{
  double cost = 0;
  if (data.collides) {
    //cout << "MERDISSIMA WE WILL HAVE A COLLISION AT TIME = " << data.collision_time << endl;
		int time_till_collision = data.collision_time;
		//exponent = (float(time_til_collision) ) ** 2
		double mult = exp(-pow(time_till_collision,2));
		cost = mult * 1000000;
  }
  cout << "Collision cost = " << cost << endl;
  return cost;
}

double buffer_cost(const Vehicle& v, struct cost_data& data)
{
  double cost;
	int closest = data.closest_approach;
  // This means collision....
	if (closest == 0) {
    cost = 10 * 100000;
    cout << "buffer cost = " << cost << endl;
    return cost;
  }

	double timesteps_away = closest / data.avg_speed;
	if(timesteps_away > 1.5) {
    cost = 0.0;
    cout << "buffer cost = " << cost << endl;
		return cost;
  }

	double multiplier = 1.0 - pow((timesteps_away / 1.5), 2);
	cost =  multiplier * 100000;
  cout << "buffer cost = " << cost << endl;
  return cost;
}

struct AccelCmp {
  bool operator()(const int a, const int b) {
    return abs(a) > abs(b);
  }
};

double calculate_total_cost(const Vehicle& v, vector<vector<int> >& future_traj,
            const map<int, vector<vector<int> >>& predictions)
{
    double cost = 0.0;
    AccelCmp acc_cmp;
    map<int,vector < vector<int> >> filtered;
    int L = 1;
    // future[i][0] = lane for time i
    // future[i][1] = s for time i
    vector<int> before_move = future_traj[0];
    vector<int> start = future_traj[1];
    vector<int> end = future_traj.back();
    int end_distance_to_goal = v.goal_s - end[1];
    //cout << "dentro calculate_total_cost final trajectory lane should be " << end[0]  << "and goal lane should be = " << v.goal_lane << endl;
	  int end_lanes_from_goal = abs(v.goal_lane - end[0]);
    double dt = double(future_traj.size()-1);
    int proposed_lane = start[0];
    //cout << " Delta time = " << dt << " end[1] = " << end[1] << " vehicle.s " << before_move[1] << endl;
    double avg_speed = double(end[1] - before_move[1]) / dt;

    vector<int> accels;
    int closest_approach = std::numeric_limits<int>::max();
    int dist;
    bool collides = false;
    int collision_time = future_traj.size();
    // In filtered we have only trajectories of other vehicle that are in
    // the same lane as our vehicle...so we will use them
    // check time after time if we collide.
    filter_prediction_by_lane(predictions, proposed_lane, filtered);
    // calculate some data...
    vector<vector<int>>::const_iterator cit;
    vector<int>::const_iterator v_cit;
    //int prev_s = future_traj[0][1];
    //int prev_a = future_traj[0][3];
    for (cit = future_traj.begin()+1; cit != future_traj.end(); cit++) {
        //int tmp_l = cit->at(0);
        int tmp_s = cit->at(1);
        int tmp_v = cit->at(2);
        int tmp_a = cit->at(3);
        accels.push_back(tmp_a);
        // Check collision between two contigous future states...
        map<int, vector<vector<int> >>::const_iterator m_cit;
        for(m_cit = filtered.begin(); m_cit != filtered.end(); ++m_cit) {
          //cout << "checking collision with car id " << m_cit->first << endl;
          const vector<vector<int>>& others = m_cit->second;
          for(int t = 1; t < future_traj.size(); t++) {
            int other_prev_s = others[t-1][1];
            //cout << "considering time prevision = " << t << endl;
            //cout << "others[" << t << "][" << 1 << "] = " << others[t][1] << endl;
            // We have at index 1 the s position of other vehicles in the same lane
            if((others[t-1][1] < tmp_s && others[t][1] >= tmp_s) ||
              (others[t-1][1] > tmp_s && others[t][1] <= tmp_s) ||
              (others[t-1][1] == tmp_s && (others[t][1] - others[t-1][1] <= tmp_v))) {
                //cout << "Merdissima we have a collision at time " << t << endl;
                collides = true;
                if(t < collision_time)
                  collision_time = t;
            }
            dist = abs(others[t][1] - tmp_s);
            if (dist < closest_approach)
              closest_approach = dist;
//          }
          }
        }
    }
    int max_accel = *max_element(accels.begin(), accels.end(), acc_cmp);
    vector<int> rms_accels;
    for(int i = 0; i < (int) accels.size(); i++)
      rms_accels.push_back(pow(accels[i],2));
	  int num_accels = rms_accels.size();
	  double rms_acceleration = double(std::accumulate(rms_accels.begin(), rms_accels.end(), 0)) / num_accels;

    struct cost_data c;

    c.proposed_lane = proposed_lane;
    c.avg_speed = avg_speed != 0 ? avg_speed : std::numeric_limits<double>::epsilon();
    //c.avg_speed = avg_speed;
    c.max_accel = max_accel;
    c.rms_acceleration = rms_acceleration;
    c.closest_approach = closest_approach;
    c.end_distance_to_goal = end_distance_to_goal;
    c.end_lanes_from_goal = end_lanes_from_goal;
    c.collides = collides;
    c.collision_time = collision_time;

    cost += out_of_the_road_cost(v, c);
    cost += change_lane_cost(v, c);
    cost += distance_from_goal_lane(v, c);
    cost += inefficiency_cost(v, c);
    cost += collision_cost(v, c);
    cost += buffer_cost(v, c);
    return cost;
}

void BehaviourPlanner::get_next_state(Vehicle& veh, map<int, vector< vector<int> > >& predictions, int horizon=1)
{
    double cost;
    double min_cost = std::numeric_limits<double>::max();
    vector<string> states = valid_next_states(state);
    for(int i = 0; i < states.size(); i++) {
        cost = 0;
        // create copy of our vehicle
        Vehicle experiment = veh;
        experiment.state = states[i];
        //cout << "Evaluating state: " << states[i] <<  endl;
        experiment.realize_state(predictions);
        // Move the vehicle in the future....
        vector<vector<int> > future = experiment.generate_predictions(horizon);
        // Finally get a cost...
        cost = calculate_total_cost(experiment, future, predictions);
        //cout << "\tTotal cost for state " << experiment.state << " = " << cost << " curren min cost " << min_cost << " for state " << this->state << endl;
        if(cost < min_cost) {
          min_cost = cost;
          //cout << "\tNEW min cost is " << min_cost << " for state " << experiment.state << endl;
          this->state = experiment.state;
        }
    }
    cout << "NEXT STATE WILL BE " << this->state << endl << endl;
}

#endif
