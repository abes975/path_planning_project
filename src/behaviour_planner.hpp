#ifndef __BEHAVIOUR_PLANNER_HPP__
#define __BEHAVIOUR_PLANNER_HPP__

#include <vector>

using namespace std;

class BehaviourPlanner
{
    public:
        enum State { BEGIN, CONSTANT_SPEED, KEEP_LANE, PREPARE_CHANGE_LANE_LEFT, PREPARE_CHANGE_LANE_RIGHT, CHANGE_LANE_LEFT, CHANGE_LANE_RIGHT};
        //void calculate_next_state(Vehichle& v, prediction, horizon);
        vector<BehaviourPlanner::State> get_valid_next_states(BehaviourPlanner::State state);
    private:
        int _curr_state;
        double _cost;


};


#endif
