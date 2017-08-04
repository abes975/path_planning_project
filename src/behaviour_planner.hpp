#ifndef __BEHAVIOUR_PLANNER_HPP__
#define __BEHAVIOUR_PLANNER_HPP__

class BehaviourPlanner
{
    private:
        int _curr_state;
        double _cost;
        int _get_valid_next_state();
        
    public:
        final enum States { "KEEP_LANE", "PREPARE_CHANGE_LANE_LEFT", "PREPARE_CHANGE_LANE_RIGHT",
                "CHANGE_LANE_LEFT", "CHANGE_LANE_RIGHT"};
        //double evaluate_state();
};

#endif
