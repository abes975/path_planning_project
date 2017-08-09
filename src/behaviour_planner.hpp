#ifndef __BEHAVIOUR_PLANNER_HPP__
#define __BEHAVIOUR_PLANNER_HPP__

#include <vector>
#include <unordered_map>
#include "vehicle.hpp"

using namespace std;

struct enum_hash
{
    template <typename T>
    inline
    typename std::enable_if<std::is_enum<T>::value, std::size_t>::type
    operator ()(T const value) const
    {
        return static_cast<std::size_t>(value);
    }
};

class BehaviourPlanner
{
    public:
        enum State { BEGIN, CONSTANT_SPEED, KEEP_LANE, PREPARE_CHANGE_LANE_LEFT, PREPARE_CHANGE_LANE_RIGHT, CHANGE_LANE_LEFT, CHANGE_LANE_RIGHT};
        //void calculate_next_state(Vehichle& v, prediction, horizon);
        BehaviourPlanner();
        //const vector<BehaviourPlanner::State>& get_valid_next_states(BehaviourPlanner::State state);
        void plan_next_move(Vehicle& ego, unordered_map<int, Vehicle>& other_cars, int horizon);
    private:
        int _curr_state;
        double _cost;
        unordered_map<State, vector<State>, enum_hash> _allowed_states;
        const vector<BehaviourPlanner::State>& get_valid_next_states(BehaviourPlanner::State current_state);
        vector<vector<double>> generate_future_positions(unordered_map<int, Vehicle>& other_cars, int horizon);
};


#endif
