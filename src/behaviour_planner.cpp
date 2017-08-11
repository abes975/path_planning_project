#include "behaviour_planner.hpp"

#include <iostream>


BehaviourPlanner::BehaviourPlanner(Style::DriveStyle style)
{
  _style = style;
  _num_lanes = lanes;
}

bool BehaviourPlanner::is_too_close()
{

}

bool BehaviourPlanner::is_adjacent_lane_safe(int lane)
{

}

void BehaviourPlanner next_action(double ego_pos, vector<vector<double> sensor_fusion)
{

}

int BehaviourPlanner::get_lane() const
{
    return _lane;
}

int BehaviourPlanner::get_speed() const
{
    return _speed;
}
