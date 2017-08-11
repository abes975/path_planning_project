#ifndef __BEHAVIOUR_PLANNER_HPP__
#define __BEHAVIOUR_PLANNER_HPP__

namespace Style
{
    enum DriveStyle { CALM, SPORTY, GRANNY };
};


using namespace std;

class BehaviourPlanner
{
  private:
    Style::DriveStyle _style;
    double _speed;
    int _lane;
    bool is_too_close();
    bool is_adjacent_lane_safe(int lane);

  public:
    BehaviourPlanner(Style::DriveStyle style);
    int get_lane() const;
    double get_speed() const;
    void next_action(double ego_pos, vector<vector<double> sensor_fusion);
};

#endif
