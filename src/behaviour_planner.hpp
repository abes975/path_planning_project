#ifndef __BEHAVIOUR_PLANNER_HPP__
#define __BEHAVIOUR_PLANNER_HPP__

#include <vector>


namespace Style
{
    enum DriveStyle { NORMAL, SPORTY, MAX_CONFORT };
};


using namespace std;

class BehaviourPlanner
{
  private:
    Style::DriveStyle _style;
    double _speed;
    double _speed_limit;
    int  _num_lanes;
    int  _lane_width;
    int _lane;
    double _cur_buffer_front;
    double _cur_buffer_rear;
    double _time_frame;

    bool is_too_close();
    bool is_same_lane(double other_d_coord, int lane);
    bool can_change_lane(double ego_pos, int closest_car_idx,
        const vector<vector<double> >& sensor_fusion, int new_lane, int path_len);
    const double get_minimum_buffer_front() const;
    const double get_minimum_buffer_rear() const;
    int convert_d_to_lane(const double d);

  public:
    BehaviourPlanner(Style::DriveStyle style, int cur_lane, int num_lanes,
      double lane_width, double max_speed, double time_frame);
    int lane() const;
    double speed() const;
    double time_frame() const;
    void next_action(double ego_pos, const vector<vector<double> >& sensor_fusion,
      double path_len);
};

#endif
