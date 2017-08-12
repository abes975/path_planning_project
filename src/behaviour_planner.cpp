#include "behaviour_planner.hpp"

#include <iostream>
#include <math.h>
#include <limits>


BehaviourPlanner::BehaviourPlanner(Style::DriveStyle style, int cur_lane, int num_lanes,
  double lane_width, double max_speed, double time_frame)
{
  _style = style;
  _num_lanes = num_lanes;
  _lane_width = lane_width;
  _lane = cur_lane;
  _speed_limit = max_speed;
  _time_frame = time_frame;
}


bool BehaviourPlanner::is_same_lane(double other_d_coord, int lane)
{
  double right_limit =  _lane_width * lane + _lane_width;
  //double right_limit =  _lane_width/2 + _lane_width * lane + _lane_width/2;
  //double left_limit = _lane_width/2 + _lane_width * lane - _lane_width/2;
  double left_limit = _lane_width * lane;
  // is this car in my lane...we don't want to bump into it
  //cout << "ma come cazzo " << other_d_coord  << " right_limit = " << right_limit << " left_limit = " << left_limit << endl;
  if (other_d_coord < right_limit && other_d_coord > left_limit)
    return true;
  return false;
}

int BehaviourPlanner::convert_d_to_lane(const double d)
{
  int lane = -1;
  if (d > 0.0 && d < _lane_width) {
    lane = 0;
  } else if (d >= _lane_width && d < 2 * _lane_width) {
    lane = 1;
  } else if (d >= 2 * _lane_width && d < 3 * _lane_width) {
    lane = 2;
  }
  return lane;
}

bool BehaviourPlanner::can_change_lane(double ego_pos, int closest_car_idx,
    const vector<vector<double> >& sensor_fusion, int new_lane, int path_len)
{
  double other_car_s;
  bool new_lane_free = true;
  bool change_lane = false;
  _cur_buffer_front = std::numeric_limits<double>::max();
  _cur_buffer_rear = std::numeric_limits<double>::max();
  double buffer_front;
  double buffer_rear;
  for(int j = 0; j < sensor_fusion.size(); j++) {
    buffer_front = std::numeric_limits<double>::max();
    buffer_rear = std::numeric_limits<double>::max();
    if(j == closest_car_idx)
      continue;
    double vx = sensor_fusion[j][3];
    double vy = sensor_fusion[j][4];
    double other_speed = sqrt(pow(vx, 2) + pow(vy, 2));
    double other_car_s_init = sensor_fusion[j][5];
    double other_car_s = other_car_s_init;
    // let's see where this car will be at the end of the path...
    // s = s + v * t;
    other_car_s += (double) path_len * _time_frame * other_speed;
    double other_car_d = sensor_fusion[j][6];

    if(is_same_lane(other_car_d, new_lane)) {
      new_lane_free = false;

      if(other_car_s > ego_pos)
        buffer_front = other_car_s - ego_pos;
      else
        buffer_rear = ego_pos - other_car_s;

      if(buffer_front <  get_minimum_buffer_front())
        return false;
      else if(buffer_rear < get_minimum_buffer_rear())
        return false;

      if(buffer_front < _cur_buffer_front)
        _cur_buffer_front = buffer_front;
      if(buffer_rear < _cur_buffer_rear)
        _cur_buffer_rear = buffer_rear;

      change_lane = true;
    }
  }
  if (new_lane_free || !new_lane_free && change_lane)
    return true;

  return false;
}


const double BehaviourPlanner::get_minimum_buffer_front() const
{
  switch(_style) {
    case Style::NORMAL:
      return 35.0;
      break;
    case Style::SPORTY:
      return 20.0;
      break;
    case Style::MAX_CONFORT:
      return 45.0;
      break;
    default:
      return 35.0;
  }
}

const double BehaviourPlanner::get_minimum_buffer_rear() const
{
  switch(_style) {
    case Style::NORMAL:
      return 15.0;
      break;
    case Style::SPORTY:
      return 7.0;
      break;
    case Style::MAX_CONFORT:
      return 20.0;
      break;
    default:
      return 15.0;
  }
}


void BehaviourPlanner::next_action(double ego_pos,
  const vector<vector<double> >& sensor_fusion, double path_len)
{
  bool almost_crash = false;
  int new_lane = _lane;
  for(int i = 0; i < sensor_fusion.size(); i++) {
    // Want to know which lane are other cars...
    double other_car_d = sensor_fusion[i][6];
    // is this car in my lane...we don't want to bump into it
    if (is_same_lane(other_car_d, _lane)) {
      double vx = sensor_fusion[i][3];
      double vy = sensor_fusion[i][4];
      // Get some data
      double other_speed = sqrt(pow(vx, 2) + pow(vy, 2));
      double other_car_s = sensor_fusion[i][5];
      // let's see where this car will be at the end of the path...
      // s = s + v * t;
      other_car_s += (double) path_len * _time_frame * other_speed;
      // We don't want to get too close to other cars....
      if ((other_car_s > ego_pos) && ((other_car_s - ego_pos) < get_minimum_buffer_front())) {
        almost_crash = true;
        // See if we can go left or right or both (we will choose the lane with
        // the biggest gap..of course if we are in leftmost lane we can just go right
        // and if we are rightmost we can just go left...
        // leftmost
        // WE MUST CHECK OTHER CARS!!!!!! NOT THIS ONE!!
        if(_lane == 0) {
          if(can_change_lane(ego_pos, i, sensor_fusion, _lane + 1, path_len)) {
            new_lane = _lane + 1;
            almost_crash = false;
          }
        } else if (_lane == _num_lanes - 1) {
          if(can_change_lane(ego_pos, i, sensor_fusion, _lane - 1, path_len)) {
            new_lane = _lane - 1;
            almost_crash = false;
          }
        } else {
          double buf_left_front = 0;
          double buf_left_rear = 0;
          double buf_right_front = 0;
          double buf_right_rear = 0;
          if(can_change_lane(ego_pos, i, sensor_fusion, _lane - 1, path_len)) {
            buf_left_front = _cur_buffer_front;
            buf_left_rear = _cur_buffer_rear;
            almost_crash = false;
          }
          if(can_change_lane(ego_pos, i, sensor_fusion, _lane + 1, path_len)) {
            buf_right_front = _cur_buffer_front;
            buf_right_rear = _cur_buffer_rear;
            almost_crash = false;
          }
          if(buf_left_front == buf_right_front) {
            if(buf_left_rear > buf_right_rear) {
              new_lane = _lane - 1;
            } else {
              new_lane = _lane + 1;
            }
          } else if(buf_right_front > buf_left_front) {
            new_lane = _lane + 1;
          }
          else {
            new_lane = _lane - 1;
          }
        }
      }
    }
  }

  double speed_correction = 0;
  if(new_lane != _lane && !almost_crash) {
    _lane = new_lane;
    speed_correction = -0.30;
  }

  if (almost_crash)
    _speed -= .40;
  else if (_speed < _speed_limit)
    _speed += .65 + speed_correction;

}

int BehaviourPlanner::lane() const
{
    return _lane;
}

double BehaviourPlanner::speed() const
{
    return _speed;
}

double BehaviourPlanner::time_frame() const
{
  return _time_frame;
}
