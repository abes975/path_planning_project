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
  public:
    BehaviourPlanner(Style::DriveStyle style);
};

#endif
