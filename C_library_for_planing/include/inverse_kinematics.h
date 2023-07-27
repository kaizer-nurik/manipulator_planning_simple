#pragma once 
#include <vector>
#include "robot.h"

namespace InverseKinematics{
    std::vector<Robot> sample_all_goals(std::vector<Robot> &answers, Robot pos, const GoalPoint &goal, const std::vector<Polygon>& obstacles,const u_int8_t sample_rate);
    std::vector<Robot> sample_all_goals_parallel(std::vector<Robot> &answers, Robot pos, const GoalPoint &goal, const std::vector<Polygon>& obstacles,const u_int8_t sample_rate);

}
