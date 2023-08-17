#pragma once 
#include <vector>
#include "robot.h"

namespace InverseKinematics{
    struct IK_statistics{
        unsigned long long number_of_collision_check = 0;
        double time_of_collision_check = 0;
    };
    IK_statistics sample_all_goals(std::vector<Robot> &answers, Robot pos, const GoalPoint &goal, const std::vector<Polygon>& obstacles,const u_int8_t sample_rate);
    void sample_all_goals_parallel(std::vector<Robot> &answers, Robot pos, const GoalPoint &goal, const std::vector<Polygon>& obstacles,const u_int8_t sample_rate);

}
