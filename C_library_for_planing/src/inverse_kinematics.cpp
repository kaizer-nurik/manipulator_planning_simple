#include "inverse_kinematics.h"
#include "robot.h"
#include <vector>
#include <deque>
#include <utility>
#include <assert.h>
// из-за числовой нестабильности чисел с плавающей запятой
// Возможны случаи, когда косинус равен не 1, а 1.000001 и т. д.
#define COS_TOLERANCE 100

namespace
{
    double acos_with_tolerance(const double &value, const double &tolerance)
    {
        double angle = std::acos(value);
        if (std::isnan(angle)) // случай, когда выражение под косинусом больше по модулю, чем 1
        {
            if ((std::abs(value) - 1) > tolerance) // когда выходим за точность
            {
                assert(false);
            }
            if (value > 0)
            {
                angle = 0; // cos = 1
            }
            else
            {
                angle = 180; // cos = -1
            }
        }
        return angle;
    }
    double get_remaining_joint_length(const Robot &pos, const int &start_joint)
    {
        if (start_joint == pos.dof_ - 1)
        {
            return 0;
        }

        double remaining_joint_length = 0;

        for (int joint_count = start_joint + 1; joint_count < pos.dof_; joint_count++)
        {
            remaining_joint_length += pos.joints[joint_count].length;
        }

        return remaining_joint_length;
    }

    // если нашелся угол, то bool = true
    std::pair<double, bool> get_angle(Robot current_sample, const int &depth, const GoalPoint &goal, double remaining_length)
    {
        double curr_angle = 0;
        Vector2D last(0, 0);
        for (int joint_count = 0; joint_count < depth; joint_count++)
        {
            curr_angle += current_sample.configuration[joint_count];
            last.x += current_sample.joints[joint_count].length * std::cos(curr_angle * 3.1415 / 180);
            last.y += current_sample.joints[joint_count].length * std::sin(curr_angle * 3.1415 / 180);
        }

        double current_joint_length = current_sample.joints[depth].length;
        Vector2D curr_vector(current_joint_length * std::cos(curr_angle * 3.1415 / 180), current_joint_length * std::sin(curr_angle * 3.1415 / 180));
        Vector2D vector_to_goal(goal.goalpoint.x - last.x, goal.goalpoint.y - last.y);
        double dist_to_goal = vector_to_goal.length();

        if ((goal.delta < dist_to_goal - (current_joint_length + remaining_length)) || // Условие, когда цель дальше, чем область достижения манипулятора
            (dist_to_goal < std::abs(current_joint_length - remaining_length)))        // Условие, когда цель слижком близко и манипулятор не может так дотянутся
        {
            return std::pair<double, bool>(0, false);
        }
        double angle_2 = 0;
        if (dist_to_goal != 0) // защита от деления на 0.
        {
            double triangle_angle_cos = -(remaining_length * remaining_length - current_joint_length * current_joint_length - dist_to_goal * dist_to_goal) / (2 * dist_to_goal * current_joint_length);
            double angle_1 = acos_with_tolerance(triangle_angle_cos, COS_TOLERANCE);

            double full_angle_cos = curr_vector.dotProduct(vector_to_goal) / dist_to_goal;
            angle_2 = acos_with_tolerance(full_angle_cos, COS_TOLERANCE) - angle_1;

            int orientation = curr_vector.x * vector_to_goal.y - curr_vector.y * vector_to_goal.x; // определение направления вращения по ориентации оси вращения вектора Z(вниз или вверх)
            if (orientation)                                                                       // если вектора коллинеарны, то angle_2 = 0, а orientation = 0, деление на 0
            {
                orientation /= std::abs(orientation);
                angle_2 *= orientation;
            }

            angle_2 = angle_2 * 180 / 3.1415; // в угол
        }
        // проверим угол на соответсвие лимитов
        if ((angle_2 < current_sample.joints[depth].limits[0]) || (angle_2 > current_sample.joints[depth].limits[1]))
        {
            return std::pair<double, bool>(0, false);
        }
        return std::pair<double, bool>(angle_2, true);
    };

    void calculate_curr_angle(Robot current_sample, const int &depth, const GoalPoint &goal, double remaining_length, const double length_coef, std::deque<Robot> &layer)
    {
        remaining_length *= length_coef;
        std::pair<double,bool> result = get_angle(current_sample, depth, goal, remaining_length);

        if (result.second)
        {
            current_sample.configuration[depth] = result.first;
            layer.push_back(current_sample);

            current_sample.configuration[depth] = fix_fmod(-result.first+ 180);
            layer.push_back(current_sample);

        }
    };

    void get_solutions(const std::deque<Robot> &layer,std::vector<Robot> &answers, const GoalPoint &goal, const std::vector<Polygon>&  obstacles)
    {
        for (Robot current_sample : layer)
        {
            std::pair<double,bool> result = get_angle(current_sample, current_sample.dof_ - 1, goal, 0);
            if (!result.second)
            {
                continue;
            }
            // Находим угол последнего звена
            current_sample.configuration[current_sample.dof_ - 1] = result.first;
            // проверяем, дошли ли,
            if (!goal.is_goal(current_sample))
            {
                continue;
            }
            // проверяем на коллизию
            if (collide(current_sample, current_sample.configuration, obstacles))
            {
                continue;
            }
            // Сохраняем

            answers.push_back(current_sample);
        }
    }

}

std::vector<Robot> InverseKinematics::sample_all_goals(std::vector<Robot> &answers, Robot pos, const GoalPoint &goal, const std::vector<Polygon>& obstacles)
{
    /*
    Идём по-слойно, начиная с 1 звена, и заканчивая последним. при обработке каждого звена набираем "слой", то есть множество
    вариантов при различных коэффициентах длин. И при обработке звена будем проходится по слою и добавлять новый.
    */
    Robot sample = pos;
    double remaining_joint_length = get_remaining_joint_length(sample, 0);

    std::deque<Robot> layer;
    layer.push_back(sample);
    int layer_size = 1;
    // проходимся по всем звеньям, кроме последнего (так как у него фиксированная длина)
    for (int depth = 0; depth < sample.dof_ - 2; depth++)
    {
        for (int layer_index = 0; layer_index < layer_size; layer_index++)
        {
            Robot current_sample = layer[0];
            for (double length_coef = 4; length_coef <= 10; length_coef++)
            {
                calculate_curr_angle(current_sample, depth, goal, remaining_joint_length, length_coef / 10.0, layer);
            }
            layer.pop_front();
        }
        remaining_joint_length -= sample.joints[depth+1].length;
        layer_size = layer.size();
    }

    // слой при последнем звене по сути есть наши решения, нужно лишь просчитать угол последнего звена и посмотреть, достигается ли цель
    for (int layer_index = 0; layer_index < layer_size; layer_index++)
    {
        Robot current_sample = layer[0];

        calculate_curr_angle(current_sample, sample.dof_ - 2, goal, sample.joints[sample.dof_ - 1].length, 1, layer);
        
        layer.pop_front();
    }

    // теперь в layers остались лишь решения ОКЗ. Надо выделить те, которые соответствуют нашим требованиям
    get_solutions(layer, answers, goal,obstacles);

    return answers;
}
