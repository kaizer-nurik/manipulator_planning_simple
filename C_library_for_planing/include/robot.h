#pragma once

#include <vector>
#include <cmath>
#include "obstacles.h"
#include <iostream>
#include "geometry.h"
#include <deque>
#include <utility>
#include <assert.h>
#include <thread>
#include <mutex>
#include <math.h>
#include <chrono>

    double fix_fmod(double angle){
    double angle_from_0_to_360 = angle - (2*M_PI) * floor( angle / (2*M_PI) );

    if (angle_from_0_to_360 > M_PI){
        angle_from_0_to_360 -= (2*M_PI);
    }  
    return angle_from_0_to_360;

}

struct Joint 
{
double length;
double width;
std::vector<double> limits;
};

class Robot
{
public:
    Robot() : dof_(0) {}

    // Robot(const Robot& other) 
    // {
    //     for (Joint a:other.joints)
    //     {
    //         this->AddJoint(a.length, a.width, a.limits);            
    //     }
    //     this->configuration = other.configuration;
    // }

    Robot(const Robot &other)
    {
    for (Joint a : other.joints)
    {
        this->AddJoint(a.length, a.width, a.limits);
    }
    this->dof_ = other.dof_;
    this->configuration = other.configuration;
    }

    void AddJoint(double length, double width, const std::vector<double>& limits) {
        Joint joint;
        joint.length = length;
        joint.width = width;
        joint.limits = limits;
        joints.push_back(joint);
        dof_++;
    }

    void printJointDetails() const {
        std::cout << "Robot DOF: " << dof_ << std::endl;

        for (const auto& joint : joints)
         {
            std::cout << "Joint length: " << joint.length << std::endl;
            std::cout << "Joint width: " << joint.width << std::endl;
            std::cout << "Joint limits: ";
            
            for (const auto& limit : joint.limits) {
                std::cout << limit << " ";
            }

            std::cout << std::endl;
        }
    }
    std::vector<Joint> get_joints() const {return joints;};
    std::vector<Joint> joints;
    std::vector<double> configuration;
    double dof_;
    


};
Vector2D end_effector(const Robot &robot, const std::vector<double> &config)
{
    Vector2D ef(0.0, 0.0);
    double angle = 0.0;
    for (auto i=0u; i<robot.dof_; i++)
    {
        double pi = std::acos(-1);
        angle+=config[i];
        ef.x += robot.joints[i].length * cos(angle);
        ef.y += robot.joints[i].length * sin(angle);
    }
    return ef;
}


struct GoalPoint
{
    Vector2D goalpoint;
    double angle1_;
    double angle2_; //tolerance
    double delta = 1;

    GoalPoint(double x, double y, double angle1, double angle2): goalpoint(x, y), angle1_(angle1), angle2_(angle2){}

    bool is_goal(const double& x, const double& y, const double& angle) const
    {
     return ((
        std::sqrt(((x - goalpoint.x) * (x - goalpoint.x))+((y - goalpoint.y) * (y - goalpoint.y))) < delta)
        && (std::abs(fix_fmod(angle-angle1_)) < angle2_) );};
       
    bool is_goal(const Robot& pos)const{
    Vector2D q = end_effector(pos, pos.configuration);
    double angle = 0.0;
    for (auto i = 0u; i < pos.dof_; i++)
    {
        angle += pos.configuration[i];
    }
    return is_goal(q.x, q.y, angle);
    }
};

bool collide(const Robot &robot, const std::vector<double> angles, const std::vector<Polygon> &poligons) //true - collide, false - not collide
{
    bool flag = false;
    double x0 = 0.0;
    double y0 = 0.0;
    double angle = 0.0;
    std::vector<Polygon> joints;
    for (int i=0; i<robot.dof_; i++)
    {
        //std::cout << x0 << ' ' << y0 << ' ' << robot.dof_ << ' ';
        angle += angles[i];
        double x1 = x0 - sin(angle)*robot.joints[i].width/2;
        double y1 = y0 + cos(angle)*robot.joints[i].width/2;
        double x2 = x0 - sin(angle)*robot.joints[i].width/2 + cos(angle)*robot.joints[i].length;
        double y2 = y0 + cos(angle)*robot.joints[i].width/2 + sin(angle)*robot.joints[i].length;
        double x3 = x0 + sin(angle)*robot.joints[i].width/2 + cos(angle)*robot.joints[i].length;
        double y3 = y0 - cos(angle)*robot.joints[i].width/2 + sin(angle)*robot.joints[i].length;
        double x4 = x0 + sin(angle)*robot.joints[i].width/2;
        double y4 = y0 - cos(angle)*robot.joints[i].width/2;
        joints.push_back(Polygon({Vector2D(x1, y1), Vector2D(x2, y2), Vector2D(x3, y3), Vector2D(x4, y4)}));
        x0+=cos(angle)*robot.joints[i].length;
        y0+=sin(angle)*robot.joints[i].length;
        
    }
    //std::cout << joints.size() << ' ' << poligons.size() << std::endl;

    for (int i=0; i<joints.size(); i++)
    {
        for (int j=0; j<poligons.size(); j++)
        {
            //std::cout << 1 << ' ' << (polygons_collide(joints[i], poligons[j])) << ' ';
            if ((polygons_collide(joints[i], poligons[j]))) 
            {
                //std::cout << "collision occured\n";
                return true;
            }
        }
    }
    //std::cout << std::endl;
    for (int i=0; i<joints.size(); i++)
    {
        for (int j=0; j<joints.size(); j++)
        {
            if (std::abs(j-i)<2) continue;
            if (polygons_collide(joints[i], joints[j])) return true;
        }
    }
    return flag;
}







#define COS_TOLERANCE 1

    struct IK_statistics{
        unsigned long long number_of_collision_check = 0;
        double time_of_collision_check = 0;
    };
namespace
{

    
    struct Robot_and_layer_info
    {
        Robot pos;
        double curr_angle = 0;
        Vector2D last = Vector2D(0.0, 0.0);
        Robot_and_layer_info(Robot p, Robot_and_layer_info other) : pos(p), curr_angle(other.curr_angle), last(other.last){};
        Robot_and_layer_info(Robot p, Vector2D other_last, double other_angle) : pos(p), curr_angle(other_angle), last(other_last){};
    };

    double acos_with_tolerance(const double &value, const double &tolerance)
    {
        double angle = 0;
        if (std::abs(value) < 1){
            angle = std::acos(value);
        }
        else// случай, когда выражение под косинусом больше по модулю, чем 1
        {

            if (value > 0)
            {
                angle = 0; // cos = 1
            }
            else
            {
                angle = M_PI; // cos = -1
            }
        }
        return angle;
    }
    double get_remaining_joint_length(const Robot &pos, const int &start_joint, const int &end_joint)
    {
        if (start_joint == pos.dof_ - 1)
        {
            return 0;
        }

        double remaining_joint_length = 0;

        for (int joint_count = start_joint - 1; joint_count < end_joint; joint_count++)
        {
            remaining_joint_length += pos.joints[joint_count].length;
        }

        return remaining_joint_length;
    }

    // если нашелся угол, то bool = true
    std::pair<std::pair<double, double>, bool> get_angle(Robot_and_layer_info current_sample, const int &depth, const GoalPoint &goal, double remaining_length)
    {

        double current_joint_length = current_sample.pos.joints[depth].length;
        Vector2D curr_vector(current_joint_length * std::cos(current_sample.curr_angle), current_joint_length * std::sin(current_sample.curr_angle ));
        Vector2D vector_to_goal(goal.goalpoint.x - current_sample.last.x, goal.goalpoint.y - current_sample.last.y);
        double dist_to_goal = vector_to_goal.length();

        if ((goal.delta < dist_to_goal - (current_joint_length + remaining_length)) ||       // Условие, когда цель дальше, чем область достижения манипулятора
            ( goal.delta < remaining_length - (dist_to_goal + current_joint_length) )) // Условие, когда цель слижком близко и манипулятор не может так дотянутся
        {
            return std::pair<std::pair<double, double>, bool>(std::pair<double, double>(0, 0), false);
        }
        double angle_2 = 0;
        double angle_1 = 0;
        if (dist_to_goal != 0) // защита от деления на 0.
        {
            double triangle_angle_cos = -(remaining_length * remaining_length - current_joint_length * current_joint_length - dist_to_goal * dist_to_goal) / (2 * dist_to_goal * current_joint_length);
            angle_1 = acos_with_tolerance(triangle_angle_cos, COS_TOLERANCE);
            double full_angle_cos = (curr_vector.dotProduct(vector_to_goal) / (dist_to_goal * current_joint_length));
            angle_2 = acos_with_tolerance(full_angle_cos, COS_TOLERANCE);
            

            double orientation = curr_vector.x * vector_to_goal.y - curr_vector.y * vector_to_goal.x; // определение направления вращения по ориентации оси вращения вектора Z(вниз или вверх)
            if (orientation < 0)                                                                      // если вектора коллинеарны, то angle_2 = 0, а orientation = 0, деление на 0
            {
                angle_2 *= -1;
            }

            // std::cout<<"depth= "<<depth<<"cos1= "<< triangle_angle_cos<< "angle1= " << angle_1 <<"cos2= "<< full_angle_cos << "angle_2= " << angle_2 << "orientation=" << orientation<<std::endl;

        }
        // проверим угол на соответсвие лимитов
        // if ((angle_2 < current_sample.joints[depth].limits[0]) || (angle_2 > current_sample.joints[depth].limits[1]))
        // {
        //     return std::pair<std::pair<double,double>, bool>(std::pair<double,double>(0,0), false);
        // }
        // std::cout<<"----------------"<<std::endl;
        // std::cout<<"depth = " << depth<<std::endl;
        // for (double angle:current_sample.configuration){
        // std::cout<<angle<<std::endl;
        // }
        // std::cout<<"answers "<< angle_2+angle_1<<" "<< angle_2-angle_1 <<std::endl;
        // std::cout<<"----------------"<<std::endl;

        return std::pair<std::pair<double, double>, bool>(std::pair<double, double>(angle_2 + angle_1, angle_2 - angle_1), true);
    };

    void calculate_curr_angle(Robot_and_layer_info current_sample, const int &depth, const GoalPoint &goal, double remaining_length, const double length_coef, std::deque<Robot_and_layer_info> &layer)
    {
        remaining_length *= length_coef;
        std::pair<std::pair<double, double>, bool> result = get_angle(current_sample, depth, goal, remaining_length);

        if (result.second)
        {
            current_sample.pos.configuration[depth] = fix_fmod(result.first.first);
            Robot_and_layer_info second_answer = current_sample;

            current_sample.curr_angle += current_sample.pos.configuration[depth];
            current_sample.last.x += current_sample.pos.joints[depth].length * std::cos(current_sample.curr_angle );
            current_sample.last.y += current_sample.pos.joints[depth].length * std::sin(current_sample.curr_angle );

            second_answer.pos.configuration[depth] = fix_fmod(result.first.second);
            second_answer.curr_angle += second_answer.pos.configuration[depth];
            second_answer.last.x += second_answer.pos.joints[depth].length * std::cos(second_answer.curr_angle );
            second_answer.last.y += second_answer.pos.joints[depth].length * std::sin(second_answer.curr_angle );

            layer.push_back(current_sample);
            layer.push_back(second_answer);
        }
    };

    IK_statistics get_solutions(const std::deque<Robot_and_layer_info> &layer, std::vector<Robot> &answers, const GoalPoint &goal, const std::vector<Polygon> &obstacles)
    {
        IK_statistics stats;
        for (Robot_and_layer_info current_sample : layer)
        {
            std::pair<std::pair<double, double>, bool> result = get_angle(current_sample, current_sample.pos.dof_ - 1, goal, 0);
            if (!result.second)
            {
                continue;
            }
            // Находим угол последнего звена
            current_sample.pos.configuration[current_sample.pos.dof_ - 1] = fix_fmod(result.first.first);
            // проверяем, дошли ли,
            if (!goal.is_goal(current_sample.pos))
            {
                continue;
            }
            // проверяем на коллизию
            stats.number_of_collision_check++;
            auto t1 = std::chrono::high_resolution_clock::now();
            bool collided = collide(current_sample.pos, current_sample.pos.configuration, obstacles);
            auto t2 = std::chrono::high_resolution_clock::now();
            stats.time_of_collision_check += (t2-t1).count();
            if (collided)
            {
                continue;
            }
            // Сохраняем

            answers.push_back(current_sample.pos);
            
        }
        return stats;
    }


   

    GoalPoint change_goal_to_last_joint(const Robot &pos, const GoalPoint &goal)
    {
        GoalPoint intermediate_goal(goal);

        double last_angle_rad = goal.angle1_ ;
        double last_joint_length = pos.joints[pos.dof_ - 1].length;
        intermediate_goal.goalpoint.x -= std::cos(last_angle_rad) * last_joint_length;
        intermediate_goal.goalpoint.y -= std::sin(last_angle_rad) * last_joint_length;
        return intermediate_goal;
    }

    void sample_by_length(Robot_and_layer_info current_sample, const int &depth, const GoalPoint &goal, double remaining_length, std::deque<Robot_and_layer_info> &layer, const u_int8_t sample_rate)
    {

        if (sample_rate == 1)
        {
            calculate_curr_angle(current_sample, depth, goal, remaining_length, 1, layer);
        }
        else
        {

            double current_joint_length = current_sample.pos.joints[depth].length;
            Vector2D curr_vector(current_joint_length * std::cos(current_sample.curr_angle ), current_joint_length * std::sin(current_sample.curr_angle ));
            Vector2D vector_to_goal(goal.goalpoint.x - current_sample.last.x, goal.goalpoint.y - current_sample.last.y);
            double dist_to_goal = vector_to_goal.length();

            double from = (dist_to_goal - current_joint_length) / remaining_length;
            from = (from < 0) ? 0 : from;
            from = (from > 1) ? 1 : from;
            double to = (dist_to_goal + current_joint_length) / remaining_length;
            to = (to < 0) ? 0 : to;
            to = (to > 1) ? 1 : to;
            double step = (to - from) / sample_rate;
            step = (from + step * (sample_rate + 1) > to) ? step : 1; // случай, когда from = to и step = 0
            for (double length_coef = from; length_coef <= to; length_coef += step)
            {
                calculate_curr_angle(current_sample, depth, goal, remaining_length, length_coef, layer);
            }
        }
    }
}

IK_statistics sample_all_goals(std::vector<Robot> &answers, Robot pos, const GoalPoint &goal, const std::vector<Polygon> &obstacles, const u_int8_t sample_rate)
{
    /*
    Идём по-слойно, начиная с 1 звена, и заканчивая последним. при обработке каждого звена набираем "слой", то есть множество
    вариантов при различных коэффициентах длин. И при обработке звена будем проходится по слою и добавлять новый.
    Для получения более точных результатов сначала получим промежуточную цель, в виде начала крепления 6 звена. TODO: семплировать эту цель исходя из условий точности угла
    */
    Robot sample = pos;
    GoalPoint intermediate_goal = change_goal_to_last_joint(sample, goal);                  // получаем промежуточную цель в виде начала 6 звена
    double remaining_joint_length = get_remaining_joint_length(sample, 2, sample.dof_ - 1); // получаем оставшуюся длину от 2 звена до предпоследнего
    int depth_global = 0;
    std::deque<Robot_and_layer_info> layer;
    layer.emplace_back(sample, Vector2D(0,0),0);
    int layer_size = layer.size();
    // проходимся по всем звеньям, кроме предпоследнего (так как у него фиксированная длина)
    for (int depth = 0; depth < sample.dof_ - 3; depth++)
    {
        for (int layer_index = 0; layer_index < layer_size; layer_index++)
        {
            Robot_and_layer_info current_sample = layer[0];
            sample_by_length(current_sample, depth, intermediate_goal, remaining_joint_length, layer, sample_rate);
            layer.pop_front();
        }
        remaining_joint_length -= sample.joints[depth + 1].length;
        layer_size = layer.size();
        depth_global++;
    }

    if (sample.dof_ > 2)
    {
        layer_size = layer.size();
        // Cлучай, когда у нас отсаётся одно звено. Тогда не надо менять коэффициент длины
        for (int layer_index = 0; layer_index < layer_size; layer_index++)
        {
            Robot_and_layer_info current_sample = layer[0];
            calculate_curr_angle(layer[0], depth_global, intermediate_goal, sample.joints[sample.dof_ - 2].length, 1, layer);

            layer.pop_front();
        }
        depth_global++;
    }

    if (sample.dof_ > 1)
    {
        layer_size = layer.size();
        // Cлучай, когда у нас отсаётся одно звено. Тогда не надо менять коэффициент длины
        for (int layer_index = 0; layer_index < layer_size; layer_index++)
        {
            Robot_and_layer_info current_sample = layer[0];

            calculate_curr_angle(current_sample, depth_global, intermediate_goal, 0, 1, layer);

            layer.pop_front();
        }
    }

    // теперь в layers остались лишь решения ОКЗ. Надо выделить те, которые соответствуют нашим требованиям
    

    return get_solutions(layer, answers, goal, obstacles);
}

