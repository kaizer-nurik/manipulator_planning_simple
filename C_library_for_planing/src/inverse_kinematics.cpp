#include "inverse_kinematics.h"
#include "robot.h"
#include <vector>
#include <deque>
#include <utility>
#include <assert.h>
#include <thread>
#include <mutex>
#include <math.h>
#include <chrono>
// из-за числовой нестабильности чисел с плавающей запятой
// Возможны случаи, когда косинус равен не 1, а 1.000001 и т. д.
#define COS_TOLERANCE 1

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
        Vector2D curr_vector(current_joint_length * std::cos(current_sample.curr_angle * M_PI / 180.0), current_joint_length * std::sin(current_sample.curr_angle * M_PI / 180.0));
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
            angle_2 = angle_2 * 180.0 / M_PI; // в угол
            angle_1 = angle_1 * 180.0 / M_PI; // в угол
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
            current_sample.last.x += current_sample.pos.joints[depth].length * std::cos(current_sample.curr_angle * M_PI / 180.0);
            current_sample.last.y += current_sample.pos.joints[depth].length * std::sin(current_sample.curr_angle * M_PI / 180.0);

            second_answer.pos.configuration[depth] = fix_fmod(result.first.second);
            second_answer.curr_angle += second_answer.pos.configuration[depth];
            second_answer.last.x += second_answer.pos.joints[depth].length * std::cos(second_answer.curr_angle * M_PI / 180.0);
            second_answer.last.y += second_answer.pos.joints[depth].length * std::sin(second_answer.curr_angle * M_PI / 180.0);

            layer.push_back(current_sample);
            layer.push_back(second_answer);
        }
    };

    InverseKinematics::IK_statistics get_solutions(const std::deque<Robot_and_layer_info> &layer, std::vector<Robot> &answers, const GoalPoint &goal, const std::vector<Polygon> &obstacles)
    {
        InverseKinematics::IK_statistics stats;
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

    void calculate_curr_angle_parallel(Robot_and_layer_info current_sample, const int &depth, const GoalPoint &goal, double remaining_length, const double length_coef, std::deque<Robot_and_layer_info> &layer, std::mutex &queue_mutex)
    {
        remaining_length *= length_coef;
        std::pair<std::pair<double, double>, bool> result = get_angle(current_sample, depth, goal, remaining_length);

        if (result.second)
        {
            current_sample.pos.configuration[depth] = fix_fmod(result.first.first);
            Robot_and_layer_info second_answer = current_sample;

            current_sample.curr_angle += current_sample.pos.configuration[depth];
            current_sample.last.x += current_sample.pos.joints[depth].length * std::cos(current_sample.curr_angle * M_PI / 180.0);
            current_sample.last.y += current_sample.pos.joints[depth].length * std::sin(current_sample.curr_angle * M_PI / 180.0);

            second_answer.pos.configuration[depth] = fix_fmod(result.first.second);
            second_answer.curr_angle += second_answer.pos.configuration[depth];
            second_answer.last.x += second_answer.pos.joints[depth].length * std::cos(second_answer.curr_angle * M_PI / 180.0);
            second_answer.last.y += second_answer.pos.joints[depth].length * std::sin(second_answer.curr_angle * M_PI / 180.0);

            std::lock_guard<std::mutex> lock(queue_mutex);

            layer.push_back(current_sample);
            layer.push_back(second_answer);
        }
    };
    void sample_by_length_parallel(std::deque<Robot_and_layer_info> &layer, std::mutex &queue_mutex, const int layer_start_index, const int layer_end_index, Robot sample, const uint8_t sample_rate, int depth, GoalPoint goal, double remaining_length)
    {
        if (sample_rate == 1)
        {
            for (int layer_index = layer_start_index; layer_index < layer_end_index; layer_index++)
            {
                queue_mutex.lock();
                Robot_and_layer_info current_sample = layer[0];
                layer.pop_front();
                queue_mutex.unlock();

                calculate_curr_angle_parallel(current_sample, depth, goal, remaining_length, 1, layer, queue_mutex);
            }
        }
        else
        {

            for (int layer_index = layer_start_index; layer_index < layer_end_index; layer_index++)
            {
                queue_mutex.lock();
                Robot_and_layer_info current_sample = layer[0];
                layer.pop_front();
                queue_mutex.unlock();

                double current_joint_length = current_sample.pos.joints[depth].length;
                Vector2D curr_vector(current_joint_length * std::cos(current_sample.curr_angle * M_PI / 180.0), current_joint_length * std::sin(current_sample.curr_angle * M_PI / 180.0));
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
                    calculate_curr_angle_parallel(current_sample, depth, goal, remaining_length, length_coef, layer, queue_mutex);
                }
            }
        }
    }

    void get_solutions_parralel(const std::deque<Robot_and_layer_info> &layer, std::vector<Robot> &answers, const GoalPoint goal, const std::vector<Polygon> obstacles, std::mutex &answers_mutex, std::mutex &queue_mutex, const int layer_start_index, const int layer_end_index)
    {
        for (int layer_index = layer_start_index; layer_index < layer_end_index; layer_index++)
        {
            queue_mutex.lock();
            Robot_and_layer_info current_sample = layer[layer_index];
            queue_mutex.unlock();
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
            if (collide(current_sample.pos, current_sample.pos.configuration, obstacles))
            {
                continue;
            }
            // Сохраняем
            answers_mutex.lock();
            answers.push_back(current_sample.pos);
            answers_mutex.unlock();
        }
    }

    GoalPoint change_goal_to_last_joint(const Robot &pos, const GoalPoint &goal)
    {
        GoalPoint intermediate_goal(goal);

        double last_angle_rad = goal.angle1_ * M_PI / 180.0;
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
            Vector2D curr_vector(current_joint_length * std::cos(current_sample.curr_angle * M_PI / 180.0), current_joint_length * std::sin(current_sample.curr_angle * M_PI / 180.0));
            Vector2D vector_to_goal(goal.goalpoint.x - current_sample.last.x, goal.goalpoint.y - current_sample.last.y);
            double dist_to_goal = vector_to_goal.length();

            double from = (dist_to_goal - current_joint_length- goal.delta) / remaining_length;
            from = (from < 0) ? 0 : from;
            from = (from > 1) ? 1 : from;
            double to = (dist_to_goal + current_joint_length+goal.delta) / remaining_length;
            to = (to < 0) ? 0 : to;
            to = (to > 1) ? 1 : to;
            double step = (to - from) / sample_rate;
            step = (from + step * (sample_rate + 1) > to) ? step : 1; // случай, когда from = to и step = 0
            for (double length_coef = from; length_coef <= to+step/2.0; length_coef += step)
            {
                calculate_curr_angle(current_sample, depth, goal, remaining_length, length_coef, layer);
            }
        }
    }
}

InverseKinematics::IK_statistics InverseKinematics::sample_all_goals(std::vector<Robot> &answers, Robot pos, const GoalPoint &goal, const std::vector<Polygon> &obstacles, const u_int8_t sample_rate)
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

void InverseKinematics::sample_all_goals_parallel(std::vector<Robot> &answers, Robot pos, const GoalPoint &goal, const std::vector<Polygon> &obstacles, const uint8_t sample_rate)
{
    /*
    Идём по-слойно, начиная с 1 звена, и заканчивая последним. при обработке каждого звена набираем "слой", то есть множество
    вариантов при различных коэффициентах длин. И при обработке звена будем проходится по слою и добавлять новый.
    */
    int processor_count = std::thread::hardware_concurrency(); // количество доступных ядер процессора
    std::cout << "processors count:" << processor_count << std::endl;
    if (processor_count == 0)
    {

        processor_count = 4; // Если вернул 0, то это значит он не смог определить
    }
    std::vector<std::thread> threads;
    std::mutex queue_mutex;
    std::mutex answers_mutex;
    threads.reserve(processor_count);

    Robot sample = pos;
    GoalPoint intermediate_goal = change_goal_to_last_joint(sample, goal);                  // получаем промежуточную цель в виде начала 6 звена
    double remaining_joint_length = get_remaining_joint_length(sample, 2, sample.dof_ - 1); // получаем оставшуюся длину от 2 звена до предпоследнего
    int depth_global = 0;
    std::deque<Robot_and_layer_info> layer;
    layer.emplace_back(sample,Vector2D(0,0),0);
    int layer_size = layer.size();
    // проходимся по всем звеньям, кроме последнего (так как у него фиксированная длина)
    for (int depth = 0; depth < sample.dof_ - 3; depth++)
    {
        for (int thread_count = 0; thread_count < processor_count; thread_count++)
        {
            int layer_start_index = (layer_size * thread_count) / processor_count;
            int layer_end_index = ((thread_count + 1) != processor_count) ? ((layer_size * (thread_count + 1)) / processor_count) : layer_size;

            threads.emplace_back(std::thread(sample_by_length_parallel, std::ref(layer), std::ref(queue_mutex), layer_start_index, layer_end_index, sample, sample_rate, depth, intermediate_goal, remaining_joint_length));
        }
        for (int thread_count = 0; thread_count < processor_count; thread_count++)
        {
            threads[thread_count].join();
        }
        threads.clear();
        depth_global++;
        remaining_joint_length -= sample.joints[depth + 1].length;
        layer_size = layer.size();
    }

    if (sample.dof_ > 2)
    {
        layer_size = layer.size();
        // Cлучай, когда у нас отсаётся одно звено. Тогда не надо менять коэффициент длины
        for (int thread_count = 0; thread_count < processor_count; thread_count++)
        {
            int layer_start_index = (layer_size * thread_count) / processor_count;
            int layer_end_index = ((thread_count + 1) != processor_count) ? ((layer_size * (thread_count + 1)) / processor_count) : layer_size;

            threads.emplace_back(std::thread(sample_by_length_parallel, std::ref(layer), std::ref(queue_mutex), layer_start_index, layer_end_index, sample, 1, depth_global, intermediate_goal, sample.joints[sample.dof_ - 2].length));
        }
        for (int thread_count = 0; thread_count < processor_count; thread_count++)
        {
            threads[thread_count].join();
        }
        threads.clear();

        depth_global++;
    }

    if (sample.dof_ > 1)
    {
        layer_size = layer.size();
        // Cлучай, когда у нас отсаётся одно звено. Тогда не надо менять коэффициент длины
        for (int thread_count = 0; thread_count < processor_count; thread_count++)
        {
            int layer_start_index = (layer_size * thread_count) / processor_count;
            int layer_end_index = ((thread_count + 1) != processor_count) ? ((layer_size * (thread_count + 1)) / processor_count) : layer_size;

            threads.emplace_back(std::thread(sample_by_length_parallel, std::ref(layer), std::ref(queue_mutex), layer_start_index, layer_end_index, sample, 1, depth_global, intermediate_goal, 0));
        }
        for (int thread_count = 0; thread_count < processor_count; thread_count++)
        {
            threads[thread_count].join();
        }
        threads.clear();

        depth_global++;
    }

    layer_size = layer.size();
    // теперь в layers остались лишь решения ОКЗ. Надо выделить те, которые соответствуют нашим требованиям

    for (int thread_count = 0; thread_count < processor_count; thread_count++)
    {
        int layer_start_index = (layer_size * thread_count) / processor_count;
        int layer_end_index = ((thread_count + 1) != processor_count) ? ((layer_size * (thread_count + 1)) / processor_count) : layer_size;

        threads.emplace_back(std::thread(get_solutions_parralel, std::ref(layer), std::ref(answers), goal, obstacles, std::ref(answers_mutex), std::ref(queue_mutex), layer_start_index, layer_end_index));
    }
    for (int thread_count = 0; thread_count < processor_count; thread_count++)
    {
        threads[thread_count].join();
    }
    threads.clear();

}
