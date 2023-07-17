#pragma once
#include "obstacles.h"
#include "planner.h"
#include "parsing.h"
#include "robot.h"
#include <chrono>
#include <typeinfo>
#include <random>
#include <vector>

class Timer 
{
private:
    std::chrono::time_point<std::chrono::steady_clock> start_time;
    std::chrono::duration<double> paused_duration;
    bool is_running;
    std::string name;

public:
    Timer(const std::string &name_) : name(name_), is_running(false)
    {
        start();
    }

    ~Timer() 
    {
        pause();
        std::cout << "Timer " << name << " stopped.Elapsed time : " << getElapsedTime() << " seconds" << std::endl;
    }

    void start()
    {
        if (!is_running) 
        {
            start_time = std::chrono::steady_clock::now();
            paused_duration = std::chrono::duration<double>::zero();
            is_running = true;
            //std::cout << "Started timer " << name << std::endl;
        }
    }

    void pause() 
    {
        if (is_running) 
        {
            auto end_time = std::chrono::steady_clock::now();
            paused_duration += end_time - start_time;
            is_running = false;
        }
    }

    double getElapsedTime() 
    {
        if (is_running)
        {
            auto current_time = std::chrono::steady_clock::now();
            return std::chrono::duration_cast<std::chrono::duration<double>>(current_time - start_time - paused_duration).count();
        }
        else 
        {
            return std::chrono::duration_cast<std::chrono::duration<double>>(paused_duration).count();
        }
    }
};

// bool run_benchmark(std::string input_file_name, std::string output_file_name, int N_tests, const Robot&robot, const std::vector<Polygon> &polygons, const GoalPoint &goal1)
// {
//     std::random_device rd;
// 	std::mt19937 engine{5};
// 	std::uniform_real_distribution<double> dist{ 0.0, 1.0 };
//     double PI = std::acos(-1);
//     int success = 0;
// 	auto goal = goal1;
//     if (robot.dof_ == 1)
//     {
//         goal.angle2_ = 2*PI;
//         for (int test=0; test<N_tests; test++)
//         {	
// 			double alpha = dist(engine) * 2 * PI;
//             double x = cos(alpha) * robot.joints[0].length;
//             double y = sin(alpha) * robot.joints[0].length;
//             goal.goalpoint.x = x;
//             goal.goalpoint.y = y;

//             Planner planner(input_file_name, output_file_name);
//             bool b = planner.AStar(robot, goal, polygons);
//             success+=b;
//             if (b) 
//             {
//                 std::cout << "test_"<< test << "successed, angle was " << std::acos(x)*180/PI << ' ' << x << ' ' << y <<  std::endl;
//             }
//             else  
//             {
//                 std::cout << "test_"<< test << "failed, angle was " << std::acos(x)*180/PI << ' ' << x << ' ' << y <<  std::endl;
//             }
//         }
//         std::cout << "testing completed: " << success << '/' << N_tests << " successes" << std::endl;
//     }
// 	else 
// 	{
// 		double total_length = 0.0;
// 		for(int i=0; i<robot.dof_; i++)
// 		{
// 			total_length+=robot.joints[i].length;
// 		}
// 		for (int test=0; test<N_tests; test++)
// 		{
// 			double alpha = dist(engine) * 2 * PI;
// 			double radius = dist(engine) * total_length;
//             double x = cos(alpha) * total_length;
//             double y = sin(alpha) * total_length;
//             goal.goalpoint.x = x;
//             goal.goalpoint.y = y;
// 			Planner planner(input_file_name, output_file_name);
//             bool b = planner.AStar(robot, goal, polygons);
//             success+=b;
//             if (b) 
//             {
//                 std::cout << "test_"<< test << "successed, angle was " << std::acos(x)*180/PI << ' ' << x << ' ' << y <<  std::endl;
//             }
//             else  
//             {
//                 std::cout << "test_"<< test << "failed, angle was " << std::acos(x)*180/PI << ' ' << x << ' ' << y <<  std::endl;
//             }
//         }
//         std::cout << "testing completed: " << success << '/' << N_tests << " successes" << std::endl;
// 		}
//     return true;

// }