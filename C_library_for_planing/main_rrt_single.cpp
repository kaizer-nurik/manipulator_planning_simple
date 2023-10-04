#include <chrono>
#include <iostream>
#include <fstream>
#include <string>
#include <limits.h>
#include <unistd.h>
#include <memory>
#include "rapidxml.hpp"
#include "rapidxml_print.hpp"
#include "obstacles.h"
#include "robot.h"
#include "parsing.h"
#include "planner.h"
#include "benchmark.h"
#include <vector>
#include <map>
#include "file_saver.h"
#include <chrono>
#include <ctime>
#include <filesystem>
#define TIMEOUT_MS 100000

std::string getexepath()
{
    char result[PATH_MAX];
    ssize_t count = readlink("/proc/self/exe", result, PATH_MAX);
    return std::string(result, (count > 0) ? count : 0);
}

int main(int argc, const char *argv[])
{
    const float angle_step = std::stof(argv[1]);// Первый параметр это угловой шаг
    const float goal_bias = std::stof(argv[2]);// Второй параметр - величина goalbias, от 0 до 1
    const long max_iteration = std::stol(argv[3]); // Третий параметр - макс число итераций алгоритма роста
    std::string path = argv[4];// Четвертый параметр - путь к файлу

    std::cout << path << std::endl;
    std::vector<Polygon> polygons;
    Robot start = Robot();
    GoalPoint goal(0.0, 0.0, 0.0, 0.0);

    bool read_normally = read_scene(path, polygons, start, goal);
    if (!read_normally)
    {
        return EXIT_FAILURE;
    }
    double angle = 0.0;

    std::cout << "angle:" << goal.angle1_ << ' ' << goal.angle2_ << std::endl;
    std::cout << goal.goalpoint.x << ' ' << goal.goalpoint.y << std::endl;
    std::map<std::string, std::string> stats;
    bool reached_goal = false;
    bool timeout = false;
    auto t1 = std::chrono::high_resolution_clock::now();
    auto t2 = std::chrono::high_resolution_clock::now();
    auto duration = t2 - t1;
    RRT planner(angle_step,goal_bias, start.dof_, start, goal, polygons, stats);
    if (TIMEOUT_MS > duration / 1ms)
    {
        for (int i = 0; i < max_iteration; i++)
        {
            planner.grow_tree();
            // std::cout<<i<<planner.is_finished()<<std::endl;
            if (planner.is_finished())
            {
                reached_goal = true;
                break;
            }
            t2 = std::chrono::high_resolution_clock::now();
            duration = t2 - t1;
        }
    }
    planner.export_stats();
    if (reached_goal)
    {
        FileSaver::save_csv<double>("result_trajectory.csv", planner.get_path(), planner.get_dof());

        // Функция, Записывающая csv файл в xml под <csv></csv>
        FileSaver::save_csv_to_xml<double>("result_trajectory.xml", path, planner.get_path(), planner.get_dof());
    }

    // Функция, сохраняющая дерево из rrt для визуализации в питоне.
    FileSaver::save_rrt_tree("rrt_tree.csv", planner.get_tree());
    // bool b = planner.coll_test(start, polygons);
    std::cout << reached_goal << std::endl;
    std::cout << "duration: " << duration / 1ns << " ns" << std::endl;
    stats["reached_goal"] = std::to_string(reached_goal);
    stats["timeout"] = std::to_string(timeout);
    stats["total_duration"] = std::to_string(duration / 1ns);
    stats["angle_step"] = std::to_string(angle_step);
    stats["goal bias"] = std::to_string(goal_bias);

    FileSaver::write_map_to_json("results_stats.json", stats);
    FileSaver::write_end_config_to_csv("results_IK_res.csv", planner.get_end_configurations());

    return EXIT_SUCCESS;
}
