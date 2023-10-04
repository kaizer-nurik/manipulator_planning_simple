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
  char result[ PATH_MAX ];
  ssize_t count = readlink( "/proc/self/exe", result, PATH_MAX );
  return std::string( result, (count > 0) ? count : 0 );
}

int main(int argc, const char *argv[])
{
    const float angle_step = std::stof(argv[1]);// Первый параметр это угловой шаг
    const float goal_bias = std::stof(argv[2]);// Второй параметр - величина goalbias, от 0 до 1
    const long max_iteration = std::stol(argv[3]); // Третий параметр - макс число итераций алгоритма роста

    std::cout << "Текущая директория:"<<std::endl;
    std::cout << getexepath()<<std::endl;

    std::string path = argv[4];// Четвертый параметр - кодирует сцену,
    //1 - Первая сцена, ручные тесты
    //1r - Первая сцена, случайные тесты
    //2 - Вторая сцена, ручные тесты
    //2r - Вторая сцена, случайные тесты
    //3 - Третья сцена, ручные тесты
    //3r - Третья сцена, случайные тесты

    std::string line = "./dataset/scene" + path + "/";
    std::string line2 = "";
    if (path.substr(1, 1) == "r")
    {
        line2 = "scene_" + path.substr(0, 1) + "_random_test_";
    }
    else
    {
        line2 = "scene_" + path.substr(0, 1) + "_test_";
    }
    int max_tests = (path.substr(1, 1) == "r")?100:50;
    for (int k = 1; k <= 100; k++)
    {
        std::string result_dir = "./results_of_scene" + path + "#" + std::to_string(k) + "/";
        std::string xml_trajectories_dir = result_dir + "scene" + path + "/";
        std::filesystem::create_directory(result_dir);
        std::filesystem::create_directory(xml_trajectories_dir);

        int successes = 0;
        for (int i = 1; i <= max_tests; i++)
        {
            std::cout << line + line2 + std::to_string(i) + ".xml" << std::endl;
            std::vector<Polygon> polygons;
            Robot start = Robot();
            GoalPoint goal(0.0, 0.0, 0.0, 0.0);

            bool read_normally = read_scene(line + line2 + std::to_string(i) + ".xml", polygons, start, goal);
            std::cout << i << " " << read_normally << std::endl;
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
            if (TIMEOUT_MS > duration/1ms)
            {
                for (int i = 0; i < 1000000; i++)
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
                FileSaver::save_csv<double>(result_dir + line2 + std::to_string(i) + "_trajectory.csv", planner.get_path(), planner.get_dof());

                // Функция, Записывающая csv файл в xml под <csv></csv>
                FileSaver::save_csv_to_xml<double>(xml_trajectories_dir + line2 + std::to_string(i) + "_trajectory.xml", line + line2 + std::to_string(i) + ".xml", planner.get_path(), planner.get_dof());
            }

            // Функция, сохраняющая дерево из rrt для визуализации в питоне.
            FileSaver::save_rrt_tree(result_dir + line2 + std::to_string(i) + "rrt_tree.csv", planner.get_tree());
            successes += reached_goal;
            // bool b = planner.coll_test(start, polygons);
            std::cout << reached_goal << std::endl;
            std::cout << "duration: " << duration / 1ns << " ns" << std::endl;
            stats["reached_goal"] = std::to_string(reached_goal);
            stats["timeout"] = std::to_string(timeout);
            stats["total_duration"] = std::to_string(duration / 1ns);
            stats["angle_step"] = std::to_string(angle_step);
            stats["goal bias"] = std::to_string(goal_bias);

            FileSaver::write_map_to_json(result_dir + line2 + std::to_string(i) + "_stats.json", stats);
            FileSaver::write_end_config_to_csv(result_dir + line2 + std::to_string(i) + "_IK_res.csv", planner.get_end_configurations());
        }

        std::cout << "tests completed " << successes << '/' << 50 << std::endl;
    }

    return EXIT_SUCCESS;
}
