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
    Timer t("FULL TIMER");
    const float angle_step = std::stof(argv[1]);// Первый параметр это угловой шаг
    const float goal_bias = std::stof(argv[2]);// Второй параметр - величина goalbias, от 0 до 1
    const long max_iteration = std::stol(argv[3]); // Третий параметр - макс число итераций алгоритма роста

    std::cout << "Текущая директория:"<<std::endl;
    std::cout << getexepath()<<std::endl;

    std::string path = argv[4];// Четвертый параметр - кодирует сцену,

    std::string line = "../../DATASET/SCENE_Fabric/8dof/";
    std::string line2 = "test_";

    int max_tests = 100;
    {
        std::string result_dir = "./results_of_scene8/";// + std::to_string(k) + "/";
        std::string xml_trajectories_dir = result_dir + "scene" + "/";
        std::filesystem::create_directory(result_dir);
        std::filesystem::create_directory(xml_trajectories_dir);

        int successes = 0;
        for (int i = 91; i <= 100; i++)
        {   
            int successes1 = 0;
            double number_of_nodes = 0;
            double number_of_goal_expanding_nodes = 0;
            double number_of_random_nodes = 0;
            double number_of_denied_nodes_random = 0;
            double number_of_denied_nodes_goal = 0;
            double number_of_IK_results = 0;
            double time_of_IK_results = 0;
            double time_of_collision_check_in_IK = 0;
            double number_of_collision_check_in_IK = 0;
            double time_of_collision_check = 0;
            double number_of_collision_check = 0;
            double time_of_nn_check = 0;
            double number_of_nn_check = 0;
            double  total_duration = 0;

            std::map<std::string, std::string> stats;
            bool reached_goal = false;
            bool timeout = false;
            int number_of_repeats = 20;
            for (int k = 1; k <= number_of_repeats; k++)
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
                
                //auto t1 = std::chrono::high_resolution_clock::now();
                //auto t2 = std::chrono::high_resolution_clock::now();
                //auto duration = t2 - t1;
                Timer t1("rrt");
                RRT planner(angle_step,goal_bias, start.dof_, start, goal, polygons, stats);
                {
                    for (int ii = 0; ii < 1000000; ii++)
                    {   
                        // if (t1.getElapsedTime() > 200.0) 
                        // {   
                        //     k--;
                        //     break;
                        // }
                        planner.grow_tree();
                        // std::cout<<i<<planner.is_finished()<<std::endl;
                        if (planner.is_finished())
                        {
                            reached_goal = true;
                            break;
                        }
                    }
                }
                
                planner.export_stats();
                
                //try
                {
                //std::cout << "OPENED NODES " << stats["opened_nodes"] << "nnn " << stats["number_of_goal_expanding_nodes"] << ' ' << stats["time_of_collision_check"]<< std::endl;
                number_of_nodes +=                 std::stod(stats["opened_nodes"]);
                number_of_goal_expanding_nodes +=  std::stod(stats["number_of_goal_expanding_nodes"]);
                number_of_random_nodes +=          std::stod(stats["number_of_random_nodes"]);
                number_of_denied_nodes_random +=   std::stod(stats["number_of_denied_nodes_random"]);
                number_of_denied_nodes_goal +=     std::stod(stats["number_of_denied_nodes_goal"]);
                number_of_IK_results +=            std::stod(stats["number_of_IK_results"]);
                time_of_IK_results +=              std::stod(stats["time_of_IK_results"]);
                time_of_collision_check_in_IK +=   std::stod(stats["time_of_collision_check_in_IK"]);
                number_of_collision_check_in_IK += std::stod(stats["number_of_collision_check_in_IK"]);
                time_of_collision_check +=         std::stod(stats["time_of_collision_check"]);
                number_of_collision_check +=       std::stod(stats["number_of_collision_check"]);
                time_of_nn_check +=                std::stod(stats["time_of_nn_check"]);
                number_of_nn_check +=              std::stod(stats["number_of_nn_check"]);
                total_duration   +=                t1.getElapsedTime() ;
                }
                // catch (const std::invalid_argument& e) {
                // std::cerr << "Invalid argument: " << e.what() << std::endl;
                //     } catch (const std::out_of_range& e) {
                // std::cerr << "Out of range: " << e.what() << std::endl;
                //     }
                if (reached_goal)
                {
                    FileSaver::save_csv<double>(result_dir + line2 + std::to_string(i) + "_trajectory.csv", planner.get_path(), planner.get_dof());

                    // Функция, Записывающая csv файл в xml под <csv></csv>
                    FileSaver::save_csv_to_xml<double>(xml_trajectories_dir + line2 + std::to_string(i) + "_trajectory.xml", line + line2 + std::to_string(i) + ".xml", planner.get_path(), planner.get_dof());
                }

                successes1 += reached_goal;
                std::cout << reached_goal << std::endl;
                std::cout << "duration: " << t1.getElapsedTime() << " s" << std::endl;


            }
            std::cout << total_duration << std::endl;
            if (successes1>0) successes+=1;

            number_of_nodes                    /= successes1;
            number_of_goal_expanding_nodes     /= successes1;
            number_of_random_nodes             /= successes1;
            number_of_denied_nodes_random      /= successes1;
            number_of_denied_nodes_goal        /= successes1;
            number_of_IK_results               /= successes1;
            time_of_IK_results                 /= successes1;
            time_of_collision_check_in_IK      /= successes1;
            number_of_collision_check_in_IK    /= successes1;
            time_of_collision_check            /= successes1;
            number_of_collision_check          /= successes1;
            time_of_nn_check                   /= successes1;
            number_of_nn_check                 /= successes1;
            total_duration                     /= successes1;

            stats["opened_nodes"] = std::to_string(number_of_nodes);
            stats["number_of_goal_expanding_nodes"] = std::to_string(number_of_goal_expanding_nodes);
            stats["number_of_random_nodes"] = std::to_string(number_of_random_nodes);
            stats["number_of_denied_nodes_random"] = std::to_string(number_of_denied_nodes_random);
            stats["number_of_denied_nodes_goal"] = std::to_string(number_of_denied_nodes_goal);
            stats["number_of_IK_results"] = std::to_string(number_of_IK_results);
            stats["time_of_IK_results"] = std::to_string(time_of_IK_results);
            stats["time_of_collision_check_in_IK"] = std::to_string(time_of_collision_check_in_IK);
            stats["number_of_collision_check_in_IK"] = std::to_string(number_of_collision_check_in_IK);
            stats["time_of_collision_check"] = std::to_string(time_of_collision_check);
            stats["number_of_collision_check"] = std::to_string(number_of_collision_check);
            stats["time_of_nn_check"] = std::to_string(time_of_nn_check);
            stats["number_of_nn_check"] = std::to_string(number_of_nn_check);
            stats["reached_goal"]   = std::to_string(reached_goal);
            stats["timeout"]       = std::to_string(timeout);
            stats["total_duration"] = std::to_string(total_duration);
            stats["angle_step"] = std::to_string(angle_step);
            stats["goal bias"] = std::to_string(goal_bias);
            stats["_number"] = std::to_string(i);
            FileSaver::write_map_to_json(result_dir + line2 + "_stats.json", stats);
            //FileSaver::write_end_config_to_csv(result_dir + line2 + std::to_string(i) + "_IK_res.csv", planner.get_end_configurations());
        }

        std::cout << "tests completed " << successes << '/' << 100 << std::endl;
    }

    return EXIT_SUCCESS;
}
