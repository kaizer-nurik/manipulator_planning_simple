#include <chrono>
#include <iostream>
#include <fstream>
#include <string>
#include <memory>
#include <random>
#include <limits.h>
#include <unistd.h>

#include "rapidxml.hpp"
#include "rapidxml_print.hpp"
#include "obstacles.h"
#include "robot.h"
#include "parsing.h"
#include "planner.h"
#include "benchmark.h"
#include <vector>
#include <map>

using json = nlohmann::json;

int main(int argc, const char *argv[])
{

    std::string path = argv[1]; // Первый параметр - путь к файлу

    std::cout << path + std::to_string(1) + ".xml" << std::endl;
    int successes = 0;

    std::vector<Polygon> polygons;
    Robot start = Robot();
    GoalPoint goal(0.0, 0.0, 0.0, 0.0);

    bool read_normally = read_scene(path, polygons, start, goal); // argv[1]
    std::cout << read_normally << std::endl;
    if (!read_normally)
    {
        return EXIT_FAILURE;
    }

    Planner_A_star Planner_A_star(path, "result_trajectory.xml");
    std::map<std::string, double> dict;
    bool b = Planner_A_star.AStar(start, goal, polygons, dict);
    writeDataToJson(0, dict["g_units"], dict["coord_tolerance"], dict["angle_tolerance"], dict["time"], dict["coll_check_percentage"], dict["opened_nodes"],
                    dict["closed_nodes"], dict["g_cost"], dict["turn_numbers"], "scene1_0_1_data.json");

    ////bool b = Planner_A_star.coll_test(start, polygons);
    return EXIT_SUCCESS;
}
