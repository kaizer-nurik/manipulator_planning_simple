#include <chrono>
#include <iostream>
#include <fstream>
#include <string>
#include <memory>
#include "rapidxml.hpp"
#include "rapidxml_print.hpp"
#include "obstacles.h"
#include "robot.h"
#include "parsing.h"
#include "planner.h"
#include "geometry.h"
#include "benchmark.h"
#include <vector>


int main(int argc, const char * argv[])
 {
    
    // Timer<std::chrono::seconds> t("test");
    // std::vector<Polygon> polygons;
    // Robot start = Robot();
    // GoalPoint goal(0.0, 0.0, 0.0, 0.0);

    // bool read_normally = read_scene("example.xml", polygons, start, goal); //argv[1]
    // std::cout << read_normally << std::endl;
    // if (! read_normally)
    // {
    //     return EXIT_FAILURE;
    // }
    // double angle=0.0;
    // for (int i=0u; i<start.dof_; i++)
    //     angle+=start.configuration[i];
    // std::cout << "angle:" << angle/6.28 << ' ' << goal.angle2_ << std::endl;
    // std::cout << goal.goalpoint.x << ' ' << goal.goalpoint.y << std::endl;
    // Planner planner("example.xml", "trajectory.xml");
    // bool b = planner.AStar(start, goal, polygons);
    // //bool b = planner.coll_test(start, polygons);
    // std::cout << b << std::endl;


    std::vector<Polygon> polygons;
    Robot start = Robot();
    GoalPoint goal(0.0, 0.0, 0.0, 0.0);

    bool read_normally = read_scene("example.xml", polygons, start, goal); //argv[1]
    std::cout << read_normally << std::endl;
    if (! read_normally)
    {
        return EXIT_FAILURE;
    }
    double angle=0.0;
    for (int i=0u; i<start.dof_; i++)
        angle+=start.configuration[i];
    std::cout << "angle:" << angle/6.28 << ' ' << goal.angle2_ << std::endl;
    std::cout << goal.goalpoint.x << ' ' << goal.goalpoint.y << std::endl;
    Planner planner("example.xml", "trajectory.xml");
    bool b = planner.AStar(start, goal, polygons);
    //bool b = planner.coll_test(start, polygons);
    std::cout << b << std::endl;

    
    // std::string line = "./dataset/scene2/";  //"./dataset/scene1/scene_1_test_1.xml"
    // std::string line2 =  "scene_2_test_";
    // int successes = 0;
    // for (int i=1; i<=50; i++)
    // {        
    //     std::vector<Polygon> polygons;
    //     Robot start = Robot();
    //     GoalPoint goal(0.0, 0.0, 0.0, 0.0);

    //     bool read_normally = read_scene(line+line2 + std::to_string(i) + ".xml", polygons, start, goal); //argv[1]
    //     std::cout << read_normally << std::endl;
    //     if (! read_normally)
    //     {
    //         return EXIT_FAILURE;
    //     }
    //     double angle=0.0;
    //     for (int i=0u; i<start.dof_; i++)
    //         angle+=start.configuration[i];
    //     std::cout << "angle:" << angle/6.28 << ' ' << goal.angle2_ << std::endl;
    //     std::cout << goal.goalpoint.x << ' ' << goal.goalpoint.y << std::endl;
    //     Planner planner(line + line2 + std::to_string(i) + ".xml", line2 + std::to_string(i) +"_trajectory.xml");
    //     bool b = planner.AStar(start, goal, polygons);
    //     successes+=b;
    //     //bool b = planner.coll_test(start, polygons);
    //     std::cout << b << std::endl;
    // }


    // std::cout << "tests completed " << successes << '/' << 50 << std::endl;
    //std::string filename = argv[1];
    //run_benchmark("example.xml", "trajectory.xml", 100, start, polygons, goal);

    //std::cout << "polygons_collide: " << polygons_collide(Polygon({Vector2D(0, 0), Vector2D(1, 1), Vector2D(0, 2), Vector2D(-1, 1)}), 
    //Polygon({Vector2D(0, 0), Vector2D(1, 1), Vector2D(0, 2), Vector2D(-1, 1)})) << std::endl;
    return EXIT_SUCCESS;
}
