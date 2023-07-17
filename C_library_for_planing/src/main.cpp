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
#include <map>


int main(int argc, const char * argv[])
 {
    
    // Timer t("test");
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

    
    std::string line = "./dataset/scene1/";  //"./dataset/scene1/scene_1_test_1.xml"
   std::string line2 =  "scene_1_test_";
    int successes = 0;
    for (int i=30; i<=50; i++)
    {        
        std::vector<Polygon> polygons;
        Robot start = Robot();
        GoalPoint goal(0.0, 0.0, 0.0, 0.0);

        bool read_normally = read_scene(line+line2 + std::to_string(i) + ".xml", polygons, start, goal); //argv[1]
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
        Planner planner(line + line2 + std::to_string(i) + ".xml", line2 + std::to_string(i) +"_trajectory.xml");
        std::map<std::string, double> dict;
        bool b = planner.AStar(start, goal, polygons, dict);
        writeDataToJson(i, dict["g_units"], dict["coord_tolerance"], dict["angle_tolerance"], dict["time"], dict["coll_check_percentage"], dict["opened_nodes"],
        dict["closed_nodes"], dict["g_cost"], dict["turn_numbers"], "scene1_data.json");

        successes+=b;
        //bool b = planner.coll_test(start, polygons);
        std::cout << b << std::endl;
    }


    std::cout << "tests completed " << successes << '/' << 50 << std::endl; 
    //std::string filename = argv[1];
    //run_benchmark("example.xml", "trajectory.xml", 100, start, polygons, goal);

    //std::cout << "polygons_collide: " << polygons_collide(Polygon({Vector2D(0, 0), Vector2D(1, 1), Vector2D(0, 2), Vector2D(-1, 1)}), 
    //Polygon({Vector2D(0, 0), Vector2D(1, 1), Vector2D(0, 2), Vector2D(-1, 1)})) << std::endl;

    using json = nlohmann::json;
    json jsonArray;

    // Create and add the first JSON object
    json object1;
    object1["name"] = "John Doe";
    object1["age"] = 30;
    jsonArray.push_back(object1);

    // Create and add the second JSON object
    json object2;
    object2["name"] = "Jane Smith";
    object2["age"] = 25;
    jsonArray.push_back(object2);

    // Open a file for writing
    std::ofstream file("data.json");

    if (file.is_open()) {
        // Write the JSON array to the file
        file << jsonArray.dump(4);  // Use dump(4) to add indentation for readability

        // Close the file
        file.close();

        std::cout << "JSON data has been written to the file." << std::endl;
    } else {
        std::cout << "Unable to open the file." << std::endl;
    }



    return EXIT_SUCCESS;
}
