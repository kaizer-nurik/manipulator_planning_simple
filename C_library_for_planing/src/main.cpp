#include <iostream>
#include <fstream>
#include <string>
#include <memory>
#include "rapidxml.hpp"
#include "obstacles.h"
#include "robot.h"
#include "parsing.h"
#include "planner.h"
#include "geometry.h"
#include <vector>


int main(int argc, const char * argv[])
 {
    std::vector<Polygon> polygons;
    Robot start = Robot();
    Vector2D goal(0.0, 0.0);
    

    bool read_normally = read_scene(argv[1], polygons, start, goal);
    std::cout << read_normally << std::endl;
    if (! read_normally)
    {
        return EXIT_FAILURE;
    }


    Planner planner("example.csv");
    //planner.motionPlanning(start, goal, obstacles);
    std::cout << polygons.size() <<  std::endl;
    //start.PrintJointDetails();
    //goal.PrintJointDetails();

    for (const auto &i:polygons)
    {
        for (const auto &j:polygons)
        {
            std::cout << checkCollision(i, j) << std::endl;
        }
    }

    return EXIT_SUCCESS;
}
