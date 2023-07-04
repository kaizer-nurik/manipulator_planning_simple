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
    GoalPoint goal(0.0, 0.0, 0.0, 0.0);
    

    bool read_normally = read_scene(argv[1], polygons, start, goal);
    std::cout << read_normally << std::endl;
    if (! read_normally)
    {
        return EXIT_FAILURE;
    }
    double angle=0.0;
    for (int i=0u; i<start.dof_; i++)
        angle+=start.configuration[i];
    std::cout << "angle:" << angle/6.28 << ' ' << goal.angle2_ << std::endl;

    Planner planner("trajectory.csv");
    bool b = planner.AStar(start, goal, polygons);
    //bool b = planner.coll_test(start, polygons);
    std::cout << b << std::endl;

    //planner.motionPlanning(start, goal, obstacles);
    //std::cout << std::endl;
    //std::cout << b << std::endl;

    //std::cout << "polygons_collide: " << polygons_collide(Polygon({Vector2D(0, 0), Vector2D(1, 1), Vector2D(0, 2), Vector2D(-1, 1)}), 
    //Polygon({Vector2D(0, 0), Vector2D(1, 1), Vector2D(0, 2), Vector2D(-1, 1)})) << std::endl;
    return EXIT_SUCCESS;
}
