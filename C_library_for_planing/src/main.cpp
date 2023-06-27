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

    RRT planner(0.01,start.dof_,start, goal, polygons);
    for(int i=0; i<1000000;i++){
        planner.grow_tree();
        std::cout<<planner.is_finished()<<std::endl;
    
    }
    return EXIT_SUCCESS;
}
