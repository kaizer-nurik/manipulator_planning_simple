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

    RRT planner(10,start.dof_,start, goal, polygons);
    for(int i=0; i<10000;i++){
        planner.grow_tree();
        //std::cout<<i<<planner.is_finished()<<std::endl;
        if (planner.is_finished()){
            break;
        }
    
    }
    planner.save("example.csv","example.xml",argv[1]);
    return EXIT_SUCCESS;
}
