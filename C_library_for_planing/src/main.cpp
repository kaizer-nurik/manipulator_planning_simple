#include <iostream>
#include <fstream>
#include <string>
#include <memory>
#include "rapidxml.hpp"
#include "obstacles.h"
#include "robot.h"
#include "parsing.h"
#include "planner.h"
#include <vector>


int main(int argc, const char * argv[])
 {
    std::vector<std::shared_ptr<Obstacle>> obstacles;
    Robot start = Robot();
    Robot goal = Robot();

    bool read_normally = read_scene(argv[1], obstacles, start, goal);

    Robot a(start);
    std::cout << "copy" << std::endl;
    a.PrintJointDetails();
    std::cout << std::endl;
    
    if (! read_normally)
    {
        return EXIT_FAILURE;
    }

    std::cout << obstacles.size()<< std::endl;

    Planner planner("example.csv");
    planner.motionPlanning(start, goal, obstacles);
    std::cout << std::endl;
    //start.PrintJointDetails();
    //goal.PrintJointDetails();



    return EXIT_SUCCESS;
}
