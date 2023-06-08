#include <iostream>
#include <fstream>
#include <string>
#include <memory>
#include "rapidxml.hpp"
#include "obstacles.h"
#include "robot.h"
#include "parsing.h"
#include <vector>

int main(int argc, const char * argv[])
 {
    std::vector<std::shared_ptr<Obstacle>> obstacles;
    Robot start = Robot();
    Robot goal = Robot();
    read_scene(argv[1], obstacles, start, goal);

    std::cout << obstacles.size()<< std::endl;

    std::cout << std::endl;
    start.PrintJointDetails();
    goal.PrintJointDetails();

    return 0;
}
