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
    std::vector<std::shared_ptr<Obstacle>> obstacles;
    Robot start = Robot();
    Robot goal = Robot();

    bool read_normally = read_scene(argv[1], obstacles, start, goal);

    Point point(4.0, 7.0);
   for (auto i=0u; i<obstacles.size(); i++)
   {
        std::cout << obstacles[i]->distancetoPoint(point) << std::endl;
   }
    if (! read_normally)
    {
        return EXIT_FAILURE;
    }


    Planner planner("example.csv");
    //planner.motionPlanning(start, goal, obstacles);
    std::cout << std::endl;
    //start.PrintJointDetails();
    //goal.PrintJointDetails();

    
    Segment segment(Point(1.0, 1.0), Point(4.0, 5.0));

    double distance = distanceToSegment(point, segment);
    std::cout << "Distance from point to segment: " << distance << std::endl;

    return EXIT_SUCCESS;
}
