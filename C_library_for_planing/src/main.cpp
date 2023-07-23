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
#include "file_saver.h"

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

    FileSaver::save_csv<double>("output.csv",planner.get_path(),planner.get_dof());

    // Функция, Записывающая csv файл в xml под <csv></csv>
    FileSaver::save_csv_to_xml<double>("output.xml",argv[1],planner.get_path(),planner.get_dof());

    // Функция, сохраняющая дерево из rrt для визуализации в питоне.
    FileSaver::save_rrt_tree("rrt_tree.csv", planner.get_tree());
    return EXIT_SUCCESS;
}
