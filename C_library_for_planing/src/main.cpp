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
#include "file_saver.h"

void writeDataToCsv(int test,  int node_count, double coord_tolerance, double angle_tolerance, 
double time, int goal_node_count, int IK_solutions_count, const std::string& filename) {
    
    std::ofstream file(filename, std::ios::app);

    if (file.is_open()) {
        file << test<<','<<node_count<< "," <<coord_tolerance<<","<<angle_tolerance<<","<< time<<","<<goal_node_count<<","<<IK_solutions_count<<std::endl;
        file.close();
    } else {
        std::cout << "Unable to open the file." << std::endl;
    }
}

int main(int argc, const char * argv[])
 {
   

    
    std::string line = "./dataset/scene1/";  //"./dataset/scene1/scene_1_test_1.xml"
   std::string line2 =  "scene_1_test_";
    int successes = 0;
    for (int i=1; i<=50; i++)
    {        
        std::cout<<line+line2 + std::to_string(i) + ".xml"<<std::endl;
        std::vector<Polygon> polygons;
        Robot start = Robot();
        GoalPoint goal(0.0, 0.0, 0.0, 0.0);

        bool read_normally = read_scene(line+line2 + std::to_string(i) + ".xml", polygons, start, goal); 
        std::cout <<i<<" "<< read_normally << std::endl;
        if (! read_normally)
        {
            return EXIT_FAILURE;
        }
        double angle=0.0;
        for (int i=0u; i<start.dof_; i++)
            angle+=start.configuration[i];
        std::cout << "angle:" << angle/6.28 << ' ' << goal.angle2_ << std::endl;
        std::cout << goal.goalpoint.x << ' ' << goal.goalpoint.y << std::endl;
        RRT planner(10,start.dof_,start, goal, polygons);
        std::map<std::string, double> dict;
        bool reached_goal = false;
        for(int i=0; i<10000;i++){
        planner.grow_tree();
        //std::cout<<i<<planner.is_finished()<<std::endl;

        if (planner.is_finished())
        {
            reached_goal = true;
            break;
        }
    
    }
        // writeDataToCsv(i, dict["node_count"], dict["coord_tolerance"], dict["angle_tolerance"], dict["time"],
        // dict["goal_node_count"], dict["IK_solutions_count"], "scene1_data.csv");
        if(reached_goal){
        FileSaver::save_csv<double>(line2 + std::to_string(i) +"_trajectory.csv",planner.get_path(),planner.get_dof());

        // Функция, Записывающая csv файл в xml под <csv></csv>
        FileSaver::save_csv_to_xml<double>(line2 + std::to_string(i) +"_trajectory.xml",line + line2 + std::to_string(i) + ".xml" ,planner.get_path(),planner.get_dof());
        }

        // Функция, сохраняющая дерево из rrt для визуализации в питоне.
        FileSaver::save_rrt_tree(line2 + std::to_string(i)+"rrt_tree.csv", planner.get_tree());
        successes+=reached_goal;
        //bool b = planner.coll_test(start, polygons);
        std::cout << reached_goal << std::endl;
    }


    std::cout << "tests completed " << successes << '/' << 50 << std::endl; 
      return EXIT_SUCCESS;
}
