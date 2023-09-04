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
#include <chrono>
#include <ctime>
#include <filesystem>
#define TIMEOUT_MS 10000
#define ANGLE_STEP 10


int main(int argc, const char * argv[])
 {
   
    using namespace std::literals;   
    
    std::string line = "./dataset/scene" + std::string(argv[1])+"/";
    std::string line2 = "";
    if (std::string(argv[1]).substr(0,2) == "r"){
        line2 =  "scene_"+std::string(argv[1]).substr(0,1)+"_random_test_";
    } 
    else{
        line2 =  "scene_"+std::string(argv[1]).substr(0,1)+"_random_test_";
    }
    std::string result_dir = "./results_of_scene"+std::string(argv[1])+"/";
    std::string xml_trajectories_dir = result_dir+"scene"+std::string(argv[1])+"/";
    std::filesystem::create_directory(result_dir);
    std::filesystem::create_directory(xml_trajectories_dir);

    int successes = 0;
    for (int i=1; i<=50; i++)
    {        
        std::cout<<line+line2 + std::to_string(i) + ".xml"<<std::endl;
        std::vector<Polygon> polygons;
        Robot start = Robot();
        GoalPoint goal(0.0, 0.0, 0.0, 0.0);

        bool read_normally = read_scene(line+line2 + std::to_string(i) + "_trajectory.xml", polygons, start, goal); 
        std::cout <<i<<" "<< read_normally << std::endl;
        if (! read_normally)
        {
            return EXIT_FAILURE;
        }
        double angle=0.0;

        std::cout << "angle:" << goal.angle1_ << ' ' << goal.angle2_ << std::endl;
        std::cout << goal.goalpoint.x << ' ' << goal.goalpoint.y << std::endl;
        std::map<std::string, std::string> stats;
        bool reached_goal = false;
        bool timeout = false;
        auto t1 = std::chrono::high_resolution_clock::now();
        auto t2 = std::chrono::high_resolution_clock::now();
        auto duration = t2-t1;
        RRT planner(ANGLE_STEP,start.dof_,start, goal, polygons,stats);
        if(planner.get_end_configurations().size()>0){
        for(int i=0; i<10000;i++){
            planner.grow_tree();
            //std::cout<<i<<planner.is_finished()<<std::endl;
            if (planner.is_finished())
            {
                reached_goal = true;
                break;
            }
            t2 = std::chrono::high_resolution_clock::now();
            duration = t2-t1;
            if (duration/1ms > TIMEOUT_MS){
                timeout = true;
                break;
            }
    }}
        planner.export_stats();
        if(reached_goal){
        FileSaver::save_csv<double>(result_dir+line2 + std::to_string(i) +"_trajectory.csv",planner.get_path(),planner.get_dof());

        // Функция, Записывающая csv файл в xml под <csv></csv>
        FileSaver::save_csv_to_xml<double>(xml_trajectories_dir+line2 + std::to_string(i) +"_trajectory.xml",line + line2 + std::to_string(i) + "_trajectory.xml" ,planner.get_path(),planner.get_dof());
        }

        // Функция, сохраняющая дерево из rrt для визуализации в питоне.
        FileSaver::save_rrt_tree(result_dir+line2 + std::to_string(i)+"rrt_tree.csv", planner.get_tree());
        successes+=reached_goal;
        //bool b = planner.coll_test(start, polygons);
        std::cout << reached_goal << std::endl;
        std::cout << "duration: " << duration/ 1ns <<" ns" << std::endl;
        stats["reached_goal"] = std::to_string(reached_goal);
        stats["timeout"] = std::to_string(timeout);
        stats["total_duration"] = std::to_string(duration/1ns);
        stats["angle_step"] = std::to_string(ANGLE_STEP);
        
        FileSaver::write_map_to_json(result_dir+line2 + std::to_string(i)+"_stats.json",stats);
        FileSaver::write_end_config_to_csv(result_dir+line2 + std::to_string(i)+"_IK_res.csv", planner.get_end_configurations());
    
    }


    std::cout << "tests completed " << successes << '/' << 50 << std::endl; 
      return EXIT_SUCCESS;
}
