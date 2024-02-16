#include <chrono>
#include <iostream>
#include <fstream>
#include <string>
#include <memory>
#include <random>
#include <limits.h>
#include <unistd.h>

#include "rapidxml.hpp"
#include "rapidxml_print.hpp"
#include "obstacles.h"
#include "robot.h"
#include "parsing.h"
#include "planner.h"
#include "benchmark.h"
#include <vector>
#include <map>

using json = nlohmann::json;

    double periodic_c(const double &n, const double &L)
    {
        if (n >= 0.0)
            return fmod(n + L, 2 * L) - L;
        else if (n >= -L && n < 0.0)
            return n;
        else
            return fmod(n + L, 2 * L) + L;
    }

    double last_angle_c(const std::vector<double> &angles)
    {
        double angle = 0.0;
        for (int i = 0; i < angles.size(); i++)
        {
            angle += angles[i];
        }
        return periodic_c(angle, 180);
    }



int main(int argc, const char *argv[])
{

    std::string path = argv[1]; // Первый параметр - путь к файлу

    std::cout << path + std::to_string(1) + ".xml" << std::endl;
    // int successes = 0;

    // std::vector<Polygon> polygons;
    // Robot start = Robot();
    // GoalPoint goal(0.0, 0.0, 0.0, 0.0);

    // bool read_normally = read_scene(path, polygons, start, goal); // argv[1]
    // std::cout << read_normally << std::endl;
    // if (!read_normally)
    // {
    //     return EXIT_FAILURE;
    // }

    // Planner_A_star Planner_A_star(path, "result_trajectory.xml");
    // std::map<std::string, double> dict;
    // bool b = Planner_A_star.AStar(start, goal, polygons, dict);
    // writeDataToJson(0, dict["g_units"], dict["coord_tolerance"], dict["angle_tolerance"], dict["time"], dict["coll_check_percentage"], dict["opened_nodes"],
    //                 dict["closed_nodes"], dict["g_cost"], dict["turn_numbers"], "scene1_0_1_data.json");

    //bool b = Planner_A_star.coll_test(start, polygons);


    //GENERATE RANDOM POSITIONS
    
    
    


    {std::vector<Polygon> polygons;
    Robot start = Robot();
    GoalPoint goal(0.0, 0.0, 0.0, 0.0);
    std::string destination = "../../DATASET/SCENE1/5dof/";

    bool read_normally = read_scene(destination+"test.xml", polygons, start, goal); 
    std::cout << read_normally << std::endl;
    std::random_device rd;
	std::mt19937 engine(975);
	std::uniform_real_distribution<double> dist{ 0.0, 0.9990 };

    for (int ii=101; ii<=1000; ii++)
    {


        double b = dist(engine) * (std::acos(-1)*2 - std::acos(-1))*170;
        Robot probe = Robot();
        std::vector<double> angles;

        // for (int i = 0; i<start.dof_; i++)
        // {
        //     probe.AddJoint(start.joints[i].length, start.joints[i].width, start.joints[i].limits);
        //     b = dist(engine) * (std::acos(-1)*2 - std::acos(-1))*1.7/1.8;
        //     angles.push_back(b);
        //     while(collide(probe, angles, polygons))
        //     {
        //         b = dist(engine) * (std::acos(-1)*2 - std::acos(-1))*170;
        //         angles[i] = b;
        //         int count = 0;            
        //         std::cout << angles[i] << std::endl;
        //     }
        // }


        for (int i = 0; i<start.dof_; i++)
        {
            probe.AddJoint(start.joints[i].length, start.joints[i].width, start.joints[i].limits);
            b = dist(engine) * (std::acos(-1)*2 - std::acos(-1))*170;
            angles.push_back(b);
        }

        while(collide(probe, angles, polygons))
            {
            for (int i = 0; i<start.dof_; i++)
            {
                b = dist(engine) * (std::acos(-1)*2 - std::acos(-1))*170;
                angles[i] = b;        
                std::cout << angles[i] << std::endl;
            }
                
            }

        std::vector<double> vv;
        for (int i = 0; i<angles.size();i++)
        {
            std::cout << " <angle number=\"" << i << "\">" << angles[i] << "</angle>" << std::endl;
            vv.push_back(angles[i]);
        }
        Vector2D g = end_effector(start, vv);
        std::cout << g.x << ' ' << g.y << ' ' << last_angle_c(angles) << std::endl;


        std::ifstream inputFile(destination + "test.xml");
        std::ofstream outputFile(destination + "test_" + std::to_string(ii) + ".xml");
        std::cout << "last angle: " << last_angle_c(angles) << std::endl;
        if (inputFile.is_open() && outputFile.is_open())
        {
            std::string line;
            while (std::getline(inputFile, line)) 
            { 
                if (line[3] == 'g')   outputFile << "  <goal_point x=\"" << g.x << "\" y=\"" << g.y << "\" delta_radius=\"0.1\" angle=\"" <<last_angle_c(angles)
                << "\" angle_tolerance=\"10\" />"  << std::endl;
                else outputFile << line << std::endl;
            }
            inputFile.close();  
            outputFile.close(); 
        }
    }
    }

    //  {std::vector<Polygon> polygons;
    // Robot start = Robot();
    // GoalPoint goal(0.0, 0.0, 0.0, 0.0);
    // std::string destination = "../../DATASET/SCENE3/7dof/";

    // bool read_normally = read_scene(destination+"test.xml", polygons, start, goal); 


    // for (int ii=1; ii<=100; ii++)
    // {
    //     std::random_device rd;
	//     std::mt19937 engine(rd());
	//     std::uniform_real_distribution<double> dist{ 0.0, 0.9990 };

    //     double b = dist(engine) * (std::acos(-1)*2 - std::acos(-1))*1.7/1.8;
    //     Robot probe = Robot();
    //     std::vector<double> angles;

    //     for (int i = 0; i<start.dof_; i++)
    //     {
    //         b = dist(engine) * std::acos(-1)*2 - std::acos(-1);
    //         probe.AddJoint(start.joints[i].length, start.joints[i].width, start.joints[i].limits);
    //         angles.push_back(b);
    //         int count = 0;            
    //         std::cout << angles[i] << std::endl;
    //     }

    //     while(collide(probe, angles, polygons))
    //         {
    //         for (int i = 0; i<start.dof_; i++)
    //         {
    //             b = dist(engine) * std::acos(-1)*2 - std::acos(-1);
    //             angles[i] = b;        
    //             std::cout << angles[i] << std::endl;
    //         }
                
    //         }

    //     std::vector<double> vv;
    //     for (int i = 0; i<angles.size();i++)
    //     {
    //         std::cout << " <angle number=\"" << i << "\">" << angles[i]*180/std::acos(-1) << "</angle>" << std::endl;
    //         vv.push_back(angles[i]*180/std::acos(-1));
    //     }
    //     Vector2D g = end_effector(start, vv);
    //     std::cout << g.x << ' ' << g.y << ' ' << last_angle_c(angles)*180/std::acos(-1) << std::endl;


    //     std::ifstream inputFile(destination + "test.xml");
    //     std::ofstream outputFile(destination + "test_" + std::to_string(ii) + ".xml");

    //     if (inputFile.is_open() && outputFile.is_open())
    //     {
    //         std::string line;
    //         while (std::getline(inputFile, line)) 
    //         { 
    //             if (line[3] == 'g')   outputFile << "  <goal_point x=\"" << g.x << "\" y=\"" << g.y << "\" delta_radius=\"0.1\" angle=\"" <<last_angle_c(angles)*180/std::acos(-1) 
    //             << "\" angle_tolerance=\"10\" />"  << std::endl;
    //             else outputFile << line << std::endl;
    //         }
    //         inputFile.close();  
    //         outputFile.close(); 
    //     }
    // }
    // }

    // {std::vector<Polygon> polygons;
    // Robot start = Robot();
    // GoalPoint goal(0.0, 0.0, 0.0, 0.0);
    // std::string destination = "../../DATASET/SCENE3/8dof/";

    // bool read_normally = read_scene(destination+"test.xml", polygons, start, goal); 


    // for (int ii=1; ii<=100; ii++)
    // {
    //     std::random_device rd;
	//     std::mt19937 engine(rd());
	//     std::uniform_real_distribution<double> dist{ 0.0, 0.9990 };

    //     double b = dist(engine) * (std::acos(-1)*2 - std::acos(-1))*1.7/1.8;
    //     Robot probe = Robot();
    //     std::vector<double> angles;

    //     for (int i = 0; i<start.dof_; i++)
    //     {
    //         b = dist(engine) * std::acos(-1)*2 - std::acos(-1);
    //         probe.AddJoint(start.joints[i].length, start.joints[i].width, start.joints[i].limits);
    //         angles.push_back(b);
    //         int count = 0;            
    //         std::cout << angles[i] << std::endl;
    //     }

    //     while(collide(probe, angles, polygons))
    //         {
    //         for (int i = 0; i<start.dof_; i++)
    //         {
    //             b = dist(engine) * std::acos(-1)*2 - std::acos(-1);
    //             angles[i] = b;        
    //             std::cout << angles[i] << std::endl;
    //         }
                
    //         }

    //     std::vector<double> vv;
    //     for (int i = 0; i<angles.size();i++)
    //     {
    //         std::cout << " <angle number=\"" << i << "\">" << angles[i]*180/std::acos(-1) << "</angle>" << std::endl;
    //         vv.push_back(angles[i]*180/std::acos(-1));
    //     }
    //     Vector2D g = end_effector(start, vv);
    //     std::cout << g.x << ' ' << g.y << ' ' << last_angle_c(angles)*180/std::acos(-1) << std::endl;


    //     std::ifstream inputFile(destination + "test.xml");
    //     std::ofstream outputFile(destination + "test_" + std::to_string(ii) + ".xml");

    //     if (inputFile.is_open() && outputFile.is_open())
    //     {
    //         std::string line;
    //         while (std::getline(inputFile, line)) 
    //         { 
    //             if (line[3] == 'g')   outputFile << "  <goal_point x=\"" << g.x << "\" y=\"" << g.y << "\" delta_radius=\"0.1\" angle=\"" <<last_angle_c(angles)*180/std::acos(-1) 
    //             << "\" angle_tolerance=\"10\" />"  << std::endl;
    //             else outputFile << line << std::endl;
    //         }
    //         inputFile.close();  
    //         outputFile.close(); 
    //     }
    // }
    // }

    return EXIT_SUCCESS;
}
