#include <chrono>
#include <iostream>
#include <fstream>
#include <string>
#include <memory>
#include <random>

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

using json = nlohmann::json;

int main(int argc, const char * argv[])
 {
    
//     // Timer t("test");
//     // std::vector<Polygon> polygons;
//     // Robot start = Robot();
//     // GoalPoint goal(0.0, 0.0, 0.0, 0.0);

//     // bool read_normally = read_scene("example.xml", polygons, start, goal); //argv[1]
//     // std::cout << read_normally << std::endl;
//     // if (! read_normally)
//     // {
//     //     return EXIT_FAILURE;
//     // }
//     // double angle=0.0;
//     // for (int i=0u; i<start.dof_; i++)
//     //     angle+=start.configuration[i];
//     // std::cout << "angle:" << angle/6.28 << ' ' << goal.angle2_ << std::endl;
//     // std::cout << goal.goalpoint.x << ' ' << goal.goalpoint.y << std::endl;
//     // Planner planner("example.xml", "trajectory.xml");
//     // bool b = planner.AStar(start, goal, polygons);
//     // //bool b = planner.coll_test(start, polygons);
//     // std::cout << b << std::endl;


    // std::vector<Polygon> polygons;
    // Robot start = Robot();
    // GoalPoint goal(0.0, 0.0, 0.0, 0.0);

    // bool read_normally = read_scene("./dataset/scene1/scene_1_test_1.xml", polygons, start, goal); //argv[1]
    // std::cout << read_normally << std::endl;
    // if (! read_normally)
    // {
    //     return EXIT_FAILURE;
    // }
    // double angle=0.0;
    // for (int i=0u; i<start.dof_; i++)
    //     angle+=start.configuration[i];
    // std::cout << "angle:" << angle/6.28 << ' ' << goal.angle2_ << std::endl;
    // std::cout << goal.goalpoint.x << ' ' << goal.goalpoint.y << std::endl;
    // Planner planner("example.xml", "trajectory.xml");
    // bool b = planner.AStar(start, goal, polygons);
    // //bool b = planner.coll_test(start, polygons);
    // std::cout << b << std::endl;

    //std::cout << periodic(-181,180) << ' ' << periodic(-721,180) << ' ' << periodic(721,180) << ' ' << periodic(181,180) << ' ' << periodic(360,180) << std::endl;

    std::string line = "./dataset/scene3/";  //"./dataset/scene1/scene_1_test_1.xml"
   std::string line2 =  "scene_3_random_test_";
    int successes = 0;
    for (int i=1; i<=50; i++)
    {        
        std::vector<Polygon> polygons;
        Robot start = Robot();
        GoalPoint goal(0.0, 0.0, 0.0, 0.0);

        bool read_normally = read_scene(line+line2 + std::to_string(i) + ".xml", polygons, start, goal); //argv[1]
        std::cout << read_normally << std::endl;
        if (! read_normally)
        {
            return EXIT_FAILURE;
        }
        double angle=0.0;
        // std::vector<Robot> answers;
        // std::cout << "Now IKT\n";
        // sample_all_goals(answers, start, goal, polygons, 10);
        // assert(answers.size()>0);
        // std::cout << answers.size() << std::endl;
        // for (auto r:answers)
        //     print_vector(r.configuration);

        Planner planner(line + line2 + std::to_string(i) + ".xml", line2 + std::to_string(i) +"_trajectory.xml");
        std::map<std::string, double> dict;
        bool b = planner.AStar(start, goal, polygons, dict);
        writeDataToJson(i, dict["g_units"], dict["coord_tolerance"], dict["angle_tolerance"], dict["time"], dict["coll_check_percentage"], dict["opened_nodes"],
        dict["closed_nodes"], dict["g_cost"], dict["turn_numbers"], "scene1_0_1_data.json");

        successes+=b;
        //bool b = planner.coll_test(start, polygons);
        std::cout << b << std::endl;
    }


    std::cout << "tests completed " << successes << '/' << 50 << std::endl; 


//     std::string filename = argv[1];
//     run_benchmark("example.xml", "trajectory.xml", 100, start, polygons, goal);

//     std::cout << "polygons_collide: " << polygons_collide(Polygon({Vector2D(0, 0), Vector2D(1, 1), Vector2D(0, 2), Vector2D(-1, 1)}), 
//     Polygon({Vector2D(0, 0), Vector2D(1, 1), Vector2D(0, 2), Vector2D(-1, 1)})) << std::endl;



//     std::string line = "./dataset/scene3/";  //"./dataset/scene1/scene_1_test_1.xml"
//    std::string line2 =  "scene_3_test_";
//     int successes = 0;
//     for (int i=47; i<=47; i++)
//     {        
        
//         auto b = check_trajectory(line2+std::to_string(i)+"_trajectory.xml");
//         successes+=b;
//         std::cout << i << ' ' << b << std::endl;
//     }


//     std::cout << "tests completed " << successes << '/' << 50 << std::endl; 



//GENERATE RANDOM POSITIONS
    // std::vector<Polygon> polygons;
    // Robot start = Robot();
    // GoalPoint goal(0.0, 0.0, 0.0, 0.0);

    // bool read_normally = read_scene("./dataset/scene2/scene_2_test_2.xml", polygons, start, goal); 


    // for (int ii=1; ii<=50; ii++)
    // {
    //          std::random_device rd;
	//     std::mt19937 engine(rd());
	//     std::uniform_real_distribution<double> dist{ 0.0, 0.9990 };

    //     double b = dist(engine) * std::acos(-1)*2 - std::acos(-1);
    //     Robot probe = Robot();
    //     std::vector<double> angles;

    //     for (int i = 0; i<start.dof_; i++)
    //     {
    //         b = dist(engine) * std::acos(-1)*2 - std::acos(-1);
    //         probe.AddJoint(start.joints[i].length, start.joints[i].width, start.joints[i].limits);
    //         angles.push_back(b);
    //         int count = 0;
    //         while(collide(probe, angles, polygons))
    //         {
    //             b = dist(engine) * std::acos(-1)*2 - std::acos(-1);
    //             angles[i] = b;
    //             count++;
    //             if (count>1000) {ii--; break;}
                
    //         }
            
    //         std::cout << angles[i] << std::endl;
    //     }

    //     for (int i = 0; i<angles.size();i++)
    //     {
    //         std::cout << " <angle number=\"" << i << "\">" << angles[i]*180/std::acos(-1) << "</angle>" << std::endl;
    //     }
    //     Vector2D g = end_effector(start, angles);
    //     std::cout << g.x << ' ' << g.y << ' ' << last_angle(angles)*180/std::acos(-1) << std::endl;


    //     std::ifstream inputFile("./dataset/scene2/scene_2_test_1.xml");
    //     std::ofstream outputFile("./dataset/scene2/scene_2_random_test_" + std::to_string(ii) + ".xml");

    //     if (inputFile.is_open() && outputFile.is_open())
    //     {
    //         std::string line;
    //         while (std::getline(inputFile, line)) 
    //         { 
    //             if (line[3] == 'g')   outputFile << "  <goal_point x=\"" << g.x << "\" y=\"" << g.y << "\" angle=\"" <<last_angle(angles)*180/std::acos(-1) 
    //             << "\" angle_tolerance=\"10\" />"  << std::endl;
    //             else outputFile << line << std::endl;
    //         }
    //         inputFile.close();  
    //         outputFile.close(); 
    //     }
    // }



//VERY IMPORTANT
// std::string filename = "./scene1_random_data.json";
//      {

//     std::ifstream file(filename);
//     if (!file) {
//         std::cerr << "Error opening file: " << filename << std::endl;
//         return 1;
//     }

//     std::string line;
//     std::string jsonData;
//     int entryNumber = 0;
//     std::cout << "time = [";
//     while (std::getline(file, line)) {
//         jsonData += line; // Concatenate the current line to the existing JSON data

//         try {
//             // Try to parse the JSON data
//             json parsedData = json::parse(jsonData);

//             // Access the properties of the JSON object
//             int number = parsedData["_number"];
//             int closedNodes = parsedData["closed_nodes"];
//             double collCheckPercentage = parsedData["coll_check_percentage"];
//             double coordTolerance = parsedData["coord_tolerance"];
//             double gCost = parsedData["g_cost"];
//             int gUnits = parsedData["g_units"];
//             int openedNodes = parsedData["opened_nodes"];
//             double totalTime = parsedData["total_time"];
//             int turnNumber = parsedData["turn_number"];

//             std::cout << totalTime << ", ";

//             jsonData.clear(); // Clear jsonData for the next JSON object
//             entryNumber++;
//         } catch (const std::exception&) {
//             // JSON is not complete yet, continue reading the next line
//             continue;
//         }
//     }}

//     std::cout << "]\n";


// {

//     std::ifstream file(filename);
//     if (!file) {
//         std::cerr << "Error opening file: " << filename << std::endl;
//         return 1;
//     }

//     std::string line;
//     std::string jsonData;
//     int entryNumber = 0;
//     std::cout << "opened_nodes = [";
//     while (std::getline(file, line)) {
//         jsonData += line; // Concatenate the current line to the existing JSON data

//         try {
//             // Try to parse the JSON data
//             json parsedData = json::parse(jsonData);

//             // Access the properties of the JSON object
//             int number = parsedData["_number"];
//             int closedNodes = parsedData["closed_nodes"];
//             double collCheckPercentage = parsedData["coll_check_percentage"];
//             double coordTolerance = parsedData["coord_tolerance"];
//             double gCost = parsedData["g_cost"];
//             int gUnits = parsedData["g_units"];
//             int openedNodes = parsedData["opened_nodes"];
//             double totalTime = parsedData["total_time"];
//             int turnNumber = parsedData["turn_number"];

//             std::cout << openedNodes << ", ";

//             jsonData.clear(); // Clear jsonData for the next JSON object
//             entryNumber++;
//         } catch (const std::exception&) {
//             // JSON is not complete yet, continue reading the next line
//             continue;
//         }
//     }
// }
    

//  std::cout <<"]\n";

// {

//     std::ifstream file(filename);
//     if (!file) {
//         std::cerr << "Error opening file: " << filename << std::endl;
//         return 1;
//     }

//     std::string line;
//     std::string jsonData;
//     int entryNumber = 0;
//     std::cout << "closed_nodes = [";

//     while (std::getline(file, line)) {
//         jsonData += line; // Concatenate the current line to the existing JSON data

//         try {
//             // Try to parse the JSON data
//             json parsedData = json::parse(jsonData);

//             // Access the properties of the JSON object
//             int number = parsedData["_number"];
//             int closedNodes = parsedData["closed_nodes"];
//             double collCheckPercentage = parsedData["coll_check_percentage"];
//             double coordTolerance = parsedData["coord_tolerance"];
//             double gCost = parsedData["g_cost"];
//             int gUnits = parsedData["g_units"];
//             int openedNodes = parsedData["opened_nodes"];
//             double totalTime = parsedData["total_time"];
//             int turnNumber = parsedData["turn_number"];

//             std::cout << closedNodes << ", ";

//             jsonData.clear(); // Clear jsonData for the next JSON object
//             entryNumber++;
//         } catch (const std::exception&) {
//             // JSON is not complete yet, continue reading the next line
//             continue;
//         }
//     }
//     }
//      std::cout <<"]\n";
// {


//     std::ifstream file(filename);
//     if (!file) {
//         std::cerr << "Error opening file: " << filename << std::endl;
//         return 1;
//     }

//     std::string line;
//     std::string jsonData;
//     int entryNumber = 0;
//     std::cout << "coll_check_perc = [";

//     while (std::getline(file, line)) {
//         jsonData += line; // Concatenate the current line to the existing JSON data

//         try {
//             // Try to parse the JSON data
//             json parsedData = json::parse(jsonData);

//             // Access the properties of the JSON object
//             int number = parsedData["_number"];
//             int closedNodes = parsedData["closed_nodes"];
//             double collCheckPercentage = parsedData["coll_check_percentage"];
//             double coordTolerance = parsedData["coord_tolerance"];
//             double gCost = parsedData["g_cost"];
//             int gUnits = parsedData["g_units"];
//             int openedNodes = parsedData["opened_nodes"];
//             double totalTime = parsedData["total_time"];
//             int turnNumber = parsedData["turn_number"];

//             std::cout << collCheckPercentage << ", ";

//             jsonData.clear(); // Clear jsonData for the next JSON object
//             entryNumber++;
//         } catch (const std::exception&) {
//             // JSON is not complete yet, continue reading the next line
//             continue;
//         }
//     }
// }

//     std::cout <<"]\n";
// {

//     std::ifstream file(filename);
//     if (!file) {
//         std::cerr << "Error opening file: " << filename << std::endl;
//         return 1;
//     }

//     std::string line;
//     std::string jsonData;
//     int entryNumber = 0;
//     std::cout << "gcost = [";
//     while (std::getline(file, line)) {
//         jsonData += line; // Concatenate the current line to the existing JSON data

//         try {
//             // Try to parse the JSON data
//             json parsedData = json::parse(jsonData);

//             // Access the properties of the JSON object
//             int number = parsedData["_number"];
//             int closedNodes = parsedData["closed_nodes"];
//             double collCheckPercentage = parsedData["coll_check_percentage"];
//             double coordTolerance = parsedData["coord_tolerance"];
//             double gCost = parsedData["g_cost"];
//             int gUnits = parsedData["g_units"];
//             int openedNodes = parsedData["opened_nodes"];
//             double totalTime = parsedData["total_time"];
//             int turnNumber = parsedData["turn_number"];

//             std::cout << gCost << ", ";

//             jsonData.clear(); // Clear jsonData for the next JSON object
//             entryNumber++;
//         } catch (const std::exception&) {
//             // JSON is not complete yet, continue reading the next line
//             continue;
//         }
//     }
// }
    return EXIT_SUCCESS;
}
