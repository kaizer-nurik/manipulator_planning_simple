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

std::string getexepath()
{
    char result[PATH_MAX];
    ssize_t count = readlink("/proc/self/exe", result, PATH_MAX);
    return std::string(result, (count > 0) ? count : 0);
}

bool folder_check(const std::string &scene_dir, const std::string &test_name)
{
    std::string line = scene_dir;  //"./dataset/scene1/scene_1_test_1.xml" //"./dataset/scene1/"
    std::string line2 = test_name; //"scene_1_random_test_"
    int successes = 0;

    int first = 1;
    int last = 50;
    for (int i = first; i <= last; i++)
    {
        std::vector<Polygon> polygons;
        Robot start = Robot();
        GoalPoint goal(0.0, 0.0, 0.0, 0.0);

        bool read_normally = read_scene(line + line2 + std::to_string(i) + ".xml", polygons, start, goal);
        std::cout << read_normally << std::endl;
        if (!read_normally)
        {
            return EXIT_FAILURE;
        }

        Planner_A_star planner(line + line2 + std::to_string(i) + ".xml", line2 + std::to_string(i) + "_trajectory.xml");
        std::map<std::string, double> dict;
        bool b = planner.AStar(start, goal, polygons, dict);
        writeDataToJson(i, dict["g_units"], dict["coord_tolerance"], dict["angle_tolerance"], dict["time"], dict["coll_check_percentage"], dict["opened_nodes"],
                        dict["closed_nodes"], dict["g_cost"], dict["turn_numbers"], "scene1_0_1_data.json");

        successes += b;
        std::cout << b << std::endl;
    }
    std::cout << "tests completed " << successes << '/' << 50 << std::endl;

    return true;
}

bool json_to_python_code(const std::string &filename) // json_to_python_code("./scene3_random_data.json")
{
    {

        std::ifstream file(filename);
        if (!file)
        {
            std::cerr << "Error opening file: " << filename << std::endl;
            return 1;
        }

        std::string line;
        std::string jsonData;
        int entryNumber = 0;
        std::cout << "time = [";
        while (std::getline(file, line))
        {
            jsonData += line; // Concatenate the current line to the existing JSON data

            try
            {
                // Try to parse the JSON data
                json parsedData = json::parse(jsonData);

                // Access the properties of the JSON object
                int number = parsedData["_number"];
                int closedNodes = parsedData["closed_nodes"];
                double collCheckPercentage = parsedData["coll_check_percentage"];
                double coordTolerance = parsedData["coord_tolerance"];
                double gCost = parsedData["g_cost"];
                int gUnits = parsedData["g_units"];
                int openedNodes = parsedData["opened_nodes"];
                double totalTime = parsedData["total_time"];
                int turnNumber = parsedData["turn_number"];

                std::cout << totalTime << ", ";

                jsonData.clear(); // Clear jsonData for the next JSON object
                entryNumber++;
            }
            catch (const std::exception &)
            {
                // JSON is not complete yet, continue reading the next line
                continue;
            }
        }
    }

    std::cout << "]\n";

    {

        std::ifstream file(filename);
        if (!file)
        {
            std::cerr << "Error opening file: " << filename << std::endl;
            return 1;
        }

        std::string line;
        std::string jsonData;
        int entryNumber = 0;
        std::cout << "opened_nodes = [";
        while (std::getline(file, line))
        {
            jsonData += line; // Concatenate the current line to the existing JSON data

            try
            {
                // Try to parse the JSON data
                json parsedData = json::parse(jsonData);

                // Access the properties of the JSON object
                int number = parsedData["_number"];
                int closedNodes = parsedData["closed_nodes"];
                double collCheckPercentage = parsedData["coll_check_percentage"];
                double coordTolerance = parsedData["coord_tolerance"];
                double gCost = parsedData["g_cost"];
                int gUnits = parsedData["g_units"];
                int openedNodes = parsedData["opened_nodes"];
                double totalTime = parsedData["total_time"];
                int turnNumber = parsedData["turn_number"];

                std::cout << openedNodes << ", ";

                jsonData.clear(); // Clear jsonData for the next JSON object
                entryNumber++;
            }
            catch (const std::exception &)
            {
                // JSON is not complete yet, continue reading the next line
                continue;
            }
        }
    }

    std::cout << "]\n";

    {

        std::ifstream file(filename);
        if (!file)
        {
            std::cerr << "Error opening file: " << filename << std::endl;
            return 1;
        }

        std::string line;
        std::string jsonData;
        int entryNumber = 0;
        std::cout << "closed_nodes = [";

        while (std::getline(file, line))
        {
            jsonData += line; // Concatenate the current line to the existing JSON data

            try
            {
                // Try to parse the JSON data
                json parsedData = json::parse(jsonData);

                // Access the properties of the JSON object
                int number = parsedData["_number"];
                int closedNodes = parsedData["closed_nodes"];
                double collCheckPercentage = parsedData["coll_check_percentage"];
                double coordTolerance = parsedData["coord_tolerance"];
                double gCost = parsedData["g_cost"];
                int gUnits = parsedData["g_units"];
                int openedNodes = parsedData["opened_nodes"];
                double totalTime = parsedData["total_time"];
                int turnNumber = parsedData["turn_number"];

                std::cout << closedNodes << ", ";

                jsonData.clear(); // Clear jsonData for the next JSON object
                entryNumber++;
            }
            catch (const std::exception &)
            {
                // JSON is not complete yet, continue reading the next line
                continue;
            }
        }
    }
    std::cout << "]\n";
    {

        std::ifstream file(filename);
        if (!file)
        {
            std::cerr << "Error opening file: " << filename << std::endl;
            return 1;
        }

        std::string line;
        std::string jsonData;
        int entryNumber = 0;
        std::cout << "coll_check_perc = [";

        while (std::getline(file, line))
        {
            jsonData += line; // Concatenate the current line to the existing JSON data

            try
            {
                // Try to parse the JSON data
                json parsedData = json::parse(jsonData);

                // Access the properties of the JSON object
                int number = parsedData["_number"];
                int closedNodes = parsedData["closed_nodes"];
                double collCheckPercentage = parsedData["coll_check_percentage"];
                double coordTolerance = parsedData["coord_tolerance"];
                double gCost = parsedData["g_cost"];
                int gUnits = parsedData["g_units"];
                int openedNodes = parsedData["opened_nodes"];
                double totalTime = parsedData["total_time"];
                int turnNumber = parsedData["turn_number"];

                std::cout << collCheckPercentage << ", ";

                jsonData.clear(); // Clear jsonData for the next JSON object
                entryNumber++;
            }
            catch (const std::exception &)
            {
                // JSON is not complete yet, continue reading the next line
                continue;
            }
        }
    }

    std::cout << "]\n";
    {

        std::ifstream file(filename);
        if (!file)
        {
            std::cerr << "Error opening file: " << filename << std::endl;
            return 1;
        }

        std::string line;
        std::string jsonData;
        int entryNumber = 0;
        std::cout << "gcost = [";
        while (std::getline(file, line))
        {
            jsonData += line; // Concatenate the current line to the existing JSON data

            try
            {
                // Try to parse the JSON data
                json parsedData = json::parse(jsonData);

                // Access the properties of the JSON object
                int number = parsedData["_number"];
                int closedNodes = parsedData["closed_nodes"];
                double collCheckPercentage = parsedData["coll_check_percentage"];
                double coordTolerance = parsedData["coord_tolerance"];
                double gCost = parsedData["g_cost"];
                int gUnits = parsedData["g_units"];
                int openedNodes = parsedData["opened_nodes"];
                double totalTime = parsedData["total_time"];
                int turnNumber = parsedData["turn_number"];

                std::cout << gCost << ", ";

                jsonData.clear(); // Clear jsonData for the next JSON object
                entryNumber++;
            }
            catch (const std::exception &)
            {
                // JSON is not complete yet, continue reading the next line
                continue;
            }
        }
        std::cout << "]\n";
        return true;
    }
}

int main(int argc, const char *argv[])
{

    std::cout << getexepath() << std::endl;

    std::string path = argv[1]; // Первый параметр - кодирует сцену,

    Timer t("FULL TIMER");
    int dof = 7;
    std::string line = "../../DATASET/SCENE2/"+ std::to_string(dof) + "dof/"; //"./dataset/scene" + path + "/";
    
    std::string line2 = "test_";
    // if (path.substr(1, 1) == "r")
    // {
    //     line2 = "scene_" + path.substr(0, 1) + "_random_test_";
    // }
    // else
    // {
    //     line2 = "scene_" + path.substr(0, 1) + "_test_";
    // }

    std::cout << line + line2 + /*std::to_string(1)*/  ".xml" << std::endl;
    int successes = 0;
    std::vector<int> successed;
    for (int i = 1; i <= 100; i++)
    {   
        std::cout << "TEST NUMBER: " << i << std::endl;
        //int i = 30;
        std::vector<Polygon> polygons;
        Robot start = Robot();
        GoalPoint goal(0.0, 0.0, 0.0, 0.0);

        bool read_normally = read_scene(line + line2 + std::to_string(i) + ".xml", polygons, start, goal); // argv[1]
        std::cout << read_normally << std::endl;
        if (!read_normally)
        {
            return EXIT_FAILURE;
        }

        // double delta = 360 / 100.0;
        // double a1 = 0.0;
        // double a2 = 0.0;
        // std::ofstream outputFile("output.txt");
        // for (int i1=0; i1<100; i1++)
        // {
        //     for (int i2=0; i2<100; i2++)
        //     {
        //         a1 = i1 * delta - 180;
        //         a2 = i2 * delta - 180;
        //         outputFile << a1 << ',' << a2 << ',' << collide(start, {a1, a2}, polygons) << std::endl;
        //     }
        // }
        // outputFile.close();

        Planner_A_star Planner_A_star(line + line2 + std::to_string(i) + ".xml", "./s2/" + std::to_string(dof) + "dof/scene/"+line2 + std::to_string(i) + "_trajectory.xml");
        std::map<std::string, double> dict;
        bool b = Planner_A_star.AStar(start, goal, polygons, dict);
        writeDataToJson(i, dict["g_units"], dict["coord_tolerance"], dict["angle_tolerance"], dict["time"], dict["coll_check_percentage"], dict["opened_nodes"],
                       dict["closed_nodes"], dict["g_cost"], dict["turn_numbers"], "./s2/" + std::to_string(dof) + "dof/scene1_data.json");

        successes += b;
        if (!b) successed.push_back(i);
        //bool b = Planner_A_star.coll_test(start, polygons);
        std::cout << b << std::endl;
    }

    std::cout << "\ntests completed " << successes << '/' << 100 << std::endl;
    for (const auto &i:successed)
        std::cout << i << ", ";
    std::cout << std::endl;

    return EXIT_SUCCESS;
}
