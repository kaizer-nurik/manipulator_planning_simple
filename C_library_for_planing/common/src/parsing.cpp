#include "parsing.h"
#include <iostream>
#include <fstream>
#include <string>
#include <memory>
#include <vector>
#include <cmath>
#include "rapidxml.hpp"
#include "obstacles.h"
#include "robot.h"
bool read_scene(const std::string &filename, std::vector<Polygon> &polygons, Robot &start, GoalPoint &goal)
{
    std::ifstream file(filename);
    if (!file) {
        std::cerr << "Failed to open XML file." << std::endl;
        return false;
    }

    std::string xmlString((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    xmlString.push_back('\0'); 
    double pi = std::acos(-1);
    try 
    {
        rapidxml::xml_document<> doc;
        doc.parse<0>(&xmlString[0]);

        rapidxml::xml_node<>* rootNode = doc.first_node("input_info");
        if (!rootNode) 
        {
            std::cerr << "Invalid XML format. Root node not found." << std::endl;
            return false;
        }

        rapidxml::xml_node<>* robot_info = rootNode->first_node("robot_info");
        if (robot_info) 
        {
            // Read joints
            rapidxml::xml_node<>* jointsNode = robot_info->first_node("joints");
            if (jointsNode)
            {
                for (rapidxml::xml_node<>* jointNode = jointsNode->first_node("joint"); 
                jointNode; jointNode = jointNode->next_sibling("joint")) 
                {    
                    int number = std::stoi(jointNode->first_attribute("number")->value());   
                                 
                    double length = std::stod(jointNode->first_attribute("length")->value());
                    //std::cout << "Link Length: " << length << std::endl;

                    double width = std::stod(jointNode->first_attribute("width")->value());
                    //std::cout << "width: " << width << std::endl;

                    double limit1 = std::stod(jointNode->first_attribute("limit_min")->value()) ;
                    //std::cout << "limit1: " << limit1 << std::endl;

                    double limit2 = std::stod(jointNode->first_attribute("limit_max")->value());
                    //std::cout << "limit2: " << limit2 << std::endl;
                    start.AddJoint(length, width,  { limit1, limit2 });
                }
            }
        }
        rapidxml::xml_node<>* startconfNode = rootNode->first_node("start_configuration");
        if (startconfNode)
        {
            std::vector<double> angles;
            for (rapidxml::xml_node<>* angleNode = startconfNode->first_node("angle"); 
            angleNode; angleNode = angleNode->next_sibling("angle")) 
            {
                std::string angleStr = angleNode->value();
                double angle = std::stod(angleStr);
                angles.push_back(angle);
            }
            start.configuration = angles;
        }

        rapidxml::xml_node<>* goalNode = rootNode->first_node("goal_point");
        if (goalNode)
        {
            double x = std::stod(goalNode->first_attribute("x")->value());
            double y = std::stod(goalNode->first_attribute("y")->value());
            double angle1 = std::stod(goalNode->first_attribute("angle")->value());
            double angle2 = std::stod(goalNode->first_attribute("angle_tolerance")->value());
            goal.goalpoint.x = x; goal.goalpoint.y = y;
            goal.angle1_ = angle1;
            goal.angle2_ = angle2;
        }

        // Parse the scene obstacles
        rapidxml::xml_node<>* sceneNode = rootNode->first_node("scene");
        if (sceneNode) 
        {
            for (rapidxml::xml_node<>* polygonNode = sceneNode->first_node("polygon"); polygonNode; 
                polygonNode = polygonNode->next_sibling("polygon")) 
            {
                std::vector<Vector2D> vertexes;
                for (rapidxml::xml_node<>* vertexNode = polygonNode->first_node("vertex"); vertexNode; 
                    vertexNode = vertexNode->next_sibling("vertex"))
                {
                    double x = std::stod(vertexNode->first_attribute("x")->value());
                    double y = std::stod(vertexNode->first_attribute("y")->value());
                    Vector2D vertex(x, y);
                    vertexes.push_back(vertex);
                }
                Polygon polygon(vertexes);
                if (polygon.points.size()>0){

                    polygons.push_back(polygon);
                }
            }
        }
    }
    catch (rapidxml::parse_error& e) 
    {
        std::cerr << "Failed to parse XML: " << e.what() << std::endl;
        return false;
    }

    return true;
}