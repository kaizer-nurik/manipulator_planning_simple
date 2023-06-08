#pragma once
#include <iostream>
#include <fstream>
#include <string>
#include <memory>
#include <vector>

#include "rapidxml.hpp"
#include "obstacles.h"
#include "robot.h"


void read_scene(const std::string &filename, std::vector<std::shared_ptr<Obstacle>> &obstacles, Robot &start, Robot &goal)
{
    // Load the XML file
    std::ifstream file(filename);
    if (!file) {
        std::cerr << "Failed to open XML file." << std::endl;
        return;
    }

    // Read the file into a string buffer
    std::string xmlString((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    xmlString.push_back('\0'); // Null-terminate the string for RapidXML

    try {
        // Parse the XML
        rapidxml::xml_document<> doc;
        doc.parse<0>(&xmlString[0]);

        // Access the root node
        rapidxml::xml_node<>* rootNode = doc.first_node("robot");
        if (!rootNode) {
            std::cerr << "Invalid XML format. Root node not found." << std::endl;
            return;
        }

        // Parse the start manipulator information
        rapidxml::xml_node<>* startNode = rootNode->first_node("start");
        if (startNode) {
            // Read degrees of freedom
            rapidxml::xml_node<>* dofNode = startNode->first_node("degrees_of_freedom");
            if (dofNode) {
                int dof = std::stoi(dofNode->value());
                std::cout << "Degrees of Freedom: " << dof << std::endl;
            }

            // Read joints
            rapidxml::xml_node<>* linksNode = startNode->first_node("joints");
            if (linksNode) {
                for (rapidxml::xml_node<>* linkNode = linksNode->first_node("joint"); linkNode; linkNode = linkNode->next_sibling("joint")) {
                    
                    double length = std::stod(linkNode->first_node("length")->value());
                    std::cout << "Link Length: " << length << std::endl;

                    double angle = std::stod(linkNode->first_node("angle")->value());
                    std::cout << "angle: " << angle << std::endl;

                    double limit1 = std::stod(linkNode->first_node("limit1")->value());
                    std::cout << "limit1: " << limit1 << std::endl;

                    double limit2 = std::stod(linkNode->first_node("limit2")->value());
                    std::cout << "limit1: " << limit2 << std::endl;
                    start.AddJoint(angle, length, { limit1, limit2 });
                }
            }
        }

        rapidxml::xml_node<>* goalNode = rootNode->first_node("goal");
        if (goalNode) {
            // Read degrees of freedom
            rapidxml::xml_node<>* dofNode = goalNode->first_node("degrees_of_freedom");
            if (dofNode) {
                int dof = std::stoi(dofNode->value());
                std::cout << "Degrees of Freedom: " << dof << std::endl;
            }

            // Read joints
            rapidxml::xml_node<>* linksNode = goalNode->first_node("joints");
            if (linksNode) {
                for (rapidxml::xml_node<>* linkNode = linksNode->first_node("joint"); linkNode; linkNode = linkNode->next_sibling("joint")) {
                    
                    double length = std::stod(linkNode->first_node("length")->value());
                    std::cout << "Link Length: " << length << std::endl;

                    double angle = std::stod(linkNode->first_node("angle")->value());
                    std::cout << "angle: " << angle << std::endl;

                    double limit1 = std::stod(linkNode->first_node("limit1")->value());
                    std::cout << "limit1: " << limit1 << std::endl;

                    double limit2 = std::stod(linkNode->first_node("limit2")->value());
                    std::cout << "limit1: " << limit2 << std::endl;
                    goal.AddJoint(angle, length, { limit1, limit2 });
                }
            }
        }

        // Parse the scene obstacles
        rapidxml::xml_node<>* sceneNode = rootNode->first_node("scene");
        if (sceneNode) {
            for (rapidxml::xml_node<>* obstacleNode = sceneNode->first_node("obstacle"); obstacleNode; obstacleNode = obstacleNode->next_sibling("obstacle")) {
                std::string obstacleType = obstacleNode->first_attribute("type")->value();

                if (obstacleType == "line") {
                    double startX = std::stod(obstacleNode->first_node("start_x")->value());
                    double startY = std::stod(obstacleNode->first_node("start_y")->value());
                    double endX = std::stod(obstacleNode->first_node("end_x")->value());
                    double endY = std::stod(obstacleNode->first_node("end_y")->value());

                    std::cout << "Line Obstacle: (" << startX << ", " << startY << ") to (" << endX << ", " << endY << ")" << std::endl;
                    std::shared_ptr<Obstacle> p = std::make_shared<Line>(startX, startY, endX, endY);
                    obstacles.push_back(p);
                }
                else if (obstacleType == "circle") {
                    double centerX = std::stod(obstacleNode->first_node("center_x")->value());
                    double centerY = std::stod(obstacleNode->first_node("center_y")->value());
                    double radius = std::stod(obstacleNode->first_node("radius")->value());

                    std::cout << "Circle Obstacle: Center (" << centerX << ", " << centerY << "), Radius: " << radius << std::endl;

                    std::shared_ptr<Obstacle> p = std::make_shared<Circle>(centerX, centerY, radius);
                    obstacles.push_back(p);
                }
                else if (obstacleType == "square") {
                    double x1 = std::stod(obstacleNode->first_node("x1")->value());
                    double y1 = std::stod(obstacleNode->first_node("y1")->value());

                    double x2 = std::stod(obstacleNode->first_node("x2")->value());
                    double y2 = std::stod(obstacleNode->first_node("y2")->value());

                    double x3 = std::stod(obstacleNode->first_node("x3")->value());
                    double y3 = std::stod(obstacleNode->first_node("y3")->value());

                    double x4 = std::stod(obstacleNode->first_node("x4")->value());
                    double y4 = std::stod(obstacleNode->first_node("y4")->value());
                    

                    std::cout << "Square Obstacle: " << x1 << ' ' << y1 << ' ' << x2 << ' ' << y2 << ' ' << x3 << ' ' << y3 << ' '
                     << x4 << ' ' << y4 << ' '  << std::endl;
                    
                    std::shared_ptr<Obstacle> p = std::make_shared<Square>(x1, y1, x2, y2, x3, y3, x4, y4);
                    obstacles.push_back(p);
                }
            }
        }
    }
    catch (rapidxml::parse_error& e) {
        std::cerr << "Failed to parse XML: " << e.what() << std::endl;
        return;
    }
}