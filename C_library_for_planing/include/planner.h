#pragma once
#include "obstacles.h"
#include "robot.h"
#include <string>
#include <unordered_set>
#include <queue>
#include <map>
#include <memory>
#include <vector>
#include <iostream>
#include <cmath>
#include <utility>
#include <algorithm>
#include <fstream>
#include "benchmark.h"
#include "../json-develop/include/nlohmann/json.hpp"

template<typename T>
std::vector<T> operator+(const std::vector<T>& vec1, const std::vector<T>& vec2) {
    std::vector<T> result;
    if (vec1.size() != vec2.size()) {
        std::cout << "Ошибка: векторы имеют разные размеры!" << std::endl;
        return result;
    }
    for (std::size_t i = 0; i < vec1.size(); ++i) {
        result.push_back(vec1[i] + vec2[i]);
    }
    return result;
}

template<typename T>
std::vector<T> operator*(const std::vector<T>& vec, const T& scalar) {
    std::vector<T> result;
    for (const T& value : vec) {
        result.push_back(value * scalar);
    }

    return result;
}

struct Node
{
    std::vector<int> position;
    double gCost;
    double hCost;
    std::shared_ptr<Node> parent;

    Node(const std::vector<int>& pos, double g, double h, const std::shared_ptr<Node> &p) : position(pos), gCost(g), hCost(h), parent(p) {}

    double getFCost() const {
        return gCost + hCost;
    }
    bool operator<(const Node& other) const {
        return getFCost() < other.getFCost();
    }
    bool operator>(const Node& other) const {
        return getFCost() > other.getFCost();
    }
    bool operator==(const Node& other) const {
        double tolerance = 1e-6;
        double diff;
        for (auto i = 0u; i < position.size(); i++)
            diff += std::abs(position[i] - other.position[i]);
        return diff <= tolerance;
    }
};

struct CompareNodes
 {
    bool operator()(const std::shared_ptr<Node>& node1, const std::shared_ptr<Node>& node2) const {
        // Higher priority nodes should have lower priority values
        return node1->getFCost() > node2->getFCost();
    }
};

struct HashNode 
{
    size_t operator()(const std::shared_ptr<Node>& node) const {
        size_t hash = 0;
        for (const auto& value : node->position) {
            // Combine hash with each value in the vector
            hash ^= std::hash<int>()(value);
        }
        return hash;
    }
};

// Define an equality function for the shared pointers to nodes
struct EqualNode
{
    bool operator()(const std::shared_ptr<Node>& node1, const std::shared_ptr<Node>& node2) const
     {
         if (node1->position.size() != node2->position.size()) 
         {
            return false;
        }

        for (size_t i = 0; i < node1->position.size(); i++) 
        {
            if (node1->position[i] != node2->position[i]) 
            {
                return false;
            }
        }

    return true;
    }
};

struct CompareKey
 {
    bool operator()(const std::vector<int>& key1, const std::vector<int>& key2) const
     {
        if (key1.size() != key2.size())
         {
            return key1.size() < key2.size();
        }

        for (size_t i = 0; i < key1.size(); ++i)
         {
            if (key1[i] != key2[i]) {
                return key1[i] < key2[i];
            }
        }

        return false;
    }
};

template<typename T>
void print_vector(std::vector<T> v)
{
    for (auto i:v)
        std::cout << i << ' ';
    std::cout << std::endl;
}

void reconstruct_path(const std::shared_ptr<Node> &node, std::string input_filename, std::string output_filename, const Robot &robot, const std::vector<double> &deltas)
{
    std::vector<std::shared_ptr<Node>> path;
    auto node_ = node;
    std::string path_csv;
    while (node_ != nullptr)
    {
        path.push_back(node_);
        node_ = node_->parent;
    }    

    std::reverse(path.begin(), path.end());    
    std::ofstream file("trajectory.csv"); // куда запишештся csv
    // Write vectors to the CSV file
    for (const auto& n : path)
    {
        for (size_t i = 0; i < n->position.size(); ++i)
        {
            file << (robot.configuration[i] + n->position[i]*deltas[i]) * 180.0 / std::acos(-1);
            path_csv.append(std::to_string((robot.configuration[i] + n->position[i]*deltas[i]) * 180.0 / std::acos(-1)));
            if (i != n->position.size() - 1) {file << ","; path_csv.append(",");}  
        }
        file << std::endl;
        path_csv.append("\n");
    }
    file.close();

    std::ifstream inputFile(input_filename);
    std::ofstream outputFile(output_filename);

    if (inputFile.is_open() && outputFile.is_open())
    {
        std::string line;
        while (std::getline(inputFile, line)) 
        { 
            if (line == "</input_info>") continue;
            outputFile << line << std::endl;
        }
        outputFile << "<csv>" << path_csv <<"</csv>";
        outputFile << "</input_info>" << std::endl;
        inputFile.close();  
        outputFile.close(); 
    }
    else { std::cerr << "Error." << std::endl;}
}

double periodic(double n, double L)
{
    if (n>=0.0) return fmod(n + L, 2*L) - L;
    else return fmod(n + L, 2*L) + L;
}

void simplify(std::vector<int> & v, int n)
{
    for (auto i=0u; i<v.size(); i++)
    {
        if (v[i] >= 0)  v[i] = (v[i] + n) % (2 * n) - n;
        if (v[i] <=-n)  v[i] = (v[i] + n) % (2 * n) + n;
    }
}

double calculateDistance(const Vector2D& a, const Vector2D& b) {
    return std::abs(a.x - b.x) + std::abs(a.y - b.y);
}

double calculateDistance2(const Vector2D& a, const Vector2D& b) {
    return std::sqrt ( (a.x - b.x)*(a.x - b.x) + (a.y - b.y) * (a.y - b.y) );
}


std::vector<double> calc_angles(const Robot &robot, const std::vector<int> &position, const std::vector<double> &deltas)
{
    std::vector<double> angles(robot.dof_, 0.0);
    for (int j=0; j<angles.size(); j++)
    {
        angles[j] = robot.configuration[j] + position[j]*deltas[j];
    }
    return angles;
}

double last_angle(const std::vector<double> angles)
{
    double angle=0.0;
    for (int i=0; i<angles.size(); i++)
       { angle+=angles[i];
       }
    return periodic(angle, std::acos(-1));
}

void generateVectors(std::vector<std::vector<int>>& result, std::vector<int>& current, int index) 
/*
    vector<vector<int>> vectors;
    vector<int> current(3);
    generateVectors(vectors, current, 0);
*/
{
    if (index == current.size()) {
        result.push_back(current);
        return;
    }

    for (int i = -1; i <= 1; i++) {
        current[index] = i;
        generateVectors(result, current, index + 1);
    }
}


void writeDataToJson(int test,  int g_units, double coord_tolerance, double angle_tolerance, double time, double coll_check_percentage, int opened_nodes, int closed_nodes, double g_cost, int turn_numbers,  const std::string& filename) {
    nlohmann::json j;
    j["_number"] = test;
    j["g_units"] = g_units;
    j["coord_tolerance"] = coord_tolerance;
    j["total_time"] = time;
    j["coll_check_percentage"] = coll_check_percentage;
    j["opened_nodes"] = opened_nodes;
    j["closed_nodes"] = closed_nodes;
    j["g_cost"] = g_cost;
    j["turn_number"] = turn_numbers;

    std::ofstream file(filename, std::ios::app);

    if (file.is_open()) {
        file << j.dump(4);  // Use dump(4) for indentation
        //std::cout << "Data has been written to the file." << std::endl;
        file.close();
    } else {
        std::cout << "Unable to open the file." << std::endl;
    }
}

class Planner 
{
public:
    Planner(std::string input_filename, std::string output_filename) : input_filename_(input_filename), output_filename_(output_filename) {}

    double heuristic(const Robot &robot, const GoalPoint &goalpoint, const std::vector<double> &angles)
    {
        return calculateDistance(end_effector(robot, angles), goalpoint.goalpoint);// + 0.8 *std::abs(fmod(angles[angles.size()-1], std::acos(-1)) - goalpoint.angle1_);
    }
    double geuristic(const Robot &robot, const std::vector<double> &angles1, const std::vector<double> &angles2)
    {
        return calculateDistance2(end_effector(robot, angles1), end_effector(robot, angles2));
    }

    
    bool AStar(const Robot& robot, const GoalPoint& goalpoint, const std::vector<Polygon>& obstacles, std::map<std::string, double> &dictionary)
    {
        Timer t("a-star");
        Timer tt("coll_check");
        double time=0.0;

        Vector2D goal = goalpoint.goalpoint;
        
        const int g_units = 15; // mesh fineness [-angle1, angle2] -- 2g_units
        double coord_tolerance = 0.1;
        std::vector<double> deltas(robot.dof_, 0.0);

        for (auto i = 0u; i < robot.configuration.size(); i++)
        {
            deltas[i] = std::abs(robot.joints[i].limits[1] - robot.joints[i].limits[0]) / (2*g_units);
        }
        // std::vector<std::vector<int>> primitivemoves;
        // std::vector<int> current(robot.dof_);
        // generateVectors(primitivemoves, current, 0);

        std::vector<std::vector<int>> primitivemoves;
        for (auto i = 0u; i < robot.dof_; i++)
        {
            std::vector<int> a(robot.dof_, 0.0);
            a[i] = 1;
            primitivemoves.push_back(a);
            a[i] = -1;
            primitivemoves.push_back(a);            
        }

        
        std::priority_queue <std::shared_ptr<Node>, std::vector<std::shared_ptr<Node>>,  CompareNodes> opened_nodes;
        std::map<std::vector<int>, std::shared_ptr<Node>, CompareKey> map_pq_opened;
        std::unordered_set<std::shared_ptr<Node>, HashNode, EqualNode> closed_nodes;

        std::vector<int> config(robot.dof_, 0);
        std::vector<double> angles = calc_angles(robot, config, deltas); 

        std::shared_ptr<Node> start = std::make_shared<Node>(config, 0.0, heuristic(robot, goalpoint, angles) , nullptr);
        opened_nodes.push(start);
        map_pq_opened.emplace(start->position, start);
        Vector2D startPoint = end_effector(robot,angles);
        int NODES = 0;
        int Nodes_closed = 0;
        int n_turns = 0;


        while (!opened_nodes.empty())
        {
            std::shared_ptr<Node> current = opened_nodes.top();
            opened_nodes.pop();
            map_pq_opened.erase(current->position);

            closed_nodes.insert(current);
            Nodes_closed++;
            auto currxy = end_effector(robot, calc_angles(robot, current->position, deltas));

            //std::cout << current->getFCost() << ' ' << current->gCost << ' ' << current->hCost  << std::endl;
            std::cout << opened_nodes.size() << ' ' << map_pq_opened.size() << ' ' << closed_nodes.size() << std::endl;
            if (opened_nodes.size()>50'000) return false;
            
            angles = calc_angles(robot, current->position, deltas);
            
            //std:: cout << angles[0]*180/std::acos(-1) << ',' << angles[1]*180/std::acos(-1) << ',' << angles[2]*180/std::acos(-1) << std::endl;
            std::vector<double> curr_angles = calc_angles(robot, current->position, deltas);
            
            if (std::abs(current->hCost) < coord_tolerance) // && std::abs(last_angle(angles) - goalpoint.angle1_)<goalpoint.angle2_)
            {
                reconstruct_path(current,input_filename_, output_filename_, robot, deltas);
                angles = calc_angles(robot, current->position, deltas);
                std::cout << "Пришел в: " << end_effector(robot, angles).x << ' ' << end_effector(robot, angles).y << 
                " angle: "<< last_angle(angles) << "  "<< angles.back()*180/std::acos(-1) << " opened nodes: " << NODES << std::endl;

                dictionary["g_units"] = g_units;
                dictionary["coord_tolerance"] = coord_tolerance;
                dictionary["angle_tolerance"] = goalpoint.angle2_;
                dictionary["time"] = t.getElapsedTime();
                dictionary["coll_check_percentage"] = time/t.getElapsedTime() * 100;
                dictionary["opened_nodes"] = NODES;
                dictionary["closed_nodes"] = Nodes_closed;
                dictionary["g_cost"] = current->gCost;
                dictionary["turn_numbers"] = n_turns;
            
                return true;
            }
            n_turns++;
            for (const auto &i:primitivemoves)
            {
                std::shared_ptr<Node> newneighbour = std::make_shared<Node>(current->position+i, 0.0, 0.0, current);
                simplify(newneighbour->position, g_units);
                angles = calc_angles(robot, newneighbour->position, deltas);
                newneighbour->hCost = heuristic(robot, goalpoint, angles);

                tt.start();
                bool flag = collide(robot, angles, obstacles);
                time+=tt.getElapsedTime();
                tt.pause();
                if  (flag)
                {
                    //closed_nodes.insert(newneighbour);
                    //std::cout << "collision" << std::endl;
                    continue;
                }
                if (closed_nodes.count(newneighbour) != 0) //if in closed list
                {
                    continue;
                }

                double g_score = current->gCost + geuristic(robot,  angles, curr_angles);   //tentative
                auto it = map_pq_opened.find(newneighbour->position);


                if (it == map_pq_opened.end()) //Neighbour not in opened => add neighbor
                {
                    newneighbour->hCost = heuristic(robot, goalpoint, angles);
                    newneighbour->gCost = g_score;
                    newneighbour->parent = current;
                    map_pq_opened.emplace(newneighbour->position, newneighbour);
                    opened_nodes.push(newneighbour);
                    NODES++;   
                }
                if (g_score < it->second->gCost && it != map_pq_opened.end()) // optimization
                {
                    it->second->gCost = g_score;
                    it->second->hCost = heuristic(robot, goalpoint, angles);
                    it->second->parent = current;
                }
            }
        }  
    std::cout << "OPENED NODES " << NODES << std::endl;      
    return false;
    }



    bool coll_test(const Robot& robot, const std::vector<Polygon>& obstacles)
    {
        const int g_units = 110; 
        std::vector<double> deltas(robot.dof_, 0.0);
        for (auto i = 0u; i < robot.configuration.size(); i++)
        {
            deltas[i] = std::abs(robot.joints[i].limits[1] - robot.joints[i].limits[0]) / (2*g_units);
        }
        std::vector<std::vector<int>> primitivemoves;
        for (auto i = 0u; i < robot.dof_; i++)
        {
            std::vector<int> a(robot.dof_, 0.0);
            a[i] = 1;
            primitivemoves.push_back(a);
            a[i] = -1;
            primitivemoves.push_back(a);            
        }

        std::vector<int> config(robot.dof_, 0);
        std::vector<double> angles = calc_angles(robot, config, deltas);
        
        std::ofstream file(output_filename_);

        int n = 0;
        while (true)
        {
            
            n++;
            if (n==500) break;
            config = config + primitivemoves[0];
            angles = calc_angles(robot, config, deltas);
            std::cout << 123 << std::endl;
            if (collide(robot, angles, obstacles))
            {
                std::cout << 123 << std::endl;
                std::cout << "collision" << std::endl;
                primitivemoves[0][0] *= -1;
                //int a;
                //std::cin >> a;
            }
    
        for (size_t i = 0; i < config.size(); ++i)
        {
            file << (robot.configuration[i] + config[i]*deltas[i]) * 180.0 / std::acos(-1);
            if (i != config.size() - 1) {file << ",";}  
        }
        file << std::endl;
        }        
        file.close();
        return false;
    }


    void RRT(const Robot& start, const Robot& goal, const std::vector<Polygon>& obstacles)
    {

    }
private:
    std::string input_filename_;
    std::string output_filename_;
    const double eps = 1e-3;
};

