#pragma once
#include "obstacles.h"
#include "robot.h"
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

struct CompareNodes {
    bool operator()(const std::shared_ptr<Node>& node1, const std::shared_ptr<Node>& node2) const {
        // Higher priority nodes should have lower priority values
        return node1->getFCost() > node2->getFCost();
    }
};

struct HashNode {
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
struct EqualNode {
    bool operator()(const std::shared_ptr<Node>& node1, const std::shared_ptr<Node>& node2) const {
        bool eq = true;
        for (auto i = 0u; i < node1->position.size(); i++)
            eq *= (node1->position[i] == node1->position[i]);
        return eq;
    }
};

struct CompareKey {
    bool operator()(const std::vector<int>& key1, const std::vector<int>& key2) const {
        // Compare the sizes of the vectors
        if (key1.size() != key2.size()) {
            return key1.size() < key2.size();
        }

        // Compare the elements of the vectors element-wise
        for (size_t i = 0; i < key1.size(); ++i) {
            if (key1[i] != key2[i]) {
                return key1[i] < key2[i];
            }
        }

        return false; // Both keys are equal
    }
};

template<typename T>
void print_vector(std::vector<T> v)
{
    for (auto i:v)
        std::cout << i << ' ';
    std::cout << std::endl;
}

void reconstruct_path(const std::shared_ptr<Node> &node, std::string filename, const Robot &robot, const std::vector<double> &deltas)
{
    std::vector<std::shared_ptr<Node>> path;
    auto node_ = node;
    while (node_ != nullptr)
    {
        path.push_back(node_);
        node_ = node_->parent;
    }
    

    std::reverse(path.begin(), path.end());    
    std::ofstream file(filename);
    // Write vectors to the CSV file
    for (const auto& n : path)
    {
        for (size_t i = 0; i < n->position.size(); ++i)
        {
            file << (robot.configuration[i] + n->position[i]*deltas[i]) * 180.0 / std::acos(-1);
            if (i != n->position.size() - 1) {file << ",";}  
        }
        file << std::endl;
    }
    file.close();
    std::cout << "Vectors successfully written to the file: " << filename << std::endl;
}


void simplify(std::vector<int> & v, int n)
{
    for (auto i=0u; i<v.size(); i++)
    {
        v[i] = v[i] % (n+1);
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
        angle+=angles[i];
    return fmod(angle, std::acos(-1));
}

class Planner 
{
public:
    Planner(std::string filename) : filename_(filename) {}

    bool AStar(const Robot& robot, const GoalPoint& goalpoint, const std::vector<Polygon>& obstacles)
    {
        Vector2D goal = goalpoint.goalpoint;
        double x_star = goal.x + robot.joints[robot.dof_-1].length * cos(goalpoint.angle1_); // x_goal for penultimate joint
        double y_star = goal.y + robot.joints[robot.dof_-1].length * sin(goalpoint.angle1_); // x_goal for penultimate joint
        Vector2D penultimate_goal(x_star, y_star);

        const int g_units = 110; // mesh fineness [-angle1, angle2] -- 2g_units
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
        std::priority_queue <std::shared_ptr<Node>, std::vector<std::shared_ptr<Node>>,  CompareNodes> opened_nodes;
        std::map<std::vector<int>, std::shared_ptr<Node>, CompareKey> map_pq_opened;
        std::unordered_set<std::shared_ptr<Node>, HashNode, EqualNode> closed_nodes;

        std::vector<int> config(robot.dof_, 0);
        std::vector<double> angles = calc_angles(robot, config, deltas); //std::abs(fmod(angles[angles.size()-1], std::acos(-1)) - goalpoint.angle1_) 

        double h = std::abs(fmod(angles[angles.size()-1], std::acos(-1)) - goalpoint.angle1_);
        std::shared_ptr<Node> start = std::make_shared<Node>(config, 0.0, calculateDistance(end_effector(robot, angles), goal)+ h , nullptr);
        opened_nodes.push(start);
        map_pq_opened.emplace(start->position, start);
        Vector2D startPoint = end_effector(robot,angles);


        while (!opened_nodes.empty())
        {
            std::shared_ptr<Node> current = opened_nodes.top();
            opened_nodes.pop();
            size_t numErased = map_pq_opened.erase(current->position);
            closed_nodes.insert(current);
            auto currxy = end_effector(robot, calc_angles(robot, current->position, deltas));
            std::cout << current->gCost << ' ' << current->hCost  << ' ' << calculateDistance(currxy, goal) << ' ' << std::abs(fmod(angles[angles.size()-1], std::acos(-1)) - goalpoint.angle1_) <<  std::endl;
            angles = calc_angles(robot, current->position, deltas);
            if (std::abs(current->hCost) < 1e-1 && std::abs(fmod(angles[angles.size()-1], std::acos(-1)) - goalpoint.angle1_)<goalpoint.angle2_)
            {
                reconstruct_path(current,filename_, robot, deltas);
                angles = calc_angles(robot, current->position, deltas);
                std::cout << "Пришел в: " << end_effector(robot, angles).x << ' ' << end_effector(robot, angles).y << " angle: "<< angles.back()*180/std::acos(-1) << std::endl;
                return true;
            }

            for (const auto &i:primitivemoves)
            {
                std::shared_ptr<Node> newneighbour = std::make_shared<Node>(current->position+i, 0.0, 0.0, current);
                //simplify(newneighbour->position, g_units);
                angles = calc_angles(robot, newneighbour->position, deltas);
                newneighbour->hCost = calculateDistance(end_effector(robot, angles), goal) + std::abs(fmod(angles[angles.size()-1], std::acos(-1)) - goalpoint.angle1_);
                if  (collide(robot, angles, obstacles))
                {
                    //closed_nodes.insert(newneighbour);
                    std::cout << "collision" << std::endl;
                    continue;
                }
                if (closed_nodes.count(newneighbour) > 0) //if in closed list
                {
                    continue;
                }
                double g_score = current->gCost + 1.0;//calculateDistance2(end_effector(robot, calc_angles(robot, current->position, deltas)), end_effector(robot, angles)); //tentative
                auto it = map_pq_opened.find(newneighbour->position);
                if (it == map_pq_opened.end()) //Neighbour not in opened
                {
                    newneighbour->hCost = calculateDistance(end_effector(robot, angles), goal) + std::abs(fmod(angles[angles.size()-1], std::acos(-1)) - goalpoint.angle1_); //+ fmod( std::abs(last_angle(angles) - goalpoint.angle1_), 2*std::acos(-1));
                    newneighbour->gCost = g_score;
                    newneighbour->parent = current;
                    map_pq_opened.emplace(newneighbour->position, newneighbour);
                    opened_nodes.push(newneighbour);   
                }
                else if (g_score < it->second->gCost) // optimization
                {
                    it->second->gCost = g_score;
                    it->second->hCost = calculateDistance(end_effector(robot, angles), goal) + std::abs(fmod(angles[angles.size()-1], std::acos(-1)) - goalpoint.angle1_); //+ fmod( std::abs(last_angle(angles) - goalpoint.angle1_), 2*std::acos(-1));
                    it->second->parent = current;
                }
            }
        }        
    return false;
    }

    bool coll_test(const Robot& robot, const std::vector<Polygon>& obstacles)
    {
        const int g_units = 110; // mesh fineness [-angle1, angle2] -- 2g_units
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
        std::ofstream file(filename_);

        int n = 0;
        while (true)
        {
            n++;
            if (n==200) break;
            config = config + primitivemoves[0];
            angles = calc_angles(robot, config, deltas);
            if (collide(robot, angles, obstacles))
            {
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
    std::string filename_;
    const double eps = 1e-3;
};

