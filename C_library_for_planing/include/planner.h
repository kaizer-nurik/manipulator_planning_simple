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

void simplify(std::vector<int> & v, int n)
{
    for (auto i=0u; i<v.size(); i++)
    {
        v[i] = v[i]%n;
    }
}

double calculateDistance(const Vector2D& a, const Vector2D& b) {
    return std::abs(a.x - b.x) + std::abs(a.y - b.y);
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

std::vector<double> calc_angles(const Robot &robot, const std::vector<int> &position, const std::vector<double> &deltas)
{
    std::vector<double> angles(robot.dof_, 0.0);
    for (int j=0; j<angles.size(); j++)
    {
        angles[j] = robot.configuration[j] + position[j]*deltas[j];
    }
    return angles;
}

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
    std::cout << "nnode\n";
    std::reverse(path.begin(), path.end());    
    std::ofstream file(filename);
    // Write vectors to the CSV file
    for (const auto& n : path)
    {
        for (size_t i = 0; i < n->position.size(); ++i)
        {
            file << robot.configuration[i] + n->position[i]*deltas[i];
            if (i != n->position.size() - 1) {file << ",";}  
        }
        file << std::endl;
    }
    file.close();
    std::cout << "Vectors successfully written to the file: " << filename << std::endl;
}


class Planner 
{
public:
    Planner(std::string filename) : filename_(filename) {}

    bool AStar(const Robot& robot, const Vector2D& goal, const std::vector<Polygon>& obstacles)
    {
        const int g_units = 220;
        std::vector<double> deltas(robot.dof_, 0.0);
        for (auto i = 0u; i < robot.configuration.size(); i++)
        {
            deltas[i] = std::abs(robot.joints[i].limits[1] - robot.joints[i].limits[0]) / g_units;
        }

        print_vector(deltas);

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

        std::shared_ptr<Node> start = std::make_shared<Node>(config, 0.0, calculateDistance(end_effector(robot, angles), goal), nullptr);
        opened_nodes.push(start);
        map_pq_opened.emplace(start->position, start);

        std::cout << "BIG ALARM:" << end_effector(robot, angles).x << ' ' << end_effector(robot, angles).y << std::endl;

        while (!opened_nodes.empty())
        {
            std::shared_ptr<Node> current = opened_nodes.top();
            opened_nodes.pop();
            size_t numErased = map_pq_opened.erase(current->position);
            closed_nodes.insert(current);
            std::cout << current->gCost << ' ' << current->hCost << ' ' << current->getFCost() << std::endl;
            
            if (std::abs(current->hCost) < 1e-1 )
            {
                reconstruct_path(current,filename_, robot, deltas);
                angles = calc_angles(robot, current->position, deltas);
                std::cout << "Пришел в: " << end_effector(robot, angles).x << ' ' << end_effector(robot, angles).x << std::endl;
                return true;
            }

            for (const auto &i:primitivemoves)
            {
                std::shared_ptr<Node> newneighbour = std::make_shared<Node>(current->position+i, current->gCost+1.0, 0.0, current);
                simplify(newneighbour->position, g_units);
                angles=calc_angles(robot, newneighbour->position, deltas);
                newneighbour->hCost = calculateDistance(end_effector(robot, angles), goal);
                if (collide(robot, angles, obstacles))
                {
                    closed_nodes.insert(newneighbour);
                }
                if (closed_nodes.count(newneighbour) > 0) //if in closed list
                {
                    continue;
                }
                double g_score = current->gCost + 1.0;

                if (map_pq_opened.find(newneighbour->position) == map_pq_opened.end() || g_score < newneighbour->gCost) //Neighbour not in opened
                {
                    newneighbour->gCost = g_score;
                    newneighbour->hCost = calculateDistance(end_effector(robot, angles), goal);
                    newneighbour->parent = current;
                    if (map_pq_opened.find(newneighbour->position) == map_pq_opened.end())
                    {
                        map_pq_opened.emplace(newneighbour->position, newneighbour);
                        opened_nodes.push(newneighbour);
                    }
                }
            }
            // if (opened_nodes.size()==1)
            // {
            //     std::cout << "1\n";
            //     reconstruct_path(current, filename_, robot, deltas);
            //     std::cout << "2\n";
            // }
            

        }        
    return false;
    }

    void RRT(const Robot& start, const Robot& goal, const std::vector<Polygon>& obstacles)
    {

    }
private:
    std::string filename_;
    const double eps = 1e-3;
};

