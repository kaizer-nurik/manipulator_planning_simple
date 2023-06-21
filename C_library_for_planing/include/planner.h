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

double calculateDistance(const Vector2D& a, const Vector2D& b) {
    return std::abs(a.x - b.x) + std::abs(a.y - b.y);
}

struct Node
{
    std::vector<double> position;
    double gCost;
    double hCost;
    std::shared_ptr<Node> parent;

    Node(const std::vector<double>& pos, double g, double h, const std::shared_ptr<Node> &p) : position(pos), gCost(g), hCost(h), parent(p) {}

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
            hash ^= std::hash<double>()(value);
        }
        return hash;
    }
};

// Define an equality function for the shared pointers to nodes
struct EqualNode {
    bool operator()(const std::shared_ptr<Node>& node1, const std::shared_ptr<Node>& node2) const {
        double tolerance = 1e-6;
        double diff;
        for (auto i = 0u; i < node1->position.size(); i++)
            diff += std::abs(node1->position[i] - node1->position[i]);
        return diff <= tolerance;
    }
};

struct CompareKey {
    bool operator()(const std::vector<double>& key1, const std::vector<double>& key2) const {
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

class Planner {
public:
    Planner(std::string filename) : filename_(filename) {}

    bool AStar(const Robot& robot, const Vector2D& goal, const std::vector<Polygon>& obstacles)
    {
        const int g_units = 128;
        std::vector<double> deltas(robot.dof_, 0.0);
        std::vector<double> config = robot.configuration;

        for (auto i = 0u; i < robot.configuration.size(); i++)
        {
            deltas[i] = (robot.joints[i].limits[0] - robot.joints[i].limits[0]) / g_units;
        }
        std::vector<std::vector<double>> primitivemoves;

        for (auto i = 0u; i < robot.dof_; i++)
        {
            std::vector<double> a(robot.dof_, 0.0);
            a[i] = deltas[i];
            primitivemoves.push_back(a);
            a[i] = -deltas[i];
            primitivemoves.push_back(a);
        }

        std::priority_queue <std::shared_ptr<Node>, std::vector<std::shared_ptr<Node>>,  CompareNodes> opened_nodes;
        std::map<std::vector<double>, std::shared_ptr<Node>, CompareKey> map_pq_opened;
        std::unordered_set<std::shared_ptr<Node>, HashNode, EqualNode> closed_nodes;

        Vector2D current = end_effector(robot, config);

        std::shared_ptr<Node> start = std::make_shared<Node>(config, 0.0, calculateDistance(current, goal), nullptr);
        opened_nodes.push(start);
        map_pq_opened.emplace(start->position, start);

        while (!opened_nodes.empty())
        {
            std::shared_ptr<Node> node = opened_nodes.top();
            
            if (current == goal)
            {
                std::vector<std::shared_ptr<Node>> path;
                while (node != nullptr)
                 {
                    path.push_back(node);
                    node = node->parent;
                }
                std::reverse(path.begin(), path.end());
                 
                 std::ofstream file(filename_);
                // Write vectors to the CSV file
                for (const auto& n : path) {
                    for (size_t i = 0; i < n->position.size(); ++i) {
                        file << n->position[i];
                        if (i != n->position.size() - 1) {
                            file << ",";
                        }
                    }
                    file << std::endl;
                }

                file.close();
                std::cout << "Vectors successfully written to the file: " << filename_ << std::endl;
                return true;
            }
            opened_nodes.pop();

            closed_nodes.insert(node);
            for (const auto &i:primitivemoves)
            {
                std::shared_ptr<Node> newneighbour = std::make_shared<Node>(config+i, node->gCost+1.0, 0.0, node);
                current = end_effector(robot, config);
                newneighbour->hCost = calculateDistance(current, goal);
                
                if (closed_nodes.count(newneighbour) > 0 )
                {
                    continue;
                }

                if (! collide(robot, config, obstacles))
                {
                    double tentative_g = node->gCost + std::abs(node->gCost - newneighbour->gCost);
                    bool tentative_is_better = false;
                    auto it = map_pq_opened.find(newneighbour->position);
                    if (it == map_pq_opened.end()) 
                    {
                        //not found
                        map_pq_opened.emplace(newneighbour->position, newneighbour);
                        opened_nodes.push(newneighbour);
                        tentative_is_better = true;
                    } 
                    else 
                    {
                        if (tentative_g < newneighbour->gCost)
                            tentative_is_better=true;
                    }
                    if (tentative_is_better)
                    {
                        newneighbour->parent = node;
                        newneighbour->gCost = tentative_g;
                        newneighbour->hCost = calculateDistance(current, goal);
                    }
                }
            }
        }

        
    return false;

    
        // std::ofstream myfile;
        // myfile.open (filename_);

        // for (int i=0; i<start.get_joints().size(); i++)
        // {
        //     while (std::abs(current.joints[i].angle - goal.joints[i].angle)>eps)
        //     {
        //         current.joints[i].angle -= eps * ((current.joints[i].angle - goal.joints[i].angle > 0.0) 
        //         - (current.joints[i].angle - goal.joints[i].angle < 0));
        //         std::cout << current.joints[i].angle << ' ' << goal.joints[i].angle << std::endl;

        //     for (int i=0; i<current.get_joints().size(); i++)
        //     {
        //         myfile <<  current.joints[i].angle << ',';
        //     }
        //     myfile << '\n';
        //     }

        //     std::cout << distance(current, goal) << std::endl;

        //}
       // myfile.close();

    }

    void RRT(const Robot& start, const Robot& goal, const std::vector<Polygon>& obstacles)
    {

    }
private:
    std::string filename_;
    const double eps = 1e-3;
};

