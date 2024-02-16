#include "planner.h"

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
#include "json.hpp"
#include <list>
#include <unordered_map>

void writeDataToJson(int test, int g_units, double coord_tolerance, double angle_tolerance, double time, double coll_check_percentage, int opened_nodes, int closed_nodes, double g_cost, int turn_numbers, const std::string &filename)
{
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

    if (file.is_open())
    {
        file << j.dump(4); // Use dump(4) for indentation
        // std::cout << "Data has been written to the file." << std::endl;
        file.close();
    }
    else
    {
        std::cout << "Unable to open the file." << std::endl;
    }
}

namespace
{
    double Node::getFCost() const
    {
        return gCost + hCost;
    }
    bool Node::operator<(const Node &other) const
    {
        return getFCost() < other.getFCost();
    }
    bool Node::operator>(const Node &other) const
    {
        return getFCost() > other.getFCost();
    }
    bool Node::operator==(const Node &other) const
    {
        double tolerance = 1e-6;
        double diff;
        for (auto i = 0u; i < position.size(); i++)
            diff += std::abs(position[i] - other.position[i]);
        return diff <= tolerance;
    }

    bool CompareNodes::operator()(const Node::Nodetype &node1, const Node::Nodetype &node2) const
    {
        // Higher priority nodes should have lower priority values
        return node1->getFCost() > node2->getFCost();
    }

    size_t HashNode::operator()(const Node::Nodetype &node) const
    {
        size_t hash = 0;
        for (const auto &value : node->position)
        {
            // Combine hash with each value in the vector
            hash ^= std::hash<int>()(value);
        }
        return hash;
    }

    size_t HashNode1::operator()(const MathVector<int> &position) const
    {
        size_t hash = 0;
        for (const auto &value : position)
        {
            // Combine hash with each value in the vector
            hash ^= std::hash<int>()(value);
        }
        return hash;
    }

    bool EqualNode::operator()(const Node::Nodetype &node1, const Node::Nodetype &node2) const
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

    bool CompareKey::operator()(const MathVector<int> &key1, const MathVector<int> &key2) const
    {
        if (key1.size() != key2.size())
        {
            return key1.size() < key2.size();
        }

        for (size_t i = 0; i < key1.size(); ++i)
        {
            if (key1[i] != key2[i])
            {
                return key1[i] < key2[i];
            }
        }

        return false;
    }

    template <typename T>
    void print_vector(MathVector<T> v)
    {
        for (auto i : v)
            std::cout << i << ' ';
        std::cout << std::endl;
    }

    void reconstruct_path(const Node::Nodetype &node, std::string input_filename, std::string output_filename, const Robot &robot, const MathVector<double> &deltas)
    {
        MathVector<Node::Nodetype> path;
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
        for (const auto &n : path)
        {
            for (size_t i = 0; i < n->position.size(); ++i)
            {
                file << (robot.configuration[i] + n->position[i] * deltas[i]) * 180.0 / std::acos(-1);
                path_csv.append(std::to_string((robot.configuration[i] + n->position[i] * deltas[i]) ));
                if (i != n->position.size() - 1)
                {
                    file << ",";
                    path_csv.append(",");
                }
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
                if (line == "</input_info>")
                    continue;
                outputFile << line << std::endl;
            }
            outputFile << "<csv>" << path_csv << "</csv>";
            outputFile << "</input_info>" << std::endl;
            inputFile.close();
            outputFile.close();
        }
        else
        {
            std::cerr << "Error." << std::endl;
        }
    }

    double periodic(const double &n, const double &L)
    {
        if (n >= 0.0)
            return fmod(n + L, 2 * L) - L;
        else if (n >= -L && n < 0.0)
            return n;
        else
            return fmod(n + L, 2 * L) + L;
    }

    void simplify(MathVector<int> &v, int n)
    {
        for (auto i = 0u; i < v.size(); i++)
        {
            if (v[i] >= 0)
                v[i] = (v[i] + n) % (2 * n) - n;
            if (v[i] <= -n)
                v[i] = (v[i] + n) % (2 * n) + n;
        }
    }

    double calculateDistance(const Vector2D &a, const Vector2D &b)
    {
        return std::abs(a.x - b.x) + std::abs(a.y - b.y);
    }

    double calculateDistance2(const Vector2D &a, const Vector2D &b)
    {
        return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
    }

    MathVector<double> calc_angles(const Robot &robot, const MathVector<int> &position, const MathVector<double> &deltas)
    {
        MathVector<double> angles(robot.dof_, 0.0);
        for (int j = 0; j < angles.size(); j++)
        {
            angles[j] = robot.configuration[j] + position[j] * deltas[j];
        }
        return angles;
    }

    double last_angle(const MathVector<double> &angles)
    {
        double angle = 0.0;
        for (int i = 0; i < angles.size(); i++)
        {
            angle += angles[i];
        }
        return periodic(angle, 180);
    }

    void generateVectors(MathVector<MathVector<int>> &result, MathVector<int> &current, int index)
    /*
        vector<vector<int>> vectors;
        vector<int> current(3);
        generateVectors(vectors, current, 0);
    */
    {
        if (index == current.size())
        {
            result.push_back(current);
            return;
        }

        for (int i = -1; i <= 1; i++)
        {
            current[index] = i;
            generateVectors(result, current, index + 1);
        }
    }
    template <typename T>
    int signum(T number)
    {
        // static_assert(std::is_arithmetic<T>, "signum() requires an arithmetic type.");
        return (T(0) < number) - (number < T(0));
    }

    double norm1(const MathVector<double> &v1, const MathVector<double> &v2)
    {
        double n = 0.0;
        for (int i = 0; i < v1.size(); i++)
        {
            n += std::abs(v1[i] - v2[i]);
        }
        return n;
    }

    double norm0(const MathVector<double> &v1, const MathVector<double> &v2)
    {
        double max = 0.0;
        for (int i = 0; i < v1.size(); i++)
        {
            if (max < std::abs(v1[i] - v2[i]))
                max = std::abs(v1[i] - v2[i]);
        }
        return max;
    }

}

double Planner_A_star::heuristic(const Robot &robot, const GoalPoint &goalpoint, const MathVector<double> &angles, const int &weight_label)
{
    double g = calculateDistance(end_effector(robot, angles), goalpoint.goalpoint);
    //std::cout << last_angle(angles) << ' ' << goalpoint.angle1_<< ' ' << std::abs(periodic(last_angle(angles) - goalpoint.angle1_, 180)) << std::endl;
    if (weight_label == 2)
         return (g * 100 + 10*std::abs(periodic(last_angle(angles) - goalpoint.angle1_, 180)/(180)) / (0.1 + 10*g)); //123.5
    return (g * 10 + 10*std::abs(periodic(last_angle(angles) - goalpoint.angle1_, 180)/(180)) / (0.1 + 10*g)); //123.5
}
// double Planner_A_star::heuristic(const Robot &robot, const GoalPoint &goalpoint, const MathVector<double> &angles)
// {
//     double x = end_effector(robot, angles).x;
//     double y = end_effector(robot, angles).y;
//     double x_goal = goalpoint.goalpoint.x;
//     double y_goal = goalpoint.goalpoint.y;
//     double alpha_goal = goalpoint.angle1_;
//     double alpha = std::abs(periodic(last_angle(angles) - goalpoint.angle1_, 180));
//     return std::sqrt((x-x_goal)*(x-x_goal) + (y-y_goal)*(y-y_goal));
// }
double Planner_A_star::geuristic(const Robot &robot, const MathVector<double> &angles1, const MathVector<double> &angles2)
{
    return calculateDistance2(end_effector(robot, angles1), end_effector(robot, angles2)); // + std::abs(periodic(last_angle(angles1) - last_angle(angles2), PI));
}

bool Planner_A_star::AStar(const Robot &robot, const GoalPoint &goalpoint, const MathVector<Polygon> &obstacles, std::map<std::string, double> &dictionary)
{
    Timer t("a-star");
    Timer tt("coll_check");
    double time = 0.0;

    Vector2D goal = goalpoint.goalpoint;

    const int g_units = 30; // mesh fineness [-angle1, angle2] -- 2g_units
    double coord_tolerance = 0.1;
    MathVector<double> deltas(robot.dof_, 0.0);

    for (auto i = 0u; i < robot.configuration.size(); i++)
    {
        deltas[i] = std::abs(robot.joints[i].limits[1] - robot.joints[i].limits[0]) / (2 * g_units);
    }

    // MathVector<MathVector<int>> primitivemoves;
    // MathVector<int> current(robot.dof_);
    // generateVectors(primitivemoves, current, 0);

    MathVector<MathVector<int>> primitivemoves;
    for (auto i = 0u; i < robot.dof_; i++)
    {
        MathVector<int> a(robot.dof_, 0.0);
        a[i] = 1;
        primitivemoves.push_back(a);
        a[i] = -1;
        primitivemoves.push_back(a);
    }

    std::priority_queue<Node::Nodetype, MathVector<Node::Nodetype>, CompareNodes> opened_nodes;
    std::map<MathVector<int>, Node::Nodetype, CompareKey> map_pq_opened;
    std::map<MathVector<int>, Node::Nodetype, CompareKey> closed_nodes;
    //std::unordered_set<Node::Nodetype, HashNode, EqualNode> closed_nodes;

    int NODES = 0;
    int Nodes_closed = 0;
    int n_turns = 0;

    MathVector<int> config(robot.dof_, 0);
    MathVector<double> angles = calc_angles(robot, config, deltas);
    Vector2D startPoint = end_effector(robot, angles);

    int choose_weight = 1;
    double product = (goalpoint.goalpoint.x - startPoint.x) * std::cos((goalpoint.angle1_+180)/180*M_PI) 
    + (goalpoint.goalpoint.y - startPoint.y) * std::sin((goalpoint.angle1_+180)/180*M_PI);
    if (product > 0.0) 
        choose_weight = 2;
    else 
        choose_weight = 1;
    std::cout << "CHOSED WEIGHTS NUMBER: " << choose_weight << std::endl;

    Node::Nodetype start = new Node(config, 0.0, heuristic(robot, goalpoint, angles, choose_weight), nullptr);
    opened_nodes.push(start);
    map_pq_opened.emplace(start->position, start);
    //closed_nodes.reserve(200000);



    

    std::string filePath = "heatmap.txt";
    std::ofstream heatmap(filePath);
    long int count = 0;
    while (!opened_nodes.empty())
    {
        Node::Nodetype current = opened_nodes.top();

        opened_nodes.pop();
        map_pq_opened.erase(current->position);
        closed_nodes.emplace(current->position, current);

        //closed_nodes.insert(current);
        Nodes_closed++;
        auto currxy = end_effector(robot, calc_angles(robot, current->position, deltas));

        angles = calc_angles(robot, current->position, deltas);
        if (count % 10000 == 0)  std::cout << opened_nodes.size() << ' ' << map_pq_opened.size() << ' ' << closed_nodes.size() << std::endl; // " endeff: "<< goal.x << ' ' << goal.y << ' ' << goalpoint.angle1_ <<" current:" << currxy.x << ' ' << currxy.y << ' ' << periodic(last_angle(angles), PI) <<" heuristic: " << heuristic(robot,goalpoint, angles) << std::endl;
        count++;

        MathVector<double> curr_angles = calc_angles(robot, current->position, deltas);

        //heatmap << currxy.x << ',' << currxy.y << ',' << std::abs(periodic(last_angle(angles), PI)) << ',' << current->getFCost() << std::endl;
        //std::cout << periodic(last_angle(angles), PI) << " " << goalpoint.angle1_  << std::endl;
        if (calculateDistance(currxy, goalpoint.goalpoint) < coord_tolerance && std::abs(periodic(last_angle(angles) - goalpoint.angle1_, 180)) < 2*goalpoint.angle2_)
        {
            reconstruct_path(current, input_filename_, output_filename_, robot, deltas);
            heatmap.close();
            angles = calc_angles(robot, current->position, deltas);
            std::cout << "Пришел в: " << end_effector(robot, angles).x << ' ' << end_effector(robot, angles).y << " angle: " << last_angle(angles) << "  " << angles.back() * 180 / std::acos(-1) << " opened nodes: " << NODES << std::endl;

            dictionary["g_units"] = g_units;
            dictionary["coord_tolerance"] = coord_tolerance;
            dictionary["angle_tolerance"] = goalpoint.angle2_;
            dictionary["time"] = t.getElapsedTime();
            dictionary["coll_check_percentage"] = time / t.getElapsedTime() * 100;
            dictionary["opened_nodes"] = NODES;
            dictionary["closed_nodes"] = Nodes_closed;
            dictionary["g_cost"] = current->gCost;
            dictionary["turn_numbers"] = n_turns;
            
            // while (opened_nodes.size() > 0)
            //     {
            //         delete opened_nodes.top();
            //         opened_nodes.pop();
            //     }
            // Delete dynamically allocated elements from the map
            for (auto& pair : closed_nodes) {
                delete pair.second;
            }
            // Clear all elements from the unordered_map
            closed_nodes.clear();

            for (auto& pair : map_pq_opened) {
                delete pair.second;
            }
            // Clear all elements from the unordered_map
            map_pq_opened.clear();


                // Delete dynamically allocated elements from the map
            // for (const auto& pair : closed_nodes) 
            // {
            //     delete pair.second;
            // }
            // for (const auto& pair : map_pq_opened) 
            // {
            //     delete pair.second;
            // }
            // for (Node::Nodetype it : closed_nodes)
            // {
            //     delete it;
            // }

            // while (opened_nodes.size() > 0)
            // {
            //     delete opened_nodes.top();
            //     opened_nodes.pop();
            // }
            return true;
        }
        if (opened_nodes.size() > 3110'000) return false;

        n_turns++;
        for (const auto &i : primitivemoves)
        {
            Node::Nodetype newneighbour = new Node((current->position) + i, 0.0, 0.0, current);
            simplify(newneighbour->position, g_units);
            angles = calc_angles(robot, newneighbour->position, deltas);
            newneighbour->hCost = heuristic(robot, goalpoint, angles, choose_weight);

            tt.start();
            bool flag = collide(robot, angles, obstacles);
            time += tt.getElapsedTime();
            tt.pause();
            if (flag)
            {
                //closed_nodes.insert(newneighbour);
                // std::cout << "collision" << std::endl;
                continue;
            }
            //if (closed_nodes.count(newneighbour) != 0) // if in closed list
            if (closed_nodes.find(newneighbour->position) != closed_nodes.end())
            {
                continue;
            }

            double g_score = current->gCost + geuristic(robot, angles, curr_angles); // tentative
            auto it = map_pq_opened.find(newneighbour->position);

            if (it == map_pq_opened.end()) // Neighbour not in opened => add neighbor
            {
                newneighbour->hCost = heuristic(robot, goalpoint, angles, choose_weight);
                newneighbour->gCost = g_score;
                newneighbour->parent = current;
                map_pq_opened.emplace(newneighbour->position, newneighbour);
                opened_nodes.push(newneighbour);
                NODES++;
            }
            if (g_score < it->second->gCost && it != map_pq_opened.end()) // optimization
            {
                it->second->gCost = g_score;
                it->second->hCost = heuristic(robot, goalpoint, angles, choose_weight);
                it->second->parent = current;
            }
        }
    }
    std::cout << "OPENED NODES " << NODES << std::endl;
    // for (Node::Nodetype it : closed_nodes)
    // {
    //     delete it;
    // }
    // for (const auto& pair : closed_nodes) 
    // {
    //     delete pair.second;
    // }
    // for (const auto& pair : map_pq_opened) 
    // {
    //     delete pair.second;
    // }
    //std::cout << "1234321\n";
    while (opened_nodes.size() > 0)
    {
        delete opened_nodes.top();
        opened_nodes.pop();
    }
                // Delete dynamically allocated elements from the map
            for (auto& pair : closed_nodes) {
                delete pair.second;
            }
            // Clear all elements from the unordered_map
            closed_nodes.clear();

            for (auto& pair : map_pq_opened) {
                delete pair.second;
            }
            // Clear all elements from the unordered_map
            map_pq_opened.clear();
    return false;
}



// bool Planner_A_star::coll_test(const Robot &robot, const MathVector<Polygon> &obstacles)
// {
//     const int g_units = 110;
//     MathVector<double> deltas(robot.dof_, 0.0);
//     for (auto i = 0u; i < robot.configuration.size(); i++)
//     {
//         deltas[i] = std::abs(robot.joints[i].limits[1] - robot.joints[i].limits[0]) / (2 * g_units);
//     }
//     MathVector<MathVector<int>> primitivemoves;
//     for (auto i = 0u; i < robot.dof_; i++)
//     {
//         MathVector<int> a(robot.dof_, 0.0);
//         a[i] = 1;
//         primitivemoves.push_back(a);
//         a[i] = -1;
//         primitivemoves.push_back(a);
//     }

//     MathVector<int> config(robot.dof_, 0);
//     MathVector<double> angles = calc_angles(robot, config, deltas);

//     std::ofstream file(output_filename_);

//     int n = 0;
//     while (true)
//     {

//         n++;
//         if (n == 500)
//             break;
//         config = config + primitivemoves[0];
//         angles = calc_angles(robot, config, deltas);
//         std::cout << 123 << std::endl;
//         if (collide(robot, angles, obstacles))
//         {
//             std::cout << 123 << std::endl;
//             std::cout << "collision" << std::endl;
//             primitivemoves[0][0] *= -1;
//             // int a;
//             // std::cin >> a;
//         }

//         for (size_t i = 0; i < config.size(); ++i)
//         {
//             file << (robot.configuration[i] + config[i] * deltas[i]) * 180.0 / std::acos(-1);
//             if (i != config.size() - 1)
//             {
//                 file << ",";
//             }
//         }
//         file << std::endl;
//     }
//     file.close();
//     return false;
// }
