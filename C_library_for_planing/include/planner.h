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
#include <list>
#include "geometry.h"


const double PI = std::acos(-1);
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
    typedef Node* Nodetype ;

    std::vector<int> position;
    double gCost;
    double hCost;
    Node::Nodetype parent;

    Node(const std::vector<int>& pos, double g, double h, const Node::Nodetype &p) : position(pos), gCost(g), hCost(h), parent(p) {}


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
    bool operator()(const Node::Nodetype& node1, const Node::Nodetype& node2) const {
        // Higher priority nodes should have lower priority values
        return node1->getFCost() > node2->getFCost();
    }
};

struct HashNode 
{
    size_t operator()(const Node::Nodetype& node) const {
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
    bool operator()(const Node::Nodetype& node1, const Node::Nodetype& node2) const
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

void reconstruct_path(const Node::Nodetype &node, std::string input_filename, std::string output_filename, const Robot &robot, const std::vector<double> &deltas)
{
    std::vector<Node::Nodetype> path;
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

double periodic(const double &n, const double &L)
{
    if (n>=0.0) return fmod(n + L, 2*L) - L;
    else if (n>=-L && n<0.0) return n;
    else  return fmod(n + L, 2*L) + L;
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

double last_angle(const std::vector<double> &angles)
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
        double g = calculateDistance(end_effector(robot, angles), goalpoint.goalpoint);
        return g + std::abs(periodic(last_angle(angles) - goalpoint.angle1_, PI))/std::pow(123.5 +g, 1);
    }
    double geuristic(const Robot &robot, const std::vector<double> &angles1, const std::vector<double> &angles2)
    {
        return calculateDistance2(end_effector(robot, angles1), end_effector(robot, angles2));// + std::abs(periodic(last_angle(angles1) - last_angle(angles2), PI));
    }

   
    bool AStar(const Robot& robot, const GoalPoint& goalpoint, const std::vector<Polygon>& obstacles, std::map<std::string, double> &dictionary)
    {
        Timer t("a-star");
        Timer tt("coll_check");
        double time=0.0;

        Vector2D goal = goalpoint.goalpoint;
        
        const int g_units = 30; // mesh fineness [-angle1, angle2] -- 2g_units
        double coord_tolerance = 0.1;
        std::vector<double> deltas(robot.dof_, 0.0);

        for (auto i = 0u; i < robot.configuration.size(); i++)
        {
            deltas[i] = std::abs(robot.joints[i].limits[1] - robot.joints[i].limits[0]) / (2*g_units);
        }



        std::vector<std::vector<int>> primitivemoves;
        std::vector<int> current(robot.dof_);
        generateVectors(primitivemoves, current, 0);



        // std::vector<std::vector<int>> primitivemoves;        
        // for (auto i = 0u; i < robot.dof_; i++)
        // {
        //     std::vector<int> a(robot.dof_, 0.0);
        //     a[i] = 1;
        //     primitivemoves.push_back(a);
        //     a[i] = -1;
        //     primitivemoves.push_back(a);            
        // }

        
        std::priority_queue <Node::Nodetype, std::vector<Node::Nodetype>,  CompareNodes> opened_nodes;
        std::map<std::vector<int>, Node::Nodetype, CompareKey> map_pq_opened;
        std::unordered_set<Node::Nodetype, HashNode, EqualNode> closed_nodes;

        std::vector<int> config(robot.dof_, 0);
        std::vector<double> angles = calc_angles(robot, config, deltas); 

        Node::Nodetype start = new Node(config, 0.0, heuristic(robot, goalpoint, angles) , nullptr);
        opened_nodes.push(start);
        map_pq_opened.emplace(start->position, start);
        Vector2D startPoint = end_effector(robot,angles);
        int NODES = 0;
        int Nodes_closed = 0;
        int n_turns = 0;

        std::string filePath = "heatmap.txt";
        std::ofstream heatmap(filePath);
        

        while (!opened_nodes.empty())
        {
            Node::Nodetype current = opened_nodes.top();


            opened_nodes.pop();
            map_pq_opened.erase(current->position);

            closed_nodes.insert(current);
            Nodes_closed++;
            auto currxy = end_effector(robot, calc_angles(robot, current->position, deltas));

            //std::cout << current->getFCost() << ' ' << current->gCost << ' ' << current->hCost  << std::endl;
            
            //if (opened_nodes.size()>50'000) return false;
            
            angles = calc_angles(robot, current->position, deltas);
            std::cout << opened_nodes.size() << ' ' << map_pq_opened.size() << ' ' << closed_nodes.size() << std::endl;// " endeff: "<< goal.x << ' ' << goal.y << ' ' << goalpoint.angle1_ <<" current:" << currxy.x << ' ' << currxy.y << ' ' << periodic(last_angle(angles), PI) <<" heuristic: " << heuristic(robot,goalpoint, angles) << std::endl;
            //std:: cout << angles[0]*180/std::acos(-1) << ',' << angles[1]*180/std::acos(-1) << ',' << angles[2]*180/std::acos(-1) << std::endl;
            std::vector<double> curr_angles = calc_angles(robot, current->position, deltas);

            heatmap << currxy.x << ',' << currxy.y << ',' << std::abs(periodic(last_angle(angles), PI)) << ',' << current->getFCost() << std::endl;
            
            if (calculateDistance(currxy, goalpoint.goalpoint) < coord_tolerance  && std::abs(periodic(last_angle(angles) - goalpoint.angle1_, PI))<goalpoint.angle2_)
            {
                reconstruct_path(current,input_filename_, output_filename_, robot, deltas);
                heatmap.close();
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
                for( Node::Nodetype it:closed_nodes){
            delete it;
        }
       
        while (opened_nodes.size()>0){
            delete opened_nodes.top();
            opened_nodes.pop();
        }
                return true;
            }
            n_turns++;
            for (const auto &i:primitivemoves)
            {
                Node::Nodetype newneighbour = new Node(current->position+i, 0.0, 0.0, current);
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
        for( Node::Nodetype it:closed_nodes){
            delete it;
        }
       
        while (opened_nodes.size()>0){
            delete opened_nodes.top();
            opened_nodes.pop();
        } 
    return false;
    }



    // //////////////
    // bool AStar2(const Robot& robot, const GoalPoint& goalpoint, const std::vector<Polygon>& obstacles, std::map<std::string, double> &dictionary)
    //  {
    //     Timer t("a-star");
    //     Timer tt("coll_check");
    //     double time=0.0;

    //     Vector2D goal = goalpoint.goalpoint;
        
    //     const int g_units = 30; // mesh fineness [-angle1, angle2] -- 2g_units
    //     double coord_tolerance = 0.1;
    //     std::vector<double> deltas(robot.dof_, 0.0);

    //     for (auto i = 0u; i < robot.configuration.size(); i++)
    //     {
    //         deltas[i] = std::abs(robot.joints[i].limits[1] - robot.joints[i].limits[0]) / (2*g_units);
    //     }



    //     std::vector<std::vector<int>> primitivemoves1;
    //     std::vector<int> current(robot.dof_);
    //     generateVectors(primitivemoves1, current, 0);



    //     std::vector<std::vector<int>> primitivemoves2;        
    //     for (auto i = 0u; i < robot.dof_; i++)
    //     {
    //         std::vector<int> a(robot.dof_, 0.0);
    //         a[i] = 1;
    //         primitivemoves2.push_back(a);
    //         a[i] = -1;
    //         primitivemoves2.push_back(a);            
    //     }

    //     std::priority_queue <Node::Nodetype, std::vector<Node::Nodetype>,  CompareNodes> opened_nodes2;
    //     std::map<std::vector<int>, Node::Nodetype, CompareKey> map_pq_opened2;
    //     std::unordered_set<Node::Nodetype, HashNode, EqualNode> closed_nodes2;

    //     std::vector<int> config2(robot.dof_, 0);
    //     std::vector<double> angles2 = calc_angles(robot, config2, deltas); 

    //     Node::Nodetype start2 = std::make_shared<Node>(config2, 0.0, heuristic(robot, goalpoint, angles2) , nullptr);
    //     opened_nodes2.push(start2);
    //     map_pq_opened2.emplace(start2->position, start2);
    //     int NODES2 = 0;
    //     int Nodes_closed2 = 0;
    //     int n_turns2 = 0;


    //     std::priority_queue <Node::Nodetype, std::vector<Node::Nodetype>,  CompareNodes> opened_nodes1;
    //     std::map<std::vector<int>, Node::Nodetype, CompareKey> map_pq_opened1;
    //     std::unordered_set<Node::Nodetype, HashNode, EqualNode> closed_nodes1;

    //     std::vector<int> config1(robot.dof_, 0);
    //     std::vector<double> angles1 = calc_angles(robot, config1, deltas); 

    //     Node::Nodetype start1 = std::make_shared<Node>(config1, 0.0, heuristic(robot, goalpoint, angles1) , nullptr);
    //     opened_nodes1.push(start1);
    //     map_pq_opened1.emplace(start1->position, start1);
    //     int NODES1 = 0;
    //     int Nodes_closed1 = 0;
    //     int n_turns1 = 0;





    //     while (!opened_nodes1.empty() && !opened_nodes2.empty())
    //     {

    //         ////
    //         Node::Nodetype current2 = opened_nodes2.top();
    //         opened_nodes2.pop();
    //         map_pq_opened2.erase(current2->position);

    //         closed_nodes2.insert(current2);
    //         Nodes_closed2++;
    //         auto currxy2 = end_effector(robot, calc_angles(robot, current2->position, deltas));
    //         angles2 = calc_angles(robot, current2->position, deltas);
    //         std::cout << opened_nodes2.size() << ' ' << map_pq_opened2.size() << ' ' << closed_nodes2.size() << std::endl;
    //         std::vector<double> curr_angles2 = calc_angles(robot, current2->position, deltas);
            

    //         Node::Nodetype current1 = opened_nodes1.top();
    //         opened_nodes1.pop();
    //         //map_pq_opened1.erase(current1->position);

    //         closed_nodes1.insert(current1);
    //         Nodes_closed1++;
    //         auto currxy1 = end_effector(robot, calc_angles(robot, current1->position, deltas));
    //         angles1 = calc_angles(robot, current1->position, deltas);
    //         std::cout << opened_nodes1.size() << ' ' << map_pq_opened1.size() << ' ' << closed_nodes1.size() << std::endl;
    //         std::vector<double> curr_angles1 = calc_angles(robot, current1->position, deltas);

    //         if (calculateDistance(currxy1, goalpoint.goalpoint) < coord_tolerance  && std::abs(periodic(last_angle(angles1) - goalpoint.angle1_, PI))<2*goalpoint.angle2_)
    //         {
    //             reconstruct_path(current1,input_filename_, output_filename_, robot, deltas);
    //             angles1 = calc_angles(robot, current1->position, deltas);
    //             std::cout << "Пришел в: " << end_effector(robot, angles1).x << ' ' << end_effector(robot, angles1).y << 
    //             " angle: "<< last_angle(angles1) << "  "<< angles1.back()*180/std::acos(-1) << " opened nodes: " << NODES1 << std::endl;

    //             dictionary["g_units"] = g_units;
    //             dictionary["coord_tolerance"] = coord_tolerance;
    //             dictionary["angle_tolerance"] = goalpoint.angle2_;
    //             dictionary["time"] = t.getElapsedTime();
    //             dictionary["coll_check_percentage"] = time/t.getElapsedTime() * 100;
    //             dictionary["opened_nodes"] = NODES1;
    //             dictionary["closed_nodes"] = Nodes_closed1;
    //             dictionary["g_cost"] = current1->gCost;
    //             dictionary["turn_numbers"] = n_turns1;
            
    //             return true;
    //         }
    //         n_turns1++;

    //         if (calculateDistance(currxy2, goalpoint.goalpoint) < coord_tolerance  && std::abs(periodic(last_angle(angles2) - goalpoint.angle1_, PI))<2*goalpoint.angle2_)
    //         {
    //             reconstruct_path(current2,input_filename_, output_filename_, robot, deltas);
    //             angles2 = calc_angles(robot, current2->position, deltas);
    //             std::cout << "Пришел в: " << end_effector(robot, angles2).x << ' ' << end_effector(robot, angles2).y << 
    //             " angle: "<< last_angle(angles2) << "  "<< angles2.back()*180/std::acos(-1) << " opened nodes: " << NODES2 << std::endl;

    //             dictionary["g_units"] = g_units;
    //             dictionary["coord_tolerance"] = coord_tolerance;
    //             dictionary["angle_tolerance"] = goalpoint.angle2_;
    //             dictionary["time"] = t.getElapsedTime();
    //             dictionary["coll_check_percentage"] = time/t.getElapsedTime() * 100;
    //             dictionary["opened_nodes"] = NODES2;
    //             dictionary["closed_nodes"] = Nodes_closed2;
    //             dictionary["g_cost"] = current2->gCost;
    //             dictionary["turn_numbers"] = n_turns2;
            
    //             return true;
    //         }
    //         n_turns2++;
            
    //         for (const auto &i:primitivemoves1)
    //         {
    //             Node::Nodetype newneighbour = std::make_shared<Node>(current1->position+i, 0.0, 0.0, current1);
    //             simplify(newneighbour->position, g_units);
    //             angles1 = calc_angles(robot, newneighbour->position, deltas);
    //             newneighbour->hCost = heuristic(robot, goalpoint, angles1);

    //             tt.start();
    //             bool flag = collide(robot, angles1, obstacles);
    //             time+=tt.getElapsedTime();
    //             tt.pause();
    //             if  (flag)
    //             {
    //                 //closed_nodes.insert(newneighbour);
    //                 //std::cout << "collision" << std::endl;
    //                 continue;
    //             }
    //             if (closed_nodes1.count(newneighbour) != 0) //if in closed list
    //             {
    //                 continue;
    //             }

    //             double g_score = current1->gCost + geuristic(robot,  angles1, curr_angles1);   //tentative
    //             auto it = map_pq_opened1.find(newneighbour->position);


    //             if (it == map_pq_opened1.end()) //Neighbour not in opened => add neighbor
    //             {
    //                 newneighbour->hCost = heuristic(robot, goalpoint, angles1);
    //                 newneighbour->gCost = g_score;
    //                 newneighbour->parent = current1;
    //                 map_pq_opened1.emplace(newneighbour->position, newneighbour);
    //                 opened_nodes1.push(newneighbour);
    //                 NODES1++;   
    //             }
    //             if (g_score < it->second->gCost && it != map_pq_opened1.end()) // optimization
    //             {
    //                 it->second->gCost = g_score;
    //                 it->second->hCost = heuristic(robot, goalpoint, angles1);
    //                 it->second->parent = current1;
    //             }
    //         }
            
            
    //         for (const auto &i:primitivemoves2)
    //         {
    //             Node::Nodetype newneighbour = std::make_shared<Node>(current2->position+i, 0.0, 0.0, current2);
    //             simplify(newneighbour->position, g_units);
    //             angles2 = calc_angles(robot, newneighbour->position, deltas);
    //             newneighbour->hCost = heuristic(robot, goalpoint, angles2);

    //             tt.start();
    //             bool flag = collide(robot, angles2, obstacles);
    //             time+=tt.getElapsedTime();
    //             tt.pause();
    //             if  (flag)
    //             {
    //                 //closed_nodes.insert(newneighbour);
    //                 //std::cout << "collision" << std::endl;
    //                 continue;
    //             }
    //             if (closed_nodes2.count(newneighbour) != 0) //if in closed list
    //             {
    //                 continue;
    //             }

    //             double g_score = current2->gCost + geuristic(robot,  angles2, curr_angles2);   //tentative
    //             auto it = map_pq_opened2.find(newneighbour->position);


    //             if (it == map_pq_opened2.end()) //Neighbour not in opened => add neighbor
    //             {
    //                 newneighbour->hCost = heuristic(robot, goalpoint, angles2);
    //                 newneighbour->gCost = g_score;
    //                 newneighbour->parent = current2;
    //                 map_pq_opened2.emplace(newneighbour->position, newneighbour);
    //                 opened_nodes2.push(newneighbour);
    //                 NODES2++;   
    //             }
    //             if (g_score < it->second->gCost && it != map_pq_opened2.end()) // optimization
    //             {
    //                 it->second->gCost = g_score;
    //                 it->second->hCost = heuristic(robot, goalpoint, angles2);
    //                 it->second->parent = current2;
    //             }
    //         }
    //     }  
    // std::cout << "OPENED NODES " << NODES2 << std::endl;     

    // return false;
    // }
    // /////////////////////////



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


template <typename T>
int signum(T number) {
    //static_assert(std::is_arithmetic<T>, "signum() requires an arithmetic type.");
    return (T(0) < number) - (number < T(0));
}


double norm1(const std::vector<double> &v1, const std::vector<double> &v2)
{
    double n = 0.0;
    for (int i=0; i<v1.size(); i++)
    {
        n += std::abs(v1[i] - v2[i]); 
    }
    return n;
}

double norm0(const std::vector<double> &v1, const std::vector<double> &v2)
{
    double max = 0.0;
    for (int i=0; i<v1.size(); i++)
    {
        if ( max < std::abs(v1[i] - v2[i])) max = std::abs(v1[i] - v2[i]); 
    }
    return max;
}







//     bool AStar(const Robot& robot, const std::vector<double> &startconfig, const std::vector<double> &goalconfig, const std::vector<Polygon>& obstacles, std::map<std::string, double> &dictionary, std::string input_filename_,
//     std::string output_filename_, GoalPoint goalpoint)
//     {
//         Timer t("a-star");
//         Timer tt("coll_check");
//         double time=0.0;

//          Vector2D goal = goalpoint.goalpoint;
//         const int g_units = 30; // mesh fineness [-angle1, angle2] -- 2g_units
//         double coord_tolerance = 0.1;
//         std::vector<double> deltas(robot.dof_, 0.0);

//         for (auto i = 0u; i < robot.configuration.size(); i++)
//         {
//             deltas[i] = std::abs(robot.joints[i].limits[1] - robot.joints[i].limits[0]) / (2*g_units);
//         }



//         std::vector<std::vector<int>> primitivemoves;
//         std::vector<int> current(robot.dof_);
//         generateVectors(primitivemoves, current, 0);



//         // std::vector<std::vector<int>> primitivemoves;        
//         // for (auto i = 0u; i < robot.dof_; i++)
//         // {
//         //     std::vector<int> a(robot.dof_, 0.0);
//         //     a[i] = 1;
//         //     primitivemoves.push_back(a);
//         //     a[i] = -1;
//         //     primitivemoves.push_back(a);            
//         // }

        
//         std::priority_queue <Node::Nodetype, std::vector<Node::Nodetype>,  CompareNodes> opened_nodes;
//         std::map<std::vector<int>, Node::Nodetype, CompareKey> map_pq_opened;
//         std::unordered_set<Node::Nodetype, HashNode, EqualNode> closed_nodes;

//         std::vector<int> config(robot.dof_, 0);
//         std::vector<double> angles = calc_angles(robot, config, deltas); 

//         Node::Nodetype start = std::make_shared<Node>(config, 0.0, norm1(startconfig, goalconfig) , nullptr);
//         opened_nodes.push(start);
//         map_pq_opened.emplace(start->position, start);
//         Vector2D startPoint = end_effector(robot,angles);
//         int NODES = 0;
//         int Nodes_closed = 0;
//         int n_turns = 0;


//         while (!opened_nodes.empty())
//         {
//             Node::Nodetype current = opened_nodes.top();
//             opened_nodes.pop();
//             map_pq_opened.erase(current->position);

//             closed_nodes.insert(current);
//             Nodes_closed++;
//             auto currxy = end_effector(robot, calc_angles(robot, current->position, deltas));
            
//             angles = calc_angles(robot, current->position, deltas);
//             std::cout << opened_nodes.size() << ' ' << map_pq_opened.size() << ' ' << closed_nodes.size() << std::endl;// " endeff: "<< goal.x << ' ' << goal.y << ' ' << goalpoint.angle1_ <<" current:" << currxy.x << ' ' << currxy.y << ' ' << periodic(last_angle(angles), PI) <<" heuristic: " << heuristic(robot,goalpoint, angles) << std::endl;
    
//             std::vector<double> curr_angles = calc_angles(robot, current->position, deltas);
            
//             if (calculateDistance(currxy, goalpoint.goalpoint) < coord_tolerance  && std::abs(periodic(last_angle(angles) - goalpoint.angle1_, PI))<2*goalpoint.angle2_)
//             {
//                 reconstruct_path(current,input_filename_, output_filename_, robot, deltas);
//                 angles = calc_angles(robot, current->position, deltas);
//                 std::cout << "Пришел в: " << end_effector(robot, angles).x << ' ' << end_effector(robot, angles).y << 
//                 " angle: "<< last_angle(angles) << "  "<< angles.back()*180/std::acos(-1) << " opened nodes: " << NODES << std::endl;

//                 dictionary["g_units"] = g_units;
//                 dictionary["coord_tolerance"] = coord_tolerance;
//                 dictionary["angle_tolerance"] = 0.0;
//                 dictionary["time"] = t.getElapsedTime();
//                 dictionary["coll_check_percentage"] = time/t.getElapsedTime() * 100;
//                 dictionary["opened_nodes"] = NODES;
//                 dictionary["closed_nodes"] = Nodes_closed;
//                 dictionary["g_cost"] = current->gCost;
//                 dictionary["turn_numbers"] = n_turns;
            
//                 return true;
//             }
//             n_turns++;
//             for (const auto &i:primitivemoves)
//             {
//                 Node::Nodetype newneighbour = std::make_shared<Node>(current->position+i, 0.0, 0.0, current);
//                 simplify(newneighbour->position, g_units);
//                 angles = calc_angles(robot, newneighbour->position, deltas);
//                 newneighbour->hCost = norm1(goalconfig, angles);

//                 tt.start();
//                 bool flag = collide(robot, angles, obstacles);
//                 time+=tt.getElapsedTime();
//                 tt.pause();
//                 if  (flag)
//                 {
//                     //closed_nodes.insert(newneighbour);
//                     //std::cout << "collision" << std::endl;
//                     continue;
//                 }
//                 if (closed_nodes.count(newneighbour) != 0) //if in closed list
//                 {
//                     continue;
//                 }

//                 double g_score = current->gCost + norm1(angles, curr_angles);   //tentative
//                 auto it = map_pq_opened.find(newneighbour->position);


//                 if (it == map_pq_opened.end()) //Neighbour not in opened => add neighbor
//                 {
//                     newneighbour->hCost = norm1(goalconfig, angles);
//                     newneighbour->gCost = g_score;
//                     newneighbour->parent = current;
//                     map_pq_opened.emplace(newneighbour->position, newneighbour);
//                     opened_nodes.push(newneighbour);
//                     NODES++;   
//                 }
//                 if (g_score < it->second->gCost && it != map_pq_opened.end()) // optimization
//                 {
//                     it->second->gCost = g_score;
//                     it->second->hCost = norm1(goalconfig, angles);
//                     it->second->parent = current;
//                 }
//             }
//         }  
//     std::cout << "OPENED NODES " << NODES << std::endl;      
//     return false;
//     }

// bool check_trajectory(const std::string &input_filename)
// {
//     bool needchange = false;
//     std::vector<Polygon> polygons;
//     Robot robot = Robot();
//     GoalPoint goal(0.0, 0.0, 0.0, 0.0);
//     bool read_normally = read_scene(input_filename, polygons, robot, goal); 

//     std::ifstream file(input_filename);

//     std::string xmlString((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
//     xmlString.push_back('\0'); 
    
//     rapidxml::xml_document<> doc;
//     doc.parse<0>(&xmlString[0]);

//     std::string csvContent;
//     rapidxml::xml_node<>* csvNode = doc.first_node("input_info")->first_node("csv");
//     if (csvNode) {
//         csvContent = csvNode->value();
//     } else {
//         std::cout << "CSV content node not found in the XML." << std::endl;
//     }

//     std::istringstream csvStream(csvContent);
//     std::string line;
//     std::list<std::vector<double>> states;
//     while (std::getline(csvStream, line)) 
//     {
//         std::vector<std::string> rowValues;
//         std::string value;
//         std::stringstream ss(line);

//         while (std::getline(ss, value, ','))  //read line
//         {
//             rowValues.push_back(value);
//         }

//         // Process the row data as needed
//         std::vector<double> angles;
//         for (const auto& val : rowValues) 
//         {
//             angles.push_back(std::stod(val) * std::acos(-1)/180);
//         }
//         states.push_back(angles);
//         //print_vector(angles);
//     }

//     std::cout << std::endl;

//     const double deltaeps = PI/180 * 0.1; // 0.1 grad
//     //in states lies configs in trajectory
//     std::list<std::vector<double>>::iterator it = states.begin();
//     std::vector<double> current = *it;
//     std::vector<double> next;

//     std::vector<int> indexes;
//     while (it!=std::prev(states.end()))
//     {
//         next = *(++it);
//         //print_vector(current);
//         //current and next -- one joint moved
//         for (int i=0; i<current.size(); i++)
//         {
//             if (std::abs(next[i] - current[i])>1e-2)
//             {
//                 indexes.push_back(i);
//             }
//         }

//         //know indexes of movable joints

        
//         std::vector<double> interpolate = current;

//         std::vector<double> signs;
//         for (int i=0; i<indexes.size(); i++)
//         {
//             signs.push_back( next[i] - interpolate[i]);
//         }
//         bool flag = false;
        
        
        
//             while (norm0(next, interpolate) > 6*deltaeps && (!flag))
//             {
//                 for (int i = 0; i<next.size(); i++)
//                 {
//                     interpolate[i] += deltaeps * signs[i];
//                     if (collide(robot, interpolate, polygons))
//                     {
//                         flag = true;
//                         needchange = true;
//                     }
                
//                 if (flag == true)
//                 { 
//                     // std::vector<std::vector<double>> a = {};// Astar(robot, polygons, current, next, deltaeps);

//                     //     for (const auto& value : a)
//                     //     {
//                     //         states.insert(it, value);
//                     //     }
//                 }
//             }
//             }
//         current = *it;
//     }

//     // Write vectors to the CSV file
//     std::string path_csv;
//     for (const auto& n : states)
//     {
//         for (size_t i = 0; i < n.size(); ++i)
//         {
//             path_csv.append(std::to_string(n[i] * 180.0 / std::acos(-1)));
//             if (i != n.size() - 1) { path_csv.append(",");}  
//         }
//         path_csv.append("\n");
//     }

//     std::ifstream inputFile(input_filename);
//     std::ofstream outputFile("fixed" + input_filename);

//     if (inputFile.is_open() && outputFile.is_open())
//     {
//         std::string line;
//         while (std::getline(inputFile, line)) 
//         { 
//             if (line == "</input_info>") continue;
//             if (line[1] == 'c' && line[2] == 's') { break;}
//             outputFile << line << std::endl; 
            
//         }
//         outputFile << "<csv>" << path_csv <<"</csv>";
//         outputFile << "</input_info>" << std::endl;
//         inputFile.close();  
//         outputFile.close(); 
//     }
//     else { std::cerr << "Error." << std::endl;}
//     return needchange;
// }