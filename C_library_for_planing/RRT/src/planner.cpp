#include "planner.h"
#include "robot.h"
#include <assert.h>
#include <random>
#include <vector>
#include <deque>
#include <iostream>
#include <fstream>
#include <cmath>
#include <algorithm>
#include <iterator>
#include <map>
#include <stdexcept>
#include <assert.h>
#include <chrono>
#include <ANN.h>
#include <multiann.h>
#define COS_TOLERANCE 0.00000001

void RRT::Stat::to_map(std::map<std::string, std::string> *inp_stats)
{
    (*inp_stats)["opened_nodes"] = std::to_string(number_of_nodes);
    (*inp_stats)["number_of_goal_expanding_nodes"] = std::to_string(number_of_goal_expanding_nodes);
    (*inp_stats)["number_of_random_nodes"] = std::to_string(number_of_random_nodes);
    (*inp_stats)["number_of_denied_nodes_random"] = std::to_string(number_of_denied_nodes_random);
    (*inp_stats)["number_of_denied_nodes_goal"] = std::to_string(number_of_denied_nodes_goal);
    (*inp_stats)["number_of_IK_results"] = std::to_string(number_of_IK_results);
    (*inp_stats)["time_of_IK_results"] = std::to_string(time_of_IK_results);
    (*inp_stats)["time_of_collision_check_in_IK"] = std::to_string(time_of_collision_check_in_IK);
    (*inp_stats)["number_of_collision_check_in_IK"] = std::to_string(number_of_collision_check_in_IK);
    (*inp_stats)["time_of_collision_check"] = std::to_string(time_of_collision_check);
    (*inp_stats)["number_of_collision_check"] = std::to_string(number_of_collision_check);
    (*inp_stats)["time_of_nn_check"] = std::to_string(time_of_nn_check);
    (*inp_stats)["number_of_nn_check"] = std::to_string(number_of_nn_check);
}

void RRT::export_stats(){
    stats.to_map(stats_map);
}

RRT::Tree::Node::~Node(){
    for(Node* child:children){
        delete child;
    }
    children.clear();
}

RRT::Tree::Node* RRT::nearest_neighbour_stats(const Robot &pos)
{
    /*
    Обёртка для NN для сбора времени выполнения
    */
    Timer1 tt1("1");
    auto result = nearest_neighbour(pos);
    stats.number_of_nn_check;
    stats.time_of_nn_check += tt1.getElapsedTime();
    return result;
}

bool RRT::collide_stats(const Robot &robot, const std::vector<double> config, const std::vector<Polygon> &poligons)
{
    /*
     Обёртка для collide, которая собирает статистику по времени выполнения
    */
    Timer1 tt1("1");
    bool result = collide(robot, config, poligons);
    stats.number_of_collision_check++;
    stats.time_of_collision_check += tt1.getElapsedTime();
    return result;
}
RRT::Tree::Node* RRT::Tree::Node::add_children(Robot& child_pos)
{
    // assert(!this->is_children(child));
    // assert(child); // проверка на nullptr
    RRT::Tree::Node* child = new RRT::Tree::Node(child_pos);
    children.push_back(child);
    child->set_parent(this);
    return child;
}

void RRT::Tree::Node::set_parent(Node* parent)
{
    _parent = parent;
}



double RRT::Tree::Node::distance(const Node* other) const
{
    return this->distance(other->get_position());
}

double RRT::Tree::Node::distance(Robot other) const
{
   return this->get_position().distance(other);
}

RRT::Tree::Node* RRT::Tree::Node::get_parent() const
{
    return this->_parent;
}

Robot RRT::Tree::Node::get_position() const
{
    return position;
}

void RRT::grow_tree()
{

    if (probability_gen(gen) < goal_bias)
    {
        expand_to_goal();
    }
    else
    {
        expand_to_random();
    }
    iteration_count++;
}


Robot RRT::get_end_config_sample()
{
    return end_configurations[std::floor(end_configurations.size() * probability_gen(gen) * 0.99)];
    
}

Robot RRT::random_sample()
{
    Robot sample = Robot(start);

    for (int i = 0; i < start.configuration.size(); i++)
    {

        sample.configuration[i] = random_gen[i](gen);
    }
    return sample;
}

void RRT::expand_to_goal()
{
    // Берём случайную финишную точку и стремимся к ней, пока можем.
    Robot sample_goal_config = get_end_config_sample();
    RRT::Tree::Node* nearest_node = nearest_neighbour_stats(sample_goal_config);

         RRT::Tree::Node* next_node = make_step(*nearest_node, sample_goal_config);
        while (next_node)
        {
            stats.number_of_goal_expanding_nodes++;
            stats.number_of_nodes++;
            nodes.push_back(RRT::NodeAndConfig(next_node,next_node->get_position()));
            std::vector<double> node_ptr = next_node->get_position().configuration;
            configuration2Node[node_ptr] = next_node;
            ANNpoint head = annAllocPt(dof);
            std::copy(node_ptr.begin(), node_ptr.end(), head);

            MAG->AddPoint(head, head);
            if (is_goal(*next_node)){
                finished = true;
               finish_node.push_back(next_node);
               return;
            }
            next_node = make_step(*next_node, sample_goal_config);
        }
        stats.number_of_denied_nodes_goal++;
        

}

void RRT::expand_to_random()
{
    Robot random_position = random_sample();
    RRT::Tree::Node* nearest_node = nearest_neighbour_stats(random_position);
    RRT::Tree::Node* next_node = make_step(*nearest_node, random_position);
    if (next_node)
    {
        stats.number_of_random_nodes++;
        stats.number_of_nodes++;
        nodes.push_back(RRT::NodeAndConfig(next_node,next_node->get_position()));
        std::vector<double> node_ptr = next_node->get_position().configuration;
            configuration2Node[node_ptr] = next_node;
            ANNpoint head = annAllocPt(dof);
            std::copy(node_ptr.begin(), node_ptr.end(), head);

            MAG->AddPoint(head, head);
        if (is_goal(*next_node))
        {
            finished = true;
            finish_node.push_back(next_node);
        }
    }
    else
    {
        stats.number_of_denied_nodes_random++;
    }
}
bool RRT::is_finished() const
{
    return finished;
}

RRT::Tree::Node* RRT::nearest_neighbour(const Robot &pos)
{
    ANNpoint query_pt; 
    query_pt = annAllocPt(dof);  
    std::copy(pos.configuration.begin(), pos.configuration.end(), query_pt);
    double d_ann = INFINITY;
    
    int idx_ann;
    double*  nearest_cfg = (double*)MAG->NearestNeighbor(
                query_pt, idx_ann,
                d_ann);

    std::vector<double> near_pos(pos.configuration);
    for(int i=0;i<dof;i++){
        near_pos[i]=nearest_cfg[i];
    }
    RRT::Tree::Node* nearest_node = configuration2Node[near_pos];
    annDeallocPt(query_pt);
    return nearest_node;
    
}
RRT::Tree::Node* RRT::make_step(RRT::Tree::Node& node, const Robot &pos)
{
    // Делаем фиксированный шаг в сторону точки
    //  проверяем на колизион, 
    // for (auto angle : pos.configuration)
    // {

    //     assert(!std::isnan(angle));
    // }
    // std::cout << (*node).distance(pos) << std::endl;
    Robot delta;
    double dist = node.distance(pos);
    if (dist != 0) // защита от деления на 0
    {
        delta = (pos - node.get_position()) / dist; // нормированный вектор от ноды по рандомной позиции
    }
    else
    {
        return nullptr;
    }

    Robot new_pose = node.get_position() + delta * tolerance; // dis(gen);
    if (node.distance(new_pose) > node.distance(pos))          // если шаг дальше, чем точка, то урезаем до точки
    {
        new_pose = pos;
    }
    if (collide_stats(new_pose, new_pose.configuration, obstacles))
    {
        return nullptr;
    }

    RRT::Tree::Node* new_node = node.add_children(new_pose);
    return new_node;
}

bool RRT::is_goal(const RRT::Tree::Node& node)
{
    Robot pos = node.get_position();
    return goal.is_goal(pos);
}

std::vector<double> RRT::get_path() const
{
    std::vector<double> path;
    RRT::Tree::Node* node = finish_node[0];

        while (node)
        {
             
            for (int joint_index = dof - 1; joint_index >= 0; joint_index--)
            {
                path.push_back(node->get_position().configuration[joint_index]);
            }
            
            node = node->get_parent();
        }
        std::reverse(path.begin(), path.end());
    
    return path;
}

RRT::Tree &RRT::get_tree()
{
    return tree;
}
int RRT::get_dof() const
{
    return dof;
}

std::vector<Robot> RRT::get_end_configurations() const
{
    return end_configurations;
}
