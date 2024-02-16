#pragma once
#include "obstacles.h"
#include "robot.h"
#include <unordered_set>
#include <queue>
#include <map>
#include <memory>
#include <vector>
#include <unordered_set>
#include <iostream>
#include <cmath>
#include <utility>
#include <string>
#include <deque>
#include <random>
#include <assert.h>
#include "inverse_kinematics.h"
#include <chrono>
#include <ANN.h>  // ANN declarations
#include <multiann.h>
#define RRT_GOAL_POINT_PROBABILITY 0.1
using namespace std::literals;   


class RRT
{
public:
    struct Stat
    {

        unsigned long long number_of_nodes = 1;
        unsigned long long number_of_goal_expanding_nodes = 0;
        unsigned long long number_of_random_nodes = 0;
        unsigned long long number_of_denied_nodes_random = 0;
        unsigned long long number_of_denied_nodes_goal = 0;
        unsigned long long number_of_IK_results = 0;
        double time_of_IK_results = 0;
        double time_of_collision_check_in_IK = 0;
        unsigned long long number_of_collision_check_in_IK = 0;
        double time_of_collision_check = 0;
        unsigned long long number_of_collision_check = 0;
        double time_of_nn_check = 0;
        unsigned long long number_of_nn_check = 0;

        void to_map(std::map<std::string, std::string> *inp_stats);
    };
    struct Tree
    {

        struct Node 
        {
            Node(Robot _position) : position(_position){};
            Node(const Node &_other) = delete;
            Node(Node &&_other) = delete;
            Node operator=(const Node &_other) = delete;
            Node operator=(Node &&_other) = delete;
            ~Node();

            Node* add_children(Robot& Node);


            double distance(const Node* other) const;
            double distance(const Robot other) const;
            Node* get_parent() const;
            Robot get_position() const;
            void set_parent(Node *parent);
            std::vector<Node *> children;

        private:
            Node* _parent = nullptr;
            Robot position;
        };

        Tree(Robot _position) : head(new Node(_position)){};
        Tree(const Tree &_other) = delete;
        Tree(Tree &&_other) = delete;
        Tree operator=(const Tree &_other) = delete;
        Tree operator=(Tree &&_other) = delete;
        ~Tree(){delete head;};

        Node* head;
    };

    RRT(double _tolerance, float _goal_bias, int other_dof, const Robot &inp_start,
        const GoalPoint &inp_goal, const std::vector<Polygon> &inp_obstacles, std::map<std::string, std::string> &inp_stats) : tolerance(_tolerance),goal_bias(_goal_bias),
        dof(other_dof), start(inp_start), goal(inp_goal),
    
                      obstacles(inp_obstacles), tree(start), distance_from_closest(goal.distance(start, start.configuration)), probability_gen(0.0, 1.0), stats_map(&inp_stats)
    {

        std::random_device rd;
        std::mt19937 gen1(rd());
        gen = gen1;

        for (int i = 0; i < dof; i++)
        {
            std::uniform_real_distribution<> dis(start.joints[i].limits[0], start.joints[i].limits[1]);
            random_gen.push_back(dis);
        }
        Timer1 tt1("1");
        InverseKinematics::IK_statistics ik_stats = InverseKinematics::sample_all_goals(end_configurations, start, goal, obstacles, 2);
        stats.time_of_IK_results = tt1.getElapsedTime();
        stats.number_of_collision_check_in_IK = ik_stats.number_of_collision_check;
        stats.time_of_collision_check_in_IK = ik_stats.time_of_collision_check;
        stats.number_of_IK_results = end_configurations.size();

        // TODO: добавить проверку решений с помощью ПЗК

        if (end_configurations.size() == 0)
        {
            for (int sample_angle = 0; sample_angle<=100; sample_angle++){
               
                GoalPoint angle_delta_goal(goal.goalpoint.x,goal.goalpoint.y,goal.angle1_-goal.angle2_+sample_angle*(goal.angle2_/50),goal.angle2_);
                InverseKinematics::IK_statistics ik_stats = InverseKinematics::sample_all_goals(end_configurations, start, angle_delta_goal, obstacles, 10);
               if (end_configurations.size() != 0){
                break;
               }
            }
            
        }
        if (end_configurations.size() == 0){
                throw std::invalid_argument("No solution found for goal configuration.");
        }
        nodes.push_back(NodeAndConfig(tree.head,tree.head->get_position()));

        topology = new int[dof];
        for(int i=0;i<dof;i++){
            topology[i]=1;
        }
        scale = new double[dof];
        for(int i=0;i<dof;i++){
            scale[i]=1;
        }
        MAG = new MultiANN(dof, 1,   topology,scale);
        std::vector<double> head_ptr = tree.head->get_position().configuration;
        configuration2Node[head_ptr] = tree.head;
        ANNpoint head = annAllocPt(dof);
        std::copy(head_ptr.begin(), head_ptr.end(), head);

        MAG->AddPoint(head, head);
        std::cout << "Inverse Kinematics solved, got " << end_configurations.size() << " solutions" << std::endl;
    };

    RRT(const RRT &_other) = delete;
    RRT(RRT &&_other) = delete;
    RRT operator=(const RRT &_other) = delete;
    RRT operator=(RRT &&_other) = delete;
    ~RRT(){
        delete MAG;
        delete[] topology;
        delete[] scale;
    };

    void grow_tree();
    std::vector<Robot> get_end_configurations() const;

    bool is_finished() const;
    Tree &get_tree();
    int get_dof() const;
    std::vector<double> get_path() const;
    void export_stats();

private:
    RRT::Tree::Node* nearest_neighbour_stats(const Robot &pos);
    bool collide_stats(const Robot &robot, const std::vector<double> config, const std::vector<Polygon> &poligons);
    
    void expand_to_goal();
    void expand_to_random();
    Robot get_end_config_sample();
    bool finished = false;
    uint32_t iteration_count = 0;
    double tolerance;
    std::vector<std::uniform_real_distribution<>> random_gen;
    std::uniform_real_distribution<> probability_gen;
    std::mt19937 gen;
    int dof;
    Robot start;
    GoalPoint goal;
    float goal_bias;
    std::vector<Polygon> obstacles;
    std::vector<RRT::Tree::Node*>finish_node;
    Robot random_sample();
    RRT::Tree::Node* nearest_neighbour(const Robot &pos);
    Tree tree;
    RRT::Tree::Node* closest_to_goal_node = tree.head;
    float distance_from_closest;
    void save_tree();
    RRT::Tree::Node* make_step( RRT::Tree::Node& node, const Robot &pos);
    bool is_goal(const RRT::Tree::Node& node);
    std::vector<Robot> end_configurations;
    struct NodeAndConfig{
        RRT::Tree::Node* node;
        Robot config;
        NodeAndConfig(RRT::Tree::Node* inp_node,Robot inp_config):
        node(inp_node), config(inp_config){};
    };
    std::vector<NodeAndConfig> nodes;
    RRT::Stat stats;
    ANNpoint scale;
    int* topology;
    MultiANN* MAG; 
    std::map<std::string, std::string> *stats_map;
    std::map<std::vector<double>, RRT::Tree::Node*> configuration2Node;
};
