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
#define RRT_GOAL_POINT_PROBABILITY 0.1

struct Node
{
    std::vector<double> position;
    double gCost;
    double hCost;
    Node *parent;

    Node(){};
    Node(const std::vector<double> &pos, double g, double h, Node *p) : position(pos), gCost(g), hCost(h), parent(p) {}

    double getFCost() const
    {
        return gCost + hCost;
    }
    bool operator<(const Node &other) const
    {
        return getFCost() < other.getFCost();
    }
    bool operator>(const Node &other) const
    {
        return getFCost() > other.getFCost();
    }
    bool operator==(const Node &other) const
    {
        double tolerance = 1e-6;
        double diff;
        for (auto i = 0u; i < position.size(); i++)
            diff += std::abs(position[i] - other.position[i]);
        return diff <= tolerance;
    }
};

struct NodeHash
{
    size_t operator()(const Node &node) const
    {
        size_t hash = 0;
        for (const auto &value : node.position)
        {
            // Combine hash with each value in the vector
            hash ^= std::hash<double>()(value);
        }
        return hash;
    }
};

class Planner
{
public:
    Planner(std::string filename) : filename_(filename) {}

    void AStar(const Robot &robot, const Vector2D &goal, const std::vector<Polygon> &obstacles);

    void RRT(const Robot &start, const Robot &goal, const std::vector<Polygon> &obstacles);

private:
    std::string filename_;
    const double eps = 1e-3;
};

class RRT
{
public:
    struct Tree
    {

        struct Node : public std::enable_shared_from_this<Node>
        {
            Node(Robot _position) : position(_position){};
            Node(const Node &_other) = delete;
            Node(Node &&_other) = delete;
            Node operator=(const Node &_other) = delete;
            Node operator=(Node &&_other) = delete;
            ~Node() = default;

            void add_childen(std::shared_ptr<Node> child);

            bool is_children(std::shared_ptr<Node> child);

            double distance(std::shared_ptr<Node> other);
            double distance(Robot other);
            std::weak_ptr<Node> get_parent();
            Robot get_position();
            void set_parent(std::weak_ptr<Node> parent);
            std::unordered_set<std::shared_ptr<Node>> children;

        private:
            std::weak_ptr<Node> _parent;
            Robot position;
        };

        Tree(Robot _position) : head(new Node(_position)){};
        Tree(const Tree &_other) = delete;
        Tree(Tree &&_other) = delete;
        Tree operator=(const Tree &_other) = delete;
        Tree operator=(Tree &&_other) = delete;
        ~Tree() = default;

        std::shared_ptr<Node> head;
    };

    RRT(double _tolerance, int other_dof, const Robot &inp_start,
        const GoalPoint &inp_goal, const std::vector<Polygon> &inp_obstacles) : tolerance(_tolerance), dof(other_dof), start(inp_start), goal(inp_goal),
                                                                                obstacles(inp_obstacles), tree(start), distance_from_closest(goal.distance(start, start.configuration)), probability_gen(0.0, 1.0)
    {
        std::random_device rd;
        std::mt19937 gen1(rd());
        gen = gen1;

        for (int i = 0; i < dof; i++)
        {
            std::uniform_real_distribution<> dis(start.joints[i].limits[0], start.joints[i].limits[1]);
            random_gen.push_back(dis);
        }
        InverseKinematics::sample_all_goals_parallel(end_configurations,start,goal,obstacles,2.0,10.0,1.0);
        // TODO: добавить проверку решений с помощью ПЗК

        if (end_configurations.size() == 0)
        {
            throw std::invalid_argument("No solution found for goal configuration.");
        }
        std::cout<<"Inverse Kinematics solved, got "<<end_configurations.size()<<" solutions"<<std::endl;
    }

    RRT(const RRT &_other) = delete;
    RRT(RRT &&_other) = delete;
    RRT operator=(const RRT &_other) = delete;
    RRT operator=(RRT &&_other) = delete;
    ~RRT() = default;

    void grow_tree();
    
    bool is_finished() const;
    Tree& get_tree();    
    int get_dof() const;
    std::vector<double> get_path() const;

private:
    void expand_to_goal();
    void expand_to_random();
    void sample_all_goals(std::vector<Robot> &answers, Robot pos, int depth, Vector2D intermediate_goal, double length_koef);
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
    float goal_bias = 0.3;
    std::vector<Polygon> obstacles;
    std::vector<std::weak_ptr<RRT::Tree::Node>> finish_node;
    Robot random_sample();
    std::shared_ptr<RRT::Tree::Node> nearest_neighbour(const Robot &pos);
    Tree tree;
    std::shared_ptr<RRT::Tree::Node> closest_to_goal_node = tree.head;
    float distance_from_closest;
    void save_tree();
    std::shared_ptr<RRT::Tree::Node> make_step(const std::shared_ptr<RRT::Tree::Node> node, const Robot &pos);
    bool is_goal(const std::shared_ptr<RRT::Tree::Node> node);
    std::vector<Robot> end_configurations;
};
