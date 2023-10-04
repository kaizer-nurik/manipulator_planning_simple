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
#include "json.hpp"
#include <list>

const double PI = M_PI;

template <typename T>
struct MathVector : public std::vector<T>
{

    using std::vector<T>::vector; // Using std::vector constructor
    MathVector(const std::vector<T> inp_v)
    {
        for (auto it : inp_v)
        {
            this->push_back(it);
        }
    }
    MathVector<T> operator+(const MathVector<T> other) const
    {
        MathVector<T> result;
        if (this->size() != other.size())
        {
            std::cout << "Ошибка: векторы имеют разные размеры!" << std::endl;
            return result;
        }
        for (std::size_t i = 0; i < this->size(); ++i)
        {
            result.push_back((*this)[i] + other[i]);
        }
        return result;
    };

    MathVector<T> operator*(const T &scalar) const
    {
        MathVector<T> result;
        for (const T &value : this)
        {
            result.push_back(value * scalar);
        }

        return result;
    };
};

namespace
{
    struct Node
    {
        typedef Node *Nodetype;

        MathVector<int> position;
        double gCost;
        double hCost;
        Node::Nodetype parent;

        Node(const MathVector<int> &pos, double g, double h, const Node::Nodetype &p) : position(pos), gCost(g), hCost(h), parent(p) {}

        double getFCost() const;

        bool operator<(const Node &other) const;
        bool operator>(const Node &other) const;
        bool operator==(const Node &other) const;
    };

    struct CompareNodes
    {
        bool operator()(const Node::Nodetype &node1, const Node::Nodetype &node2) const;
    };

    struct HashNode
    {
        size_t operator()(const Node::Nodetype &node) const;
    };

    // Define an equality function for the shared pointers to nodes
    struct EqualNode
    {
        bool operator()(const Node::Nodetype &node1, const Node::Nodetype &node2) const;
    };

    struct CompareKey
    {
        bool operator()(const MathVector<int> &key1, const MathVector<int> &key2) const;
    };

    template <typename T>
    void print_vector(MathVector<T> v);

    void reconstruct_path(const Node::Nodetype &node, std::string input_filename, std::string output_filename, const Robot &robot, const MathVector<double> &deltas);

    double periodic(const double &n, const double &L);

    void simplify(MathVector<int> &v, int n);

    double calculateDistance(const Vector2D &a, const Vector2D &b);

    double calculateDistance2(const Vector2D &a, const Vector2D &b);

    MathVector<double> calc_angles(const Robot &robot, const MathVector<int> &position, const MathVector<double> &deltas);

    double last_angle(const MathVector<double> &angles);

    void generateVectors(MathVector<MathVector<int>> &result, MathVector<int> &current, int index);

    template <typename T>
    int signum(T number);

    double norm1(const MathVector<double> &v1, const MathVector<double> &v2);

    double norm0(const MathVector<double> &v1, const MathVector<double> &v2);
}

class Planner_A_star
{
public:
    Planner_A_star(std::string input_filename, std::string output_filename) : input_filename_(input_filename), output_filename_(output_filename) {}

    double heuristic(const Robot &robot, const GoalPoint &goalpoint, const MathVector<double> &angles);

    double geuristic(const Robot &robot, const MathVector<double> &angles1, const MathVector<double> &angles2);

    bool AStar(const Robot &robot, const GoalPoint &goalpoint, const MathVector<Polygon> &obstacles, std::map<std::string, double> &dictionary);

    bool coll_test(const Robot &robot, const MathVector<Polygon> &obstacles);

private:
    std::string input_filename_;
    std::string output_filename_;
    const double eps = 1e-3;
};

void writeDataToJson(int test, int g_units, double coord_tolerance, double angle_tolerance, double time, double coll_check_percentage, int opened_nodes, int closed_nodes, double g_cost, int turn_numbers, const std::string &filename);
