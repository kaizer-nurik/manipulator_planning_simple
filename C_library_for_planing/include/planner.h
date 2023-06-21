#pragma once
#include "robot.h"
#include <unordered_set>
#include <queue>
#include <map>
#include <memory>

struct Node 
{
    std::vector<double> position;
    double gCost;
    double hCost;
    Node* parent;

    Node(const std::vector<double>& pos, double g, double h, Node* p) : position(pos), gCost(g), hCost(h), parent(p) {}

    double getFCost() const {
        return gCost + hCost;
    }
};

bool operator<(const Node& other) const {
        return getFCost() < other.getFCost();
    }
bool operator>(const Node& other) const {
    return getFCost() > other.getFCost();
}

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

class Planner {
public:
    Planner(std::string filename): filename_(filename){}

    void AStar(const Robot &robot, const Robot& goal, const std::vector<Polygon> &obstacles) 
    {
        const int g_units = 128;
        std::vector<double> deltas(robot.dof_, 0.0);
        for (auto i = 0u; i<robot.configuration.size(); i++)
        {
            deltas[i] = (robot.joints[i].limits[0] - robot.joints[i].limits[0])/g_units;
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

        std::priority_queue <Node, vector<int>, greater<int> > opened_nodes;
        std::map<std::vector<double>, std::shared_ptr<Node>> map_pq_opened;
        std::unordered_set<Node> closed_nodes;
        primitivemo


        // Define the possible movements (up, down, left, right, and diagonal)
    // const std::vector<Point> directions = {
    //     Point(0, -1), Point(0, 1), Point(-1, 0), Point(1, 0),
    //     Point(-1, -1), Point(-1, 1), Point(1, -1), Point(1, 1)
    // };

    // // Create a 2D vector to store the costs (gCost, hCost)
    // std::vector<std::vector<int>> costs(grid.size(), std::vector<int>(grid[0].size(), std::numeric_limits<int>::max()));

    // // Create a 2D vector to store the parents of each cell
    // std::vector<std::vector<Point>> parents(grid.size(), std::vector<Point>(grid[0].size(), Point(-1, -1)));

    // // Create a priority queue for open nodes
    // std::priority_queue<Node*, std::vector<Node*>, std::function<bool(Node*, Node*)>> openNodes(
    //     [](Node* a, Node* b) { return a->getFCost() > b->getFCost(); });

    // // Start node
    // Node* startNode = new Node(start, 0, calculateDistance(start, goal), nullptr);
    // costs[start.x][start.y] = 0;
    // openNodes.push(startNode);

    // while (!openNodes.empty()) {
    //     Node* current = openNodes.top();
    //     openNodes.pop();

    //     if (current->position.x == goal.x && current->position.y == goal.y) {
    //         // Reconstruct the path
    //         std::vector<Point> path;
    //         Node* node = current;
    //         while (node != nullptr) {
    //             path.push_back(node->position);
    //             node = node->parent;
    //         }
    //         std::reverse(path.begin(), path.end());
    //         return path;
    //     }

    //     for (const Point& direction : directions) {
    //         Point next(current->position.x + direction.x, current->position.y + direction.y);

    //         if (next.x < 0 || next.x >= grid.size() || next.y < 0 || next.y >= grid[0].size() || grid[next.x][next.y] == 1) {
    //             // Skip invalid or blocked cells if checkCollide()
    //             continue;
    //         }

    //         int newCost = current->gCost + 1; //

    //         if (newCost < costs[next.x][next.y]) {
    //             // Update the costs and parent of the next cell
    //             costs[next.x][next.y] = newCost;
    //             parents[next.x][next.y] = current->position;
    //             Node* nextNode = new Node(next, newCost, calculateDistance(next, goal), current);
    //             openNodes.push(nextNode);
    //         }
    //     }
    // }

    // // No path found
    // return std::vector<Point>();
        // Robot current(start);
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

    void RRT(const Robot &start, const Robot& goal, const std::vector<Polygon> &obstacles) 
    {

    }
    private:
    std::string filename_;
    const double eps = 1e-3;
};

