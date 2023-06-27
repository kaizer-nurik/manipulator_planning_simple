#include "planner.h"
#include "robot.h"
#include <assert.h>
#include <random>
#include <vector>
#include <deque>

void RRT::Tree::Node::add_childen(std::shared_ptr<Node> child)
{
    assert(!this->is_children(child));
    assert(child); // проверка на nullptr
    children.insert(child);
    child->set_parent(weak_from_this());
}

void RRT::Tree::Node::set_parent(std::weak_ptr<Node> parent)
{
    _parent = parent;
}

bool RRT::Tree::Node::is_children(std::shared_ptr<Node> child)
{
    return children.count(child);
}
double RRT::Tree::Node::distance(std::shared_ptr<Node> other)
{
    std::vector<double> n1 = get_position().configuration;
    std::vector<double> n2 = (*other).get_position().configuration;
    double result = 0;
    for (int i = 0; i < n1.size(); i++)
    {
        result += (n1[i] - n2[i]) * (n1[i] - n2[i]);
    }
    return result;
}
double RRT::Tree::Node::distance(Robot other)
{
    std::vector<double> n1 = other.configuration;
    std::vector<double> n2 = other.configuration;
    double result = 0;
    for (int i = 0; i < n1.size(); i++)
    {
        result += (n1[i] - n2[i]) * (n1[i] - n2[i]);
    }
    return result;
}

std::weak_ptr<RRT::Tree::Node> RRT::Tree::Node::get_parent()
{
    return this->_parent;
}
Robot RRT::Tree::Node::get_position()
{
    return position;
}

void RRT::grow_tree()
{
    Robot random_position = random_sample();
    std::shared_ptr<RRT::Tree::Node> nearest_node = nearest_neighbour(random_position);
    std::shared_ptr<RRT::Tree::Node> next_node = make_step(nearest_node,random_position);
    if (next_node){
        nearest_node->add_childen(next_node);
        if (is_goal(next_node))
        {
            finished = true;
            finish_node.push_back(next_node);
        }
    }
    
    iteration_count++;
}
bool RRT::is_finished()
{
    return finished;
}


Robot RRT::random_sample(){
    Robot sample = Robot(start);
    std::random_device rd;  
    std::mt19937 gen(rd()); 
    for(int i=0;i<start.configuration.size();i++){
        std::uniform_real_distribution<> dis(start.joints[i].limits[0], start.joints[i].limits[1]);
        sample.configuration[i] = dis(gen);
    }
    return sample;
}
std::shared_ptr<RRT::Tree::Node> RRT::nearest_neighbour(const Robot &pos){
    // Определение ближайшего соседа полным перебором
    double min_distance = tree.head->distance(pos);
    std::shared_ptr<RRT::Tree::Node> min_node = tree.head;
    std::deque<std::shared_ptr<RRT::Tree::Node>> to_go;
    for(auto child:(*min_node).children){
        to_go.push_back(child);
    }
    while (to_go.size()>0){
        if (min_distance > (*to_go[0]).distance(pos)){
            min_distance = (*to_go[0]).distance(pos);
            min_node = to_go[0];
        }
        for(auto child:to_go[0]->children){
        to_go.push_back(child);
    }
        to_go.pop_front();
    }
    return min_node;
}

std::shared_ptr<RRT::Tree::Node> RRT::make_step(const std::shared_ptr<RRT::Tree::Node> node,const Robot& pos){
    //Делаем фиксированный шаг в сторону точки
    // проверяем на колизион, если есть, то ограничиваем до стенки
    Robot delta =  (pos - (*node).get_position())/(*node).distance(pos); // нормированный вектор от ноды по рандомной позиции
    Robot new_pose = (*node).get_position()+delta*tolerance;
    if (collide(new_pose,new_pose.configuration,obstacles)){
        std::shared_ptr<RRT::Tree::Node> null_node;
        return null_node;
    }
    std::shared_ptr<RRT::Tree::Node> new_node(new RRT::Tree::Node(new_pose));
    return new_node;

}
bool RRT::is_goal(std::shared_ptr<RRT::Tree::Node> node){
    Robot pos = (*node).get_position();
    Vector2D q = end_effector(pos,pos.configuration);
    double angle = 0.0;
    for (auto i=0u; i<pos.dof_; i++)
    {
        angle+=pos.configuration[i];

    }
    return goal.is_goal(q.x, q.y,angle);
}

bool collide(const Robot &robot, const std::vector<double> config, const std::vector<Polygon> &poligons)
{
    return false;
}

