#pragma once

#include <vector>
#include <cmath>
#include "obstacles.h"

struct Joint 
{
double length;
double width;
std::vector<double> limits;
};

class Robot_position
{
public:
    Robot_position() : dof_(0) {}

    Robot_position(const Robot_position& other) 
    {
        for (Joint a:other.joints)
        {
            this->AddJoint(a.length, a.width, a.limits);            
        }
        this->configuration = other.configuration;
    }
    Robot_position& operator=(const Robot_position& other){
        for (Joint a:other.joints)
        {
            this->AddJoint(a.length, a.width, a.limits);            
        }
        this->configuration = other.configuration;
    }

    Robot_position(Robot_position&& _other) = default; //move constructor
    Robot_position& operator=(Robot_position&& other) = default;//move asigment
    ~Robot_position()= default;;//Destructor

    void AddJoint(double length, double width, const std::vector<double>& limits) {
        Joint joint;
        joint.length = length;
        joint.width = width;
        joint.limits = limits;
        joints.push_back(joint);
        dof_++;
    }

    void printJointDetails() const {
        std::cout << "Robot_position DOF: " << dof_ << std::endl;

        for (const auto& joint : joints)
         {
            std::cout << "Joint length: " << joint.length << std::endl;
            std::cout << "Joint width: " << joint.width << std::endl;
            std::cout << "Joint limits: ";
            
            for (const auto& limit : joint.limits) {
                std::cout << limit << " ";
            }

            std::cout << std::endl;
        }
    }
    std::vector<Joint> get_joints() const {return joints;};
    std::vector<Joint> joints;
    std::vector<double> configuration;
    private:
    double dof_;
    


};

struct GoalPoint
{
    Vector2D goalpoint;
    double angle1_;
    double angle2_;

    GoalPoint(double x, double y, double angle1, double angle2): goalpoint(x, y), angle1_(angle1), angle2_(angle2){}
    
};

double distance(Robot_position robot1, Robot_position robot2)
{
    // std::vector<Robot_position::Joint> joints1 = robot1.get_joints();
    // std::vector<Robot_position::Joint> joints2 = robot2.get_joints();
    // double dist = 0.0;
    // if (joints1.size() == joints2.size())
    // {
    //     for (auto i=0u; i<joints1.size(); i++)
    //     {
    //         dist+= std::pow((joints1[i].angle - joints1[i].angle), 2); 
    //     }
    // }
    // return std::sqrt(dist);
    return 0.0;
}

int s(int a, int b)
{
    return a+b;
}