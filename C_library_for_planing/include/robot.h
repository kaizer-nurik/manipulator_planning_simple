#pragma once

#include <vector>
#include <cmath>
#include "obstacles.h"
#include <iostream>

struct Joint 
{
double length;
double width;
std::vector<double> limits;
};

class Robot
{
public:
    Robot() : dof_(0) {}

    Robot(const Robot& other) 
    {
        for (Joint a:other.joints)
        {
            this->AddJoint(a.length, a.width, a.limits);            
        }
        this->configuration = other.configuration;
    }

    void AddJoint(double length, double width, const std::vector<double>& limits) {
        Joint joint;
        joint.length = length;
        joint.width = width;
        joint.limits = limits;
        joints.push_back(joint);
        dof_++;
    }

    void printJointDetails() const {
        std::cout << "Robot DOF: " << dof_ << std::endl;

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
    double dof_;
    


};

struct GoalPoint
{
    Vector2D goalpoint;
    double angle1_;
    double angle2_; //tolerance

    GoalPoint(double x, double y, double angle1, double angle2): goalpoint(x, y), angle1_(angle1), angle2_(angle2){}
};

bool collide(const Robot &robot, const std::vector<double> angles, const std::vector<Polygon> &poligons)
{
    bool flag = false;
    double x0 = 0.0;
    double y0 = 0.0;
    double angle = 0.0;
    std::vector<Polygon> joints;
    for (int i=0; i<robot.dof_; i++)
    {
        angle += angles[i];
        double x1 = x0 - sin(angle)*robot.joints[i].width;
        double y1 = y0 + cos(angle)*robot.joints[i].width;
        double x2 = x0 - sin(angle)*robot.joints[i].width + cos(angle)*robot.joints[i].length;
        double y2 = y0 + cos(angle)*robot.joints[i].width + sin(angle)*robot.joints[i].length;
        double x3 = x0 + sin(angle)*robot.joints[i].width + cos(angle)*robot.joints[i].length;
        double y3 = y0 - cos(angle)*robot.joints[i].width + sin(angle)*robot.joints[i].length;
        double x4 = x0 + sin(angle)*robot.joints[i].width;
        double y4 = y0 - cos(angle)*robot.joints[i].width;
        joints.push_back(Polygon({Vector2D(x1, y1), Vector2D(x2, y2), Vector2D(x3, y3), Vector2D(x4, y4)}));
        x0+=cos(angle)*robot.joints[i].length;
        y0+=sin(angle)*robot.joints[i].length;
    }


    for (int i=0; i<joints.size(); i++)
    {
        for (int j=0; j<poligons.size(); j++)
        {
            flag+=!(checkCollision(joints[i], poligons[j]));
            if (flag == true) return flag;
        }
    }

    for (int i=0; i<joints.size(); i++)
    {
        for (int j=0; j<joints.size(); j++)
        {
            if (std::abs(j-i)<1) continue;
            flag+=!(checkCollision(joints[i], joints[j]));
            if (flag == true) return flag;
        }
    }
    return flag;
}

Vector2D end_effector(const Robot &robot, const std::vector<double> &config)
{
    Vector2D ef(0.0, 0.0);
    double angle = 0.0;
    for (auto i=0u; i<robot.dof_; i++)
    {
        angle+=config[i];
        ef.x += robot.joints[i].length * cos(angle);
        ef.y += robot.joints[i].length * sin(angle);
    }
    return ef;
}
