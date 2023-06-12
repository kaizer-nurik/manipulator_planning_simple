#pragma once

#include <vector>
#include <cmath>

class Robot {
public:
    Robot() : degreesOfFreedom(0) {}
    Robot(const Robot& other) 
    {
        for (Joint a:other.joints)
        {
            this->AddJoint(a.angle, a.length, a.limits);            
        }
    }

    struct Joint {
    double angle;
    double length;
    std::vector<double> limits;
    };

    void AddJoint(double angle, double length, const std::vector<double>& limits) {
        Joint joint;
        joint.angle = angle;
        joint.length = length;
        joint.limits = limits;

        joints.push_back(joint);
        degreesOfFreedom++;
    }

    void PrintJointDetails() const {
        std::cout << "Robot DOF: " << degreesOfFreedom << std::endl;

        for (const auto& joint : joints) {
            std::cout << "Joint Angle: " << joint.angle << std::endl;
            std::cout << "Joint Length: " << joint.length << std::endl;
            std::cout << "Joint Limits: ";
            
            for (const auto& limit : joint.limits) {
                std::cout << limit << " ";
            }

            std::cout << std::endl;
        }
    }
    std::vector<Joint> get_joints() const {return joints;};
    std::vector<Joint> joints;
    private:
    double degreesOfFreedom;
    


};

double distance(Robot robot1, Robot robot2)
{
    std::vector<Robot::Joint> joints1 = robot1.get_joints();
    std::vector<Robot::Joint> joints2 = robot2.get_joints();
    double dist = 0.0;
    if (joints1.size() == joints2.size())
    {
        for (auto i=0u; i<joints1.size(); i++)
        {
            dist+= std::pow((joints1[i].angle - joints1[i].angle), 2); 
        }
    }
    return std::sqrt(dist);
}