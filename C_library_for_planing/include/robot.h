#pragma once

#include <vector>

struct Joint {
    double angle;
    double length;
    std::vector<double> limits;
};

class Robot {
private:
    double degreesOfFreedom;
    std::vector<Joint*> joints;

public:
    Robot() : degreesOfFreedom(0) {}

    void AddJoint(double angle, double length, const std::vector<double>& limits) {
        Joint* joint = new Joint;
        joint->angle = angle;
        joint->length = length;
        joint->limits = limits;

        joints.push_back(joint);
        degreesOfFreedom++;
    }

    void PrintJointDetails() const {
        std::cout << "Robot DOF: " << degreesOfFreedom << std::endl;

        for (const auto& joint : joints) {
            std::cout << "Joint Angle: " << joint->angle << std::endl;
            std::cout << "Joint Length: " << joint->length << std::endl;
            std::cout << "Joint Limits: ";
            
            for (const auto& limit : joint->limits) {
                std::cout << limit << " ";
            }

            std::cout << std::endl;
        }
    }

    ~Robot() {
        for (const auto& joint : joints) {
            delete joint;
        }
    }
};