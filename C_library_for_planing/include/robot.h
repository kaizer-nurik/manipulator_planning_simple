#pragma once

#include <vector>


class Robot {
public:
    Robot() : degreesOfFreedom(0) {}

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
    private:
    double degreesOfFreedom;
    std::vector<Joint> joints;


};