#pragma once
#include "robot.h"

class Planner {
public:
    Planner(std::string filename): filename_(filename){}

    void motionPlanning(const Robot &start, const Robot& goal, const std::vector<std::shared_ptr<Obstacle>> &obstacles) 
    {
        Robot current(start);
        std::ofstream myfile;
        myfile.open (filename_);
           
        for (int i=0; i<start.get_joints().size(); i++)
        {
            while (std::abs(current.joints[i].angle - goal.joints[i].angle)>eps)
            {
                current.joints[i].angle -= eps * ((current.joints[i].angle - goal.joints[i].angle > 0.0) 
                - (current.joints[i].angle - goal.joints[i].angle < 0));
                std::cout << current.joints[i].angle << ' ' << goal.joints[i].angle << std::endl;

            for (int i=0; i<current.get_joints().size(); i++)
            {
                myfile <<  current.joints[i].angle << ',';
            }
            myfile << '\n';
            }

            std::cout << distance(current, goal) << std::endl;
        
        }
        myfile.close();

    }
    private:
    std::string filename_;
    const double eps = 1e-3;
};