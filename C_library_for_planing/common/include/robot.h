#pragma once

#include <vector>
#include <cmath>
#include "obstacles.h"
#include <iostream>
#include <chrono>
#include <typeinfo>
#include <random>
#include <vector>

class Timer1 
{
private:
    std::chrono::time_point<std::chrono::steady_clock> start_time;
    std::chrono::duration<double> paused_duration;
    bool is_running;
    std::string name;

public:
    Timer1(const std::string &name_) : name(name_), is_running(false)
    {
        start();
    }

    ~Timer1() 
    {
        pause();
        //std::cout << "Timer " << name << " stopped.Elapsed time : " << getElapsedTime() << " seconds" << std::endl;
    }

    void start()
    {
        if (!is_running) 
        {
            start_time = std::chrono::steady_clock::now();
            paused_duration = std::chrono::duration<double>::zero();
            is_running = true;
            //std::cout << "Started timer " << name << std::endl;
        }
    }

    void pause() 
    {
        if (is_running) 
        {
            auto end_time = std::chrono::steady_clock::now();
            paused_duration += end_time - start_time;
            is_running = false;
        }
    }

    double getElapsedTime() 
    {
        if (is_running)
        {
            auto current_time = std::chrono::steady_clock::now();
            return std::chrono::duration_cast<std::chrono::duration<double>>(current_time - start_time - paused_duration).count();
        }
        else 
        {
            return std::chrono::duration_cast<std::chrono::duration<double>>(paused_duration).count();
        }
    }
};

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

    Robot(const Robot& other) ;

    void AddJoint(double length, double width, const std::vector<double>& limits);

    void printJointDetails() const ;
    std::vector<Joint> get_joints() const {return joints;};
    std::vector<Joint> joints;
    std::vector<double> configuration;
    double dof_;
    Robot operator-(Robot const rhs) const;
    Robot operator+(Robot const rhs) const;

    Robot operator/(double const rhs) const;
    Robot operator*(double const rhs) const;

    double distance(const Robot& other) const;

};

struct GoalPoint
{
    Vector2D goalpoint;
    double angle1_;
    double angle2_;
    double delta = 0.1;
    bool is_goal(const double& x, const double& y, const double& angle) const;
    bool is_goal(const Robot& pos)const ;
    float distance(const Robot &robot, const std::vector<double> config) const;
    GoalPoint(double x, double y, double angle1, double angle2): goalpoint(x, y), angle1_(angle1), angle2_(angle2){};
};

Vector2D end_effector(const Robot &robot, const std::vector<double>& config);

bool collide(const Robot &robot, const std::vector<double> config, const std::vector<Polygon> &poligons);
double fix_fmod(double angle);