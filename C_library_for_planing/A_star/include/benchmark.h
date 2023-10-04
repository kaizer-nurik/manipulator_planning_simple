#pragma once
#include "obstacles.h"
#include "planner.h"
#include "parsing.h"
#include "robot.h"
#include <chrono>
#include <typeinfo>
#include <random>
#include <vector>

class Timer 
{
private:
    std::chrono::time_point<std::chrono::steady_clock> start_time;
    std::chrono::duration<double> paused_duration;
    bool is_running;
    std::string name;

public:
    Timer(const std::string &name_) : name(name_), is_running(false)
    {
        start();
    }

    ~Timer() 
    {
        pause();
        std::cout << "Timer " << name << " stopped.Elapsed time : " << getElapsedTime() << " seconds" << std::endl;
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

