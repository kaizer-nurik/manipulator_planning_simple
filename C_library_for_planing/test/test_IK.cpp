#include <gtest/gtest.h>

#include <iostream>
#include <fstream>
#include <string>
#include <memory>
#include "rapidxml.hpp"
#include "obstacles.h"
#include "robot.h"
#include "parsing.h"
#include "planner.h"
#include "geometry.h"
#include <vector>
#include "file_saver.h"
#include <random>
#include "inverse_kinematics.h"
#define TEST_IK_ITER 10

TEST(TEST_IK, CORRECTNESS)
{
    std::vector<Polygon> polygons;
    Robot start = Robot();
    std::random_device rd;
    std::mt19937 gen(rd());
    std::vector<std::uniform_real_distribution<>> random_gen;
    std::uniform_real_distribution<> dis(-180,180);

    for (int i = 0; i < TEST_IK_ITER; i++)
    {

        GoalPoint goal(0.0, 0.0, 0.0, 0.0);

        std::vector<Robot> end_configurations;
        Robot sample = Robot(start);
        for (int joint_count = 0; joint_count <= i; joint_count++)
        {
            sample.AddJoint(1, 0.2, {-180, 180});
            sample.configuration.push_back(0);
        }

        for (int j = 0; j <= i; j++)
        {

            sample.configuration[j] = dis(gen);
            std::cout<<"i: "<<j<<" angle: "<< sample.configuration[j] <<std::endl;
        }
        goal.goalpoint = end_effector(sample, sample.configuration);
        goal.angle2_ = 1000;
        goal.delta = 0.01;
        double angle = 0;
        for (auto joint_angle : sample.configuration)
        {
            angle += joint_angle;
        }

        for (int j = 0; j <= i; j++) // инициализируем случайный старт
        {

            sample.configuration[j] = dis(gen);
        }
        goal.angle1_ = angle;

        InverseKinematics::sample_all_goals(end_configurations, sample, goal, polygons, 10.0, 10.0, 1.0);
        std::cout<<i<<std::endl;
        ASSERT_GT(end_configurations.size(), 0);
    }
}

TEST(TEST_IK, CORRECTNESS_PARRALLEL)
{
}

int main(int argc, char *argv[])
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}