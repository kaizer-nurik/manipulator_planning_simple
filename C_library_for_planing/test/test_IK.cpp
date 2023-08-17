#include <gtest/gtest.h>
#include <chrono>
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
#define TEST_IK_ITER_MAX 7
#define TEST_IK_ITER_STRESS 5

void test_IK(const uint8_t max_joint, const int max_iter){
    for (int iter=0;iter<max_iter;iter++)
    {
        
        std::vector<Polygon> polygons;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::vector<std::uniform_real_distribution<>> random_gen;
    std::uniform_real_distribution<> dis(-180,180);
    std::uniform_real_distribution<> dis_length(0.1,5);
    for (int i = 0; i < max_joint; i++)
    {

        GoalPoint goal(0.0, 0.0, 0.0, 0.0);

        std::vector<Robot> end_configurations;
        Robot sample;
        for (int joint_count = 0; joint_count <= i; joint_count++)
        {
            sample.AddJoint(dis_length(gen), 0.2, {-180, 180});
            sample.configuration.push_back(0);
        }
        std::cout<<"joint number "<< i+1<<std::endl;
        for (int j = 0; j <= i; j++)
        {

            sample.configuration[j] = dis(gen);
            std::cout<<"i: "<<j<<" angle: "<< sample.configuration[j] <<std::endl;
            std::cout<<"i: "<<j<<" length: "<< sample.joints[j].length <<std::endl;

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

            sample.configuration[j] = 0;
        }
        goal.angle1_ = angle;

        if (!collide(sample,sample.configuration,polygons));{
            InverseKinematics::sample_all_goals(end_configurations, sample, goal, polygons, 10);
        
            ASSERT_GT(end_configurations.size(), 0);
        }
        

    }
    }
}

void test_IK_parallel(const uint8_t max_joint, const uint8_t max_iter){
    for (int iter=0;iter<max_iter;iter++)
    {
        std::vector<Polygon> polygons;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::vector<std::uniform_real_distribution<>> random_gen;
    std::uniform_real_distribution<> dis(-180,180);
    std::uniform_real_distribution<> dis_length(0.1,5);
    for (int i = 0; i < max_joint; i++)
    {

        GoalPoint goal(0.0, 0.0, 0.0, 0.0);

        std::vector<Robot> end_configurations;
        Robot sample;
        for (int joint_count = 0; joint_count <= i; joint_count++)
        {
            sample.AddJoint(dis_length(gen), 0.2, {-180, 180});
            sample.configuration.push_back(0);
        }
        std::cout<<"joint number "<< i+1<<std::endl;
        for (int j = 0; j <= i; j++)
        {

            sample.configuration[j] = dis(gen);
            std::cout<<"i: "<<j<<" angle: "<< sample.configuration[j] <<std::endl;
            std::cout<<"i: "<<j<<" length: "<< sample.joints[j].length <<std::endl;
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

            sample.configuration[j] = 0;
        }
        goal.angle1_ = angle;

        
        InverseKinematics::sample_all_goals_parallel(end_configurations, sample, goal, polygons, 10);
        
        ASSERT_GT(end_configurations.size(), 0);

    }
    }
}
TEST(TEST_IK, CORRECTNESS)
{
    test_IK(TEST_IK_ITER_MAX,3);
    test_IK(TEST_IK_ITER_STRESS,100);
}
TEST(TEST_IK, CORRECTNESS_PARALLEL)
{
    test_IK_parallel(TEST_IK_ITER_MAX,3);
    test_IK_parallel(TEST_IK_ITER_STRESS,40);
}
TEST(TEST_IK, CORRECTNESS_MANUAL)
{
    Robot start = Robot();
    std::vector<Polygon> polygons;
    GoalPoint goal(0.0, 0.0, 0.0, 0.0);

        std::vector<Robot> end_configurations;
        Robot sample = Robot(start);
        for (int joint_count = 0; joint_count <= 2; joint_count++)
        {
            sample.AddJoint(1, 0.2, {-180, 180});
        }

            sample.configuration.push_back(53.9975);
            sample.configuration.push_back(-100.873);
            sample.configuration.push_back(59.0889);

       
        goal.goalpoint = end_effector(sample, sample.configuration);
        goal.angle2_ = 1000;
        goal.delta = 0.01;
        double angle = 0;
        for (auto joint_angle : sample.configuration)
        {
            angle += joint_angle;
        }

        goal.angle1_ = angle;

        InverseKinematics::sample_all_goals(end_configurations, sample, goal, polygons, 10);
        ASSERT_GT(end_configurations.size(), 0);
  
}

TEST(TEST_IK, CORRECTNESS_MANUAL_PARALLEL)
{
    Robot start = Robot();
    std::vector<Polygon> polygons;
    GoalPoint goal(0.0, 0.0, 0.0, 0.0);

        std::vector<Robot> end_configurations;
        Robot sample = Robot(start);
        for (int joint_count = 0; joint_count <= 2; joint_count++)
        {
            sample.AddJoint(1, 0.2, {-180, 180});
        }

            sample.configuration.push_back(53.9975);
            sample.configuration.push_back(-100.873);
            sample.configuration.push_back(59.0889);

       
        goal.goalpoint = end_effector(sample, sample.configuration);
        goal.angle2_ = 1000;
        goal.delta = 0.01;
        double angle = 0;
        for (auto joint_angle : sample.configuration)
        {
            angle += joint_angle;
        }

        goal.angle1_ = angle;

        if (!collide(sample,sample.configuration,polygons));{
            InverseKinematics::sample_all_goals_parallel(end_configurations, sample, goal, polygons, 10);
            ASSERT_GT(end_configurations.size(), 0);
        }
        
  
}

TEST(TEST_IK, CORRECTNESS_MANUAL_5)
{
    Robot start = Robot();
    std::vector<Polygon> polygons;
    GoalPoint goal(0.0, 0.0, 0.0, 0.0);

        std::vector<Robot> end_configurations;
        Robot sample = Robot(start);
        for (int joint_count = 0; joint_count <= 4; joint_count++)
        {
            sample.AddJoint(1, 0.2, {-180, 180});
        }

            sample.configuration.push_back(37.7854);
            sample.configuration.push_back(136.189);
            sample.configuration.push_back(25.1971);
            sample.configuration.push_back(149.376);
            sample.configuration.push_back(-83.6869);

       
        goal.goalpoint = end_effector(sample, sample.configuration);
        goal.angle2_ = 1000;
        goal.delta = 0.01;
        double angle = 0;
        for (auto joint_angle : sample.configuration)
        {
            angle += joint_angle;
        }

        goal.angle1_ = angle;

        InverseKinematics::sample_all_goals(end_configurations, sample, goal, polygons, 10);
        ASSERT_GT(end_configurations.size(), 0);
  
}

TEST(TEST_IK, EXECUTION_TIME)
{

    using std::chrono::high_resolution_clock;
    using std::chrono::duration_cast;
    using std::chrono::duration;
    using std::chrono::milliseconds;

    auto t1 = high_resolution_clock::now();
    test_IK(4,10);
    auto t2 = high_resolution_clock::now();

    /* Getting number of milliseconds as a double. */
    duration<double, std::milli> ms_double = t2 - t1;

    std::cout <<"до 4 звеньев, 10 итераций, время обычной программы: "<< ms_double.count() << "ms\n";
    
    t1 = high_resolution_clock::now();
    test_IK_parallel(4,10);
    t2 = high_resolution_clock::now();

    /* Getting number of milliseconds as a double. */
    ms_double = t2 - t1;

    std::cout <<"до 4 звеньев, 10 итераций, время многопоточной программы: "<< ms_double.count() << "ms\n";


    t1 = high_resolution_clock::now();
    test_IK(10,2);
    t2 = high_resolution_clock::now();

    /* Getting number of milliseconds as a double. */
    ms_double = t2 - t1;

    std::cout <<"до 10 звеньев, 2 итераций, время обычной программы: "<< ms_double.count() << "ms\n";
    
    t1 = high_resolution_clock::now();
    test_IK_parallel(10,2);
    t2 = high_resolution_clock::now();

    /* Getting number of milliseconds as a double. */
    ms_double = t2 - t1;

    std::cout <<"до 10 звеньев, 2 итераций, время многопоточной программы: "<< ms_double.count() << "ms\n";

    
}



TEST(TEST_IK, CORRECTNESS_MANUAL_20)
{
    Robot start = Robot();
    std::vector<Polygon> polygons;
    GoalPoint goal(0.0, 0.0, 0.0, 0.0);

        std::vector<Robot> end_configurations;
        Robot sample = Robot(start);
        sample.AddJoint(1.89601, 0.2, {-180, 180});
        sample.AddJoint(3.85691, 0.2, {-180, 180});
        sample.AddJoint(0.729285, 0.2, {-180, 180});
        sample.AddJoint(1.30756, 0.2, {-180, 180});
        

            sample.configuration.push_back(81.6052);
            sample.configuration.push_back(169.012);
            sample.configuration.push_back(-83.8549);
            sample.configuration.push_back(-38.8422);

        goal.goalpoint = end_effector(sample, sample.configuration);
        goal.angle2_ = 1000;
        goal.delta = 0.01;
        double angle = 0;
        for (auto joint_angle : sample.configuration)
        {
            angle += joint_angle;
        }

        goal.angle1_ = angle;

        InverseKinematics::sample_all_goals(end_configurations, sample, goal, polygons, 10);
        ASSERT_GT(end_configurations.size(), 0);
  
}






int main(int argc, char *argv[])
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}