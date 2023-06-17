#include <gtest/gtest.h>
#include "../include/robot.h"
#include "../include/obstacles.h"
#include "../include/parsing.h"
TEST(myfunctions, add)
{
    std::vector<Polygon> polygons;
    Robot_position start = Robot_position();
    GoalPoint goal(0.0, 0.0, 0.0, 0.0);
    
    EXPECT_EQ(xmlParser::read_scene("example.xml", polygons, start, goal), true);
    EXPECT_EQ(polygons.size(), 3);
    EXPECT_EQ(goal.angle1_, 80); 
    EXPECT_EQ(start.joints.size(), 4); 
    EXPECT_EQ(xmlParser::read_scene("error_example.xml", polygons, start, goal), false);

}

int main(int argc, char* argv[])
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}