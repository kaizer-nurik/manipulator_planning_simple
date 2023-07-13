#include <gtest/gtest.h>
#include "../include/robot.h"
#include "../include/obstacles.h"
#include "../include/parsing.h"

TEST(myfunctions, add)
{
    std::vector<Polygon> polygons;
    Robot start = Robot();
    GoalPoint goal(0.0, 0.0, 0.0, 0.0);
    
    EXPECT_EQ(read_scene("example.xml", polygons, start, goal), true);
    EXPECT_EQ(polygons.size(), 3);
    EXPECT_EQ(goal.angle1_, 80); 
    EXPECT_EQ(start.joints.size(), 4); 
    EXPECT_EQ(read_scene("error_example.xml", polygons, start, goal), false);

}

TEST(test1, t1)
{
    EXPECT_EQ(polygons_collide(Polygon({ Vector2D(-5, 3), Vector2D(1, 8), Vector2D(4, 5), Vector2D(4, 0), Vector2D(0, 0) }),
     Polygon({ Vector2D(2, 8), Vector2D(3, 9), Vector2D(5, 6),  Vector2D(-2, 0)})), true);

     EXPECT_EQ(polygons_collide(Polygon({ Vector2D(0, 0), Vector2D(-1, 0), Vector2D(-1, 1), Vector2D(0,0.99)}),
     Polygon({ Vector2D(0, 1.01), Vector2D(0, 2), Vector2D(2, 1),  Vector2D(1, 1)})), false);

    EXPECT_EQ(polygons_collide(Polygon({ Vector2D(0, 0), Vector2D(-1, 0), Vector2D(-1, 1), Vector2D(0,1)}),
    Polygon({ Vector2D(0, 1), Vector2D(0, 2), Vector2D(2, 1),  Vector2D(1, 1)})), true);

    EXPECT_EQ(polygons_collide(Polygon({ Vector2D(0, 0), Vector2D(-2, 0), Vector2D(-2, 2), Vector2D(-1,1), Vector2D(0,2)}),
    Polygon({ Vector2D(-1, 3), Vector2D(0, 5), Vector2D(3, 5),  Vector2D(1, 1),  Vector2D(1, 4)})), true);

    EXPECT_EQ(isConvexPolygon(Polygon({ Vector2D(0, 0), Vector2D(-1, 0), Vector2D(-1, 1), Vector2D(0,1)})), true);
}

int main(int argc, char* argv[])
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}