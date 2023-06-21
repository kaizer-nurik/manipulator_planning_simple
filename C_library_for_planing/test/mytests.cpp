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
<<<<<<< HEAD
    EXPECT_EQ(read_scene("error_example.xml", polygons, start, goal), false);

}

TEST(test1, t1)
{
    EXPECT_EQ(checkCollision(Polygon({ Vector2D(-5, 3), Vector2D(1, 8), Vector2D(4, 5), Vector2D(4, 0), Vector2D(0, 0) }),
     Polygon({ Vector2D(2, 8), Vector2D(3, 9), Vector2D(5, 6),  Vector2D(-2, 0)})), true);

     EXPECT_EQ(checkCollision(Polygon({ Vector2D(0, 0), Vector2D(-1, 0), Vector2D(-1, 1), Vector2D(0,0.99)}),
     Polygon({ Vector2D(0, 1.01), Vector2D(0, 2), Vector2D(2, 1),  Vector2D(1, 1)})), false);

    EXPECT_EQ(checkCollision(Polygon({ Vector2D(0, 0), Vector2D(-1, 0), Vector2D(-1, 1), Vector2D(0,1)}),
    Polygon({ Vector2D(0, 1), Vector2D(0, 2), Vector2D(2, 1),  Vector2D(1, 1)})), true);

    EXPECT_EQ(checkCollision(Polygon({ Vector2D(0, 0), Vector2D(-2, 0), Vector2D(-2, 2), Vector2D(-1,1), Vector2D(0,2)}),
    Polygon({ Vector2D(-1, 3), Vector2D(0, 5), Vector2D(3, 5),  Vector2D(1, 1),  Vector2D(1, 4)})), true);

    EXPECT_EQ(isConvexPolygon(Polygon({ Vector2D(0, 0), Vector2D(-1, 0), Vector2D(-1, 1), Vector2D(0,1)})), true);
<<<<<<< HEAD
<<<<<<< HEAD

    EXPECT_EQ(isConvexPolygon(Polygon({ Vector2D(0, 0), Vector2D(1, 1), Vector2D(1, 0), Vector2D(0,0), 
    Vector2D(-1,0), Vector2D(-1,-1)})), false);
=======
    EXPECT_EQ(xmlParser::read_scene("error_example.xml", polygons, start, goal), false);
>>>>>>> fdb7aa87893498a1a8b7b8abfba282f7874ca4b3

    //EXPECT_EQ(checkCollision(Polygon({ Vector2D(0, 0), Vector2D(-1, 0), Vector2D(-2, 1), Vector2D(-2,2), Vector2D(-1,3), Vector2D(0,3),
    //Vector2D(1,2), Vector2D(1,2)}), Polygon({ Vector2D(10, 0), Vector2D(9, 0), Vector2D(8, 1), Vector2D(8,2), Vector2D(9,3), Vector2D(10,3),
    //Vector2D(11,2), Vector2D(11,2)})), true);
=======
>>>>>>> parent of 626851b (dd)
=======
>>>>>>> parent of 626851b (dd)
}

int main(int argc, char* argv[])
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}