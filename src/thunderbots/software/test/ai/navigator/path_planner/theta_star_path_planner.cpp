#include <gtest/gtest.h>
#include "ai/navigator/path_planner/theta_star/theta_star_path_planner.h"

template <typename T>
std::ostream& operator<<(std::ostream& os, std::vector<T> vec) {
    os << "{";
    for (const T& item : vec) {
        os << item << ", ";
    }
    os << "}";
    return os;
}

TEST(TestThetaStarPathPlanner, test_1)
{
    Robot obstacle_robot(
            0,
            Point(0, 0),
            Vector(0, 0),
            Angle(),
            AngularVelocity(),
            Timestamp()
            );

    Obstacle obstacle = Obstacle::createRobotObstacle(obstacle_robot, false);
    std::vector<Obstacle> obstacles = {obstacle};

    ThetaStarPathPlanner planner;
    auto path = planner.findPath(Point(-1, 0), Point(1, 0), obstacles, [](const auto&){return 0;});

    std::cout << path.value();
}

int main(int argc, char **argv)
{
    std::cout << argv[0] << std::endl;
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}