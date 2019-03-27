#pragma once
#include "ai/navigator/path_planner/path_planner.h"

class ThetaStarPathPlanner : public PathPlanner
{
   public:
    std::optional<std::vector<Point>> findPath(
        const Point &start, const Point &dest, const std::vector<Obstacle> &obstacles,
        const ViolationFunction &violation_function) override;
};