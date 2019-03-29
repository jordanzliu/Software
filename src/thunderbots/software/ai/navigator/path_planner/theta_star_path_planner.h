#pragma once
#include "ai/navigator/path_planner/path_planner.h"

class ThetaStarPathPlanner : public PathPlanner
{
   public:
    std::optional<std::vector<Point>> findPath(
        const Point &start, const Point &dest, std::vector<std::unique_ptr<Obstacle>>&& obstacles,
        const ViolationFunction &violation_function) override;
};