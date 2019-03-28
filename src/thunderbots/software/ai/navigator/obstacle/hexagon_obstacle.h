#pragma once
#include "ai/navigator/obstacle/obstacle.h"

class HexagonObstacle : public Obstacle {
public:
    HexagonObstacle() = delete;
    explicit HexagonObstacle(const Point& centre, double radius);
    const Polygon& getBoundaryPolygon() const override;
private:
    const Polygon polygon;
};
