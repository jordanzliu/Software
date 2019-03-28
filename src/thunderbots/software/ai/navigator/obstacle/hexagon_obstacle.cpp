#include "ai/navigator/obstacle/hexagon_obstacle.h"

#include "hexagon_obstacle.h"

HexagonObstacle::HexagonObstacle(const Point &centre, double radius) :
    polygon({
        centre + Point::createFromAngle(Angle::zero()) * radius,
        centre + Point::createFromAngle(Angle::ofDegrees(60)) * radius,
        centre + Point::createFromAngle(Angle::ofDegrees(120)) * radius,
        centre + Point::createFromAngle(Angle::ofDegrees(180)) * radius,
        centre + Point::createFromAngle(Angle::ofDegrees(240)) * radius,
        centre + Point::createFromAngle(Angle::ofDegrees(300)) * radius
    })
{}

const Polygon &HexagonObstacle::getBoundaryPolygon() const {
    return polygon;
}

