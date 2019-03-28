#include "ai/navigator/obstacle/placeholder_obstacle.h"

namespace
{
}

PlaceholderObstacle::PlaceholderObstacle(double radius_scaling):
    polygon({
        Point::createFromAngle(Angle::zero()) * ROBOT_MAX_RADIUS_METERS * radius_scaling,
        Point::createFromAngle(Angle::ofDegrees(60)) * ROBOT_MAX_RADIUS_METERS * radius_scaling,
        Point::createFromAngle(Angle::ofDegrees(120)) * ROBOT_MAX_RADIUS_METERS * radius_scaling,
        Point::createFromAngle(Angle::ofDegrees(180)) * ROBOT_MAX_RADIUS_METERS * radius_scaling,
        Point::createFromAngle(Angle::ofDegrees(240)) * ROBOT_MAX_RADIUS_METERS * radius_scaling,
        Point::createFromAngle(Angle::ofDegrees(300)) * ROBOT_MAX_RADIUS_METERS * radius_scaling
    })
{}

const Polygon &PlaceholderObstacle::getBoundaryPolygon() const
{
    return polygon;
}
