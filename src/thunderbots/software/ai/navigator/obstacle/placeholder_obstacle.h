#pragma once

#include "../shared/constants.h"
#include "ai/navigator/obstacle/obstacle.h"

// Placeholder obstacle represents a stationary robot centred at (0,0)
class PlaceholderObstacle : public Obstacle
{
   public:
    explicit PlaceholderObstacle(double radius_scaling);

    /*
     * Gets the boundary polygon around the centre
     *
     * @return a six-sided Polygon to represent the boundary around the obstacle
     */
    const Polygon& getBoundaryPolygon() const override;
private:
    Polygon polygon;
};
