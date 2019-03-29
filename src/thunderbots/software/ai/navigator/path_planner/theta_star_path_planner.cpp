#include "ai/navigator/path_planner/theta_star_path_planner.h"

#include <dosl/dosl>

namespace
{
    inline const bool intersectsAny(const Segment &seg, const std::vector<Obstacle>& obstacles)
    {
        for (const auto &obstacle : obstacles)
        {
            if (obstacle.getBoundaryPolygon().intersects(seg)) {
                return false;
            }
        }
        return true;
    }

    using cost_type = double;

    class PointNode : public ThetaStar::Node<PointNode, cost_type>
    {
       public:
        PointNode() : _point(){};
        explicit PointNode(const Point &point)
            : _point(point), ThetaStar::Node<PointNode, cost_type>(){};

        ~PointNode() = default;

        bool operator==(const PointNode &other) const
        {
            return _point == other._point;
        }

        const Point & point() const {
            return _point;
        }

       private:
        const Point _point;
    };

    class ThetaStarSearch : public ThetaStar::Algorithm<PointNode, double>
    {
       public:
        ~ThetaStarSearch() override = default;
        explicit ThetaStarSearch(const Point &dest, std::vector<Obstacle> &&_obstacles)
            : dest_node(dest), obstacles(std::move(_obstacles)){};

        void getSuccessors(PointNode &node, std::vector<PointNode> *neighbours,
                           std::vector<cost_type> *neighbour_costs) override
        {
            // unfortunately this is O(n^3) - could probably be reduced down if we had a function in
            // polygon that returns the distance of the point furthest from the mean of the points
            // so that we can cull polygons that have no chance to intersect
            for (const auto& obstacle : obstacles) {
                for (const auto& point : obstacle.getBoundaryPolygon().getPoints()) {
                    if (node.point() == point) continue;
                    if (!intersectsAny(Segment{node.point(), point}, obstacles)) {
                        neighbours->emplace_back(PointNode(point));
                        neighbour_costs->emplace_back(dist(node.point(), point));
                    }
                }
            }

            // check if we can reach destination and add to successors
            if (!intersectsAny(Segment{node.point(), dest_node.point()}, obstacles)) {
                neighbours->emplace_back(dest_node.point());
                neighbour_costs->emplace_back(dist(node.point(), dest_node.point()));
            }
        }

       private:
        const PointNode dest_node;
        const std::vector<Obstacle> obstacles;
    };
}  // namespace


std::optional<std::vector<Point>> ThetaStarPathPlanner::findPath(
    const Point &start, const Point &dest, const std::vector<Obstacle> &obstacles,
    const ViolationFunction &violation_function)
{
    return std::nullopt;
}
