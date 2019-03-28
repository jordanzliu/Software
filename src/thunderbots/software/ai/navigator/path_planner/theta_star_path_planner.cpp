#include "ai/navigator/path_planner/theta_star_path_planner.h"

#include <dosl/dosl>

namespace
{
    bool intersectsAny(const Segment &seg, const std::vector<Obstacle> obstacles)
    {
        for (const auto &obstacle : obstacles)
        {
            if (obstacle.getBoundaryPolygon())
        }
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

        const Point &point()
        {
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
            for (const auto &obstacle : obstacles)
            {
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
