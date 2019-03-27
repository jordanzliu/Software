#include "ai/navigator/path_planner/theta_star_path_planner.h"

#include <dosl/dosl>

namespace
{
    using cost_type = double;

    class PointNode : public ThetaStar::Node<PointNode, cost_type>
    {
       public:
        PointNode() = delete;
        explicit PointNode(const Point &point) : _point(point){};

        bool operator==(const PointNode &other)
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

    class Planner : public ThetaStar::Algorithm<PointNode, double>
    {
       public:
        Planner() = delete;
        explicit Planner(const Point &dest) : dest_node(dest){};

       private:
        ~Planner() = default;
        const PointNode dest_node;
    };
}  // namespace


std::optional<std::vector<Point>> ThetaStarPathPlanner::findPath(
    const Point &start, const Point &dest, const std::vector<Obstacle> &obstacles,
    const ViolationFunction &violation_function)
{
    return std::nullopt;
}
