#include "theta_star_path_planner.h"
#include <dosl/dosl>

namespace {
    using cost_type = double;

    class PointNode : public ThetaStar::Node<PointNode, cost_type> {
    public:
        PointNode() : _point() {};

        explicit PointNode(const Point &point) : _point(point) {};

        bool operator==(const PointNode &other) { return _point == other._point; }

        const Point &point() const { return _point; }

    private:
        Point _point;
    };

    class ThetaStarSearch : public ThetaStar::Algorithm<PointNode, cost_type>
    {
    public:
        explicit ThetaStarSearch(const Point& point, const std::vector<Obstacle>& the_obstacles) :
        dest_node(point), obstacles(&the_obstacles) {};

        void getSuccessors(PointNode& node, std::vector<PointNode>* successors,
                            std::vector<double>* successor_costs) override {
            // TODO: this entire method
        }

        bool stopSearch(PointNode& node) override
        {
            return node == dest_node;
        }

        virtual ~ThetaStarSearch() = default;

    private:
        PointNode dest_node;
        // we are using a raw pointer to be able to "own" a reference to a
        // vector of Obstacles, and avoid a deep copy
        const std::vector<Obstacle>* const obstacles;
    };
}

std::optional <std::vector<Point>>
ThetaStarPathPlanner::findPath(const Point &start, const Point &dest, const std::vector <Obstacle> &obstacles,
                               const ViolationFunction &violation_function) {
    return std::nullopt;
}
