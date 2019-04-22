#include "theta_star_path_planner.h"
#include <dosl/dosl>

namespace {
    using cost_type = double;

    class PointNode : public ThetaStar::Node<PointNode, cost_type> {
    public:
        PointNode() : _point() {};

        explicit PointNode(const Point &point) : _point(point) {};

        bool operator==(const PointNode &other) const { return _point == other._point; }

        const Point &point() const { return _point; }

    private:
        Point _point;
    };

    class ThetaStarSearch : public ThetaStar::Algorithm<PointNode, cost_type>
    {
    public:
        // TODO: constructor needs to also take a ViolationFunction argument
        explicit ThetaStarSearch(const Point& start, const Point& dest,
                const std::vector<Obstacle>& the_obstacles) :
                start_node(start), dest_node(dest), obstacles(&the_obstacles) {};

        std::vector<PointNode> getStartNodes() override;

        /**
         * This function returns the nodes reachable from a given node, as well as
         * the cost associated with each node.
         *
         * @param node The node we are currently at
         * @param successors a pointer to a vector of PointNodes that are reachable from
         *                  the current node
         * @param successor_costs a pointer to a vector of costs corresponding to the above
         *                          nodes
         */
        void getSuccessors(PointNode& node, std::vector<PointNode>* successors,
                            std::vector<cost_type>* successor_costs) override;

        /**
         * Returns true if we are at the destination.
         * @param node the current node
         * @return true if the current node is the destination node
         */
        bool stopSearch(PointNode& node) override;

        const PointNode& destinationNode() {return dest_node;};

        virtual ~ThetaStarSearch() = default;

    private:
        const PointNode start_node, dest_node;
        // we are using a raw pointer to be able to "own" a reference to a
        // vector of Obstacles, and avoid a deep copy
        const std::vector<Obstacle>* const obstacles;
    };

    std::vector<PointNode> ThetaStarSearch::getStartNodes() {
        return {start_node};
    }

    void ThetaStarSearch::getSuccessors(PointNode &node, std::vector<PointNode> *successors,
                                        std::vector<cost_type> *successor_costs) {

        // draw a line segment from the current point to each obstacle vertex,
        // and add it to successors if it doesn't intersect anything else
        for (const Obstacle& ob : *obstacles) {
            for (const Point& point : ob.getBoundaryPolygon().getPoints()) {
                Segment seg = Segment{node.point(), point};

                // O(n^2) w.r.t points unfortunately
                // TODO: potentially create a centre and radius for polygons
                // as a heuristic for whether to check intersection or not

                for (const Obstacle& ob2 : *obstacles) {
                    for (const Segment& seg2 : ob2.getBoundaryPolygon().getSegments()) {
                        if (!intersects(seg, seg2)) {
                            // the segment from the current node to the point "point"
                            // does not intersect line segments in any obstacle,
                            // add it to successors
                            // TODO: add violation cost to the cost
                            successors->emplace_back(PointNode(point));
                            // cost is euclidian distance
                            successor_costs->emplace_back(dist(node.point(), point));
                        }
                    }
                }
            }
        }
    }

    bool ThetaStarSearch::stopSearch(PointNode &node)
    {
        return node == dest_node;
    }
}

std::optional <std::vector<Point>>
ThetaStarPathPlanner::findPath(const Point &start, const Point &dest, const std::vector <Obstacle> &obstacles,
                               const ViolationFunction &violation_function) {
    ThetaStarSearch search(start, dest, obstacles);
    search.search();

    std::vector<PointNode*> node_path = search.reconstructPointerPath(search.destinationNode());

    std::vector<Point> point_path;
    point_path.reserve(node_path.size());

    std::transform(node_path.begin(), node_path.end(), std::back_inserter(point_path),
                    [](const PointNode* const point_node_ptr) {
        return point_node_ptr->point();
    });

    return point_path;
}
