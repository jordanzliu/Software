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
         * This function returns the nodes adjacent to given node, as well as
         * the cost associated with each node.
         *
         * @param node The node we are currently at
         * @param successors a pointer to a vector of PointNodes that are adjecent to
         *                  the current node
         * @param successor_costs a pointer to a vector of costs corresponding to the above
         *                          nodes
         */
        void getSuccessors(PointNode& node, std::vector<PointNode>* successors,
                            std::vector<cost_type>* successor_costs) override;
        /**
         * Returns true if the line segment {node1, node2} does not intersect an
         * obstacle.
         * @param node1 first node
         * @param node2 2nd node
         * @param cost (out parameter) distance between the two nodes
         * @return true if {node1, node2} doesn't intersect anything
         */
        bool isSegmentFree(PointNode& node1, PointNode& node2, cost_type* cost) override;

        /**
         * Returns true if we are at the destination.
         * @param node the current node
         * @return true if the current node is the destination node
         */
        bool stopSearch(PointNode& node) override;

        const PointNode& destinationNode() {return dest_node;};

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
        // TODO: add nodes that are on the same parent polygon, and are on the same
        // line segment as `node`
    }


    bool ThetaStarSearch::stopSearch(PointNode &node)
    {
        return node == dest_node;
    }

    bool ThetaStarSearch::isSegmentFree(PointNode &node1, PointNode &node2, cost_type *cost) {
        // TODO: replace this with a function that checks if the line segment
        // between node1 and node2 intersects anything
        return Algorithm::isSegmentFree(node1, node2, cost);
    }
}

std::optional <std::vector<Point>>
ThetaStarPathPlanner::findPath(const Point &start, const Point &dest, const std::vector <Obstacle> &obstacles,
                               const ViolationFunction &violation_function) {
    ThetaStarSearch search(start, dest, obstacles);
    search.search();
    // TODO: figure out if the search failed to find a path, and return nullopt
    std::vector<PointNode*> node_path = search.reconstructPointerPath(search.destinationNode());

    std::vector<Point> point_path;
    point_path.reserve(node_path.size());

    std::transform(node_path.begin(), node_path.end(), std::back_inserter(point_path),
                    [](const PointNode* const point_node_ptr) {
        return point_node_ptr->point();
    });

    return point_path;
}
