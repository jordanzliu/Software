#pragma once

// I apologize if your CLion slows to a crawl
#include <boost/graph/astar_search.hpp>
#include <boost/graph/grid_graph.hpp>

#include "ai/world/field.h"
#include "path_planner.h"
namespace AStar
{
    // TODO: potentially move a lot of stuff out of this header

    // number of graph vertices per metre
    // TODO: make this a dynamic parameter or passed in the constructor
    // to AStarGridGraph
    static constexpr unsigned long GRID_VERTEX_DENSITY = 20;
    // edge cost type
    typedef double edge_cost_t;
    // 2D grid graph type
    typedef boost::grid_graph<2> GridGraph2D;
    // vertex type - a 2D point on a grid
    typedef GridGraph2D::vertex_descriptor GridVertex;

    // a hash function for a grid vertex, necessary for all the maps
    // that boost::astar_search uses
    struct grid_vertex_hash : std::unary_function<GridVertex, std::size_t>
    {
        std::size_t operator()(const GridVertex &gp) const;
    };

    class AStarGridGraph
    {
       public:
        AStarGridGraph() = delete;

        /**
         * Constructs a grid graph from a Field object.
         *
         * @param field the field to create a graph for
         */
        explicit AStarGridGraph(const Field &field);

        /**
         * Converts a grid point to a Point on the field.
         *
         * @param grid_v the grid vertex to convert to a Point.
         */
        Point gridPointToPoint(const GridVertex &grid_v);

        /**
         * Finds the nearest vertex on the grid graph to the
         * given point.
         *
         * @param point the point to find a grid vertex close to it
         */
        GridVertex nearestGridVertex(const Point &point);

        /**
         * Returns a const reference to the grid graph itself.
         * @return the underlying grid graph.
         */
        const GridGraph2D &graph();

        /**
         * The distance between vertices on the grid graph.
         * @return the distance between vertices on the grid graph.
         */
        constexpr double gridPointDistance();

       private:
        std::unique_ptr<GridGraph2D> field_graph;

        // this map is used to convert from the grid points that A* will
        // return, into Points that can be used for navigation
        std::vector<std::pair<Point, GridVertex>> grid_points_list;

        const double field_min_x, field_min_y;
    };

    class AStarHeuristic : public boost::astar_heuristic<GridGraph2D, edge_cost_t>
    {
       public:
        AStarHeuristic() = delete;

        /**
         * Constructs an AStarHeuristic for a given graph and destination point.
         * @param _graph the graph
         * @param _dest the destination point
         */
        explicit AStarHeuristic(const std::shared_ptr<AStarGridGraph> &_graph,
                                const Point &_dest);
        edge_cost_t operator()(GridVertex gp);

       private:
        std::shared_ptr<AStarGridGraph> graph;
        const Point dest_point;
    };

    struct FoundGoal
    {
    };

    class AStarVertexVisitor : public boost::default_astar_visitor
    {
       public:
        AStarVertexVisitor() = delete;

        /**
         * Construct a visitor for the given destination graph vertex.
         * @param _dest destinaton graph vertex
         */
        explicit AStarVertexVisitor(GridVertex _dest);

        /**
         * Throws FoundGoal if the destination vertex is reached.
         * @param grid_v the grid vertex that may or may not be the destination
         * @param graph the graph
         */
        void examine_vertex(GridVertex grid_v, const GridGraph2D &graph);

       private:
        GridVertex dest;
    };

    class AStarPathPlanner : public PathPlanner
    {
       public:
        /**
         * Constructs an AStarPathPlanner for a given field.
         * @param field the field
         */
        explicit AStarPathPlanner(const Field &field);
        /**
         * Returns a path from start to dest if it is possible,
         * otherwise return std::nullopt
         *
         * @param world the world
         * @param start the start point
         * @param dest the destination point
         * @return
         */
        std::optional<std::vector<Point>> findPath(const World &world, const Point &start,
                                                   const Point &dest) override;
        ~AStarPathPlanner() override = default;

       private:
        std::shared_ptr<AStarGridGraph> field_graph_ptr;
    };
}  // namespace AStar