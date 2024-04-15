#include "node2d.hpp"
#include <vector>
#include <iostream>
#include <nav_msgs/OccupancyGrid.h>
#include <optional>

#ifndef DP_2D
#define DP_2D

const double INF = 1e6;

class DynamicProgram2D{
    public:
        int width, height;
        int iter_min; // Minimum number of iterations before terminating
        int iter_max; // Maximum number of iterations before terminating
        nav_msgs::OccupancyGridPtr occ_map; // Occupancy map 
        nav_msgs::OccupancyGridPtr ext_map; // External cost map
        std::vector<std::vector<double>> opt_cost; // Optimal cost map
        std::vector<std::vector<bool>> visited_mask; // Visited mask
        Node2D goal; // Goal node
        DynamicProgram2D (const int& width_, const int& height_, const int& iter_min_, const int& iter_max_);
        // Algorithm
        std::vector<Node2D> search(const Node2D& start);
        std::vector<Node2D> search(const Node2D& start, const Node2D& goal_);
        // Auxilliary functions
        void reset_opt_cost();
        // Getters
        void get_opt_cost(const double& x, const double& y);
        // Setters
        void set_goal(const Node2D& goal_);
        void set_occ_map(const nav_msgs::OccupancyGridPtr& occ_map_);
        void set_ext_map(const nav_msgs::OccupancyGridPtr& ext_map_);
};

#endif