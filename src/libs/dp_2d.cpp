#include "dp_2d.hpp"

// Constructor

DynamicProgram2D::DynamicProgram2D(const int& width_, const int& height_) : width(width_), height(height_){
    opt_cost.resize(width);
    for (int i = 0; i < width; i++){
        opt_cost[i].resize(height);
    }
    reset_opt_cost();
}

// Algorithm
std::vector<Node2D> DynamicProgram2D::search(const Node2D& start){
    
    std::vector<Node2D> path;
    

    return path;
}

std::vector<Node2D> DynamicProgram2D::search(const Node2D& start, const Node2D& goal_){
    set_goal(goal_);
    return search(start);
}

// Auxillary functions
void DynamicProgram2D::reset_opt_cost(){
    for (int i = 0; i < width; ++i){
        for (int j = 0; j < height; ++j){
            opt_cost[i][j] = 0;
        }
    }
}


// Getters

// Setters
void DynamicProgram2D::set_goal(const Node2D& goal_){goal = goal_;}
void DynamicProgram2D::set_occ_map(const nav_msgs::OccupancyGridPtr& occ_map_){occ_map = occ_map_;}
void DynamicProgram2D::set_ext_map(const nav_msgs::OccupancyGridPtr& ext_map_){ext_map = ext_map_;}