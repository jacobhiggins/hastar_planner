#include "dp_2d.hpp"
#include <queue>

// Constructor
DynamicProgram2D::DynamicProgram2D() : width(0), height(0), iter_min(0), iter_max(0){}

DynamicProgram2D::DynamicProgram2D(const int& iter_min_, const int& iter_max_) : width(0), height(0), iter_min(iter_min_), iter_max(iter_max_){
    goal = Node2D();
    reset_opt_cost();
}

DynamicProgram2D::DynamicProgram2D(const int& width_, const int& height_, const int& iter_min_, const int& iter_max_) : width(width_), height(height_), iter_min(iter_min_), iter_max(iter_max_){
    opt_cost.resize(width);
    for (int i = 0; i < width; i++){
        opt_cost[i].resize(height);
    }
    reset_opt_cost();
}

// Algorithm
std::vector<Node2D> DynamicProgram2D::search(const Node2D& start){
    std::vector<Node2D> path;
    if (occ_map == nullptr){
        std::cout << "Occupancy map not set" << std::endl;
        return path;
    }
    if (!goal.init){
        std::cout << "Goal node not set" << std::endl;
        return path;
    }
    if (opt_cost[start.get_xidx()][start.get_yidx()] != INF){
        path = get_path(start);
        return path;
    }
    opt_cost[goal.get_xidx()][goal.get_yidx()] = 0;
    int iter = 0;
    while (iter < iter_max){
        ++iter;
        // std::cout << "\tIteration: " << iter << std::endl;
        for (int i = 0; i < width; ++i){
            for (int j = 0; j < height; ++j){
                Node2D u(i,j);
                std::vector<Node2D> vs = get_neighbors(u, width, height);
                for (const Node2D& v : vs){
                    // std::cout << v.to_string() << std::endl;
                    int occ_val = occ_map->data[v.get_xidx() + v.get_yidx() * width];
                    if (occ_val==100 || occ_val==-1) continue;
                    
                    double cost = get_dist(u, v); // + ext_map->data[v.get_xidx() + v.get_yidx() * width];

                    opt_cost[u.get_xidx()][u.get_yidx()] = std::min(opt_cost[u.get_xidx()][u.get_yidx()], opt_cost[v.get_xidx()][v.get_yidx()] + cost);
                }
            }
        }
        if (opt_cost[start.get_xidx()][start.get_yidx()] < INF && iter>=iter_min) break;
    }
    
    if (opt_cost[start.get_xidx()][start.get_yidx()] == INF){
        std::cout << "DP did not reach start node" << std::endl;
    } else {
        path = get_path(start);
    }
        
    return path;
}

std::vector<Node2D> DynamicProgram2D::get_path(const Node2D& start){
    std::vector<Node2D> path;
    if (opt_cost[start.get_xidx()][start.get_yidx()] == INF){
        std::cout << "DP did not reach start node" << std::endl;
    } else {
        Node2D u = start;
        path.push_back(u);
        while (opt_cost[u.get_xidx()][u.get_yidx()] > 1){
            std::vector<Node2D> vs = get_neighbors(u, width, height);
            double min_cost = INF;
            Node2D min_node;
            for (const Node2D& v : vs){
                int occ_val = occ_map->data[v.get_xidx() + v.get_yidx() * width];
                if (occ_val==100 || occ_val==-1) continue;
                if (opt_cost[v.get_xidx()][v.get_yidx()] < min_cost){
                    min_cost = opt_cost[v.get_xidx()][v.get_yidx()];
                    min_node = v;
                }
            }
            u = min_node;
            path.push_back(u);
        }
    }
    return path;
}

std::vector<Node2D> DynamicProgram2D::search(const Node2D& start, const Node2D& goal_){
    set_goal(goal_);
    reset_opt_cost();
    return search(start);
}

// Auxillary functions
void DynamicProgram2D::reset_opt_cost(){
    for (int i = 0; i < width; ++i){
        for (int j = 0; j < height; ++j){
            opt_cost[i][j] = INF;
        }
    }
}


// Getters

// Setters
void DynamicProgram2D::set_goal(const Node2D& goal_){
    goal = goal_;
    reset_opt_cost();
}
void DynamicProgram2D::set_occ_map(const nav_msgs::OccupancyGridPtr& occ_map_){
    occ_map = occ_map_;
    set_map_dims(occ_map->info.width, occ_map->info.height);
    opt_cost.resize(width);
    for (int i = 0; i < width; i++){
        opt_cost[i].resize(height);
    }
}
void DynamicProgram2D::set_ext_map(const nav_msgs::OccupancyGridPtr& ext_map_){ext_map = ext_map_;}
void DynamicProgram2D::set_map_dims(const int& width_, const int& height_){width = width_;height = height_;}