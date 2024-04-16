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
    if (occ_map == nullptr){ // Skip if no occupancy map set
        std::cout << "Occupancy map not set" << std::endl;
        return path;
    }
    if (!goal.init){ // Skip if goal is not initialized
        std::cout << "Goal node not set" << std::endl;
        return path;
    }
    if (opt_cost[start.get_xidx()][start.get_yidx()] != INF){ // If this state was already visited during a previous query, then use previous computation
        path = get_path(start);
        return path;
    }

    dp_iterations(start);
    
    if (opt_cost[start.get_xidx()][start.get_yidx()] == INF){
        std::cout << "DP did not reach start node" << std::endl;
    } else {
        path = get_path(start);
    }
        
    return path;
}

double DynamicProgram2D::find_opt_cost(const Node2D& start){
    double cost = INF;
    if (occ_map == nullptr){ // Skip if no occupancy map set
        std::cout << "Occupancy map not set" << std::endl;
        return cost;
    }
    if (!goal.init){ // Skip if goal is not initialized
        std::cout << "Goal node not set" << std::endl;
        return cost;
    }
    if (opt_cost[start.get_xidx()][start.get_yidx()] != INF){ // If this state was already visited during a previous query, then use previous computation
        return opt_cost[start.get_xidx()][start.get_yidx()];
    }

    dp_iterations(start);

    if (opt_cost[start.get_xidx()][start.get_yidx()] == INF){
        std::cout << "DP did not reach start node" << std::endl;
    } else {
        cost = opt_cost[start.get_xidx()][start.get_yidx()];
    }

    return cost;
    
}

void DynamicProgram2D::dp_iterations(const Node2D& start){
    opt_cost[goal.get_xidx()][goal.get_yidx()] = 0;
    int iter = 0;
    while (iter < iter_max){ // Main logic loop
        ++iter;
        // std::cout << "\tIteration: " << iter << std::endl;
        for (int i = 0; i < width; ++i){
            for (int j = 0; j < height; ++j){
                Node2D u(i,j);
                std::vector<Node2D> vs = get_neighbors(u, width, height);
                for (const Node2D& v : vs){
                    // std::cout << v.to_string() << std::endl;
                    if (opt_cost[v.get_xidx()][v.get_yidx()] == INF) continue; // If next state has INF optimal cost, then skip

                    int occ_val = occ_map->data[v.get_xidx() + v.get_yidx() * width]; // If next state is occupied, then skip
                    if (occ_val==100 || occ_val==-1) continue;
                    
                    double cost = get_dist(u, v); // + ext_map->data[v.get_xidx() + v.get_yidx() * width];

                    opt_cost[u.get_xidx()][u.get_yidx()] = std::min(opt_cost[u.get_xidx()][u.get_yidx()], opt_cost[v.get_xidx()][v.get_yidx()] + cost);
                }
            }
        }
        if (opt_cost[start.get_xidx()][start.get_yidx()] < INF && iter>=iter_min) break;
    }
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