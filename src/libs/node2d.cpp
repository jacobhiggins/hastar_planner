#include "node2d.hpp"
#include <algorithm>
#include <cmath>

// Constructor
Node2D::Node2D() : xidx(-1), yidx(-1){init=false;}
Node2D::Node2D(const int& xidx_, const int& yidx_) : xidx(xidx_), yidx(yidx_){init = true;}

// Getters
int Node2D::get_xidx() {return xidx;}
int Node2D::get_yidx() {return yidx;}
int Node2D::get_xidx() const {return xidx;}
int Node2D::get_yidx() const {return yidx;}

// Setters
void Node2D::set_xidx(const int& xidx_, const int& width){xidx = std::max(0, std::min(xidx_, width-1));}
void Node2D::set_yidx(const int& yidx_, const int& height){yidx = std::max(0, std::min(yidx_, height-1));}

// Auxilliary functions
double get_dist(const Node2D& node1, const Node2D& node2){
    return std::sqrt(std::pow(node1.xidx - node2.xidx, 2) + std::pow(node1.yidx - node2.yidx, 2));
}
std::vector<double> node2xy(const Node2D& node, const double& resolution, const double& x0, const double& y0){
    std::vector<double> xy;
    xy.push_back(x0 + node.get_xidx() * resolution);
    xy.push_back(y0 + node.get_yidx() * resolution);
    return xy;
}
std::vector<Node2D> get_neighbors(const Node2D& node, const int& width, const int& height){
    std::vector<Node2D> neighbors;
    for (int i = -1; i <= 1; i++){
        for (int j = -1; j <= 1; j++){
            if (i == 0 && j == 0) continue;

            int n_xidx = node.get_xidx() + i;
            int n_yidx = node.get_yidx() + j;
            if (n_xidx < 0 || n_xidx >= width || n_yidx < 0 || n_yidx >= height) continue;

            Node2D neighbor(n_xidx, n_yidx);
            neighbors.push_back(neighbor);
        }
    }
    return neighbors;
}