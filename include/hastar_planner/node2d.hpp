#include <vector>
#ifndef NODE2D
#define NODE2D
class Node2D {
    public:
        bool init;
        int xidx, yidx;
        Node2D ();
        Node2D (const int& xidx_, const int& yidx_);
        void set_xidx(const int& xidx_, const int& width);
        void set_yidx(const int& yidx_, const int& height);
        int get_xidx();
        int get_yidx();
        int get_xidx() const;
        int get_yidx() const;
};
double get_dist(const Node2D& node1, const Node2D& node2);
std::vector<Node2D> get_neighbors(Node2D& node, const int& width, const int& height);
std::vector<double> node2xy(const Node2D& node, const double& resolution, const double& x0, const double& y0);
#endif