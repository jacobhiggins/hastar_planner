#include "dp_2d.hpp"
#include <ros/ros.h>
#include <string>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
class NodeLogic{
    public:
        DynamicProgram2D dp_solver;
        ros::NodeHandle nh;
        nav_msgs::OccupancyGridPtr occ_map;
        geometry_msgs::PoseStampedPtr goal_pose;
        geometry_msgs::PoseWithCovarianceStampedPtr init_pose;
        ros::Subscriber occ_map_sub, ext_map_sub, init_pose_sub, goal_pose_sub;
        ros::Publisher viz_path_pub;
        NodeLogic(ros::NodeHandle nh_, std::string occ_map_topic, const double& min_iters, const double& max_iters) : nh(nh_){
            occ_map_sub = nh.subscribe(occ_map_topic, 1, &NodeLogic::occ_map_cb, this);
            init_pose_sub = nh.subscribe("/initialpose", 1, &NodeLogic::init_pose_cb, this);
            goal_pose_sub = nh.subscribe("/move_base_simple/goal", 1, &NodeLogic::goal_pose_cb, this);
            viz_path_pub = nh.advertise<visualization_msgs::Marker>("/dp_path", 1);
            dp_solver = DynamicProgram2D(min_iters, max_iters);
        }
        void occ_map_cb(const nav_msgs::OccupancyGridPtr& msg){
            occ_map = msg;
            dp_solver.set_occ_map(msg);
            // std::cout << occ_map->info.resolution << std::endl;
        }
        void goal_pose_cb(const geometry_msgs::PoseStampedPtr& msg){
            goal_pose = msg;
            int goal_xidx = int(msg->pose.position.x - occ_map->info.origin.position.x)/ occ_map->info.resolution;
            int goal_yidx = int(msg->pose.position.y - occ_map->info.origin.position.y)/ occ_map->info.resolution;
            Node2D goal(goal_xidx, goal_yidx);
            dp_solver.set_goal(goal);
        }
        void init_pose_cb(const geometry_msgs::PoseWithCovarianceStampedPtr& msg){
            init_pose = msg;
            int init_xidx = int(msg->pose.pose.position.x - occ_map->info.origin.position.x)/ occ_map->info.resolution;
            int init_yidx = int(msg->pose.pose.position.y - occ_map->info.origin.position.y)/ occ_map->info.resolution;
            Node2D start(init_xidx, init_yidx);
            std::vector<Node2D> path = dp_solver.search(start);
            // for (const Node2D& node : path){
            //     std::vector<double> xy = node2xy(node, occ_map->info.resolution, occ_map->info.origin.position.x, occ_map->info.origin.position.y);
            //     std::cout << "x: " << xy[0] << " y: " << xy[1] << std::endl;
            // }
            // Visualize path
            visualization_msgs::Marker path_viz;
            path_viz.header.frame_id = "map";
            path_viz.header.stamp = ros::Time::now();
            path_viz.ns = "dp_path";
            path_viz.action = visualization_msgs::Marker::ADD;
            path_viz.pose.orientation.w = 1.0;
            path_viz.id = 0;
            path_viz.type = visualization_msgs::Marker::LINE_STRIP;
            path_viz.scale.x = 0.1;
            path_viz.color.r = 1.0;
            path_viz.color.a = 1.0;
            for (const Node2D& node : path){
                std::vector<double> xy = node2xy(node, occ_map->info.resolution, occ_map->info.origin.position.x, occ_map->info.origin.position.y);
                geometry_msgs::Point p;
                p.x = xy[0];
                p.y = xy[1];
                path_viz.points.push_back(p);
            }
            viz_path_pub.publish(path_viz);
        }

};

int main(int argc, char **argv){
    ros::init(argc, argv, "dp_2d_node");
    ros::NodeHandle nh;
    int min_iters, max_iters;
    std::string occ_map_topic;
    if(!nh.getParam("dp_2d_node/occ_map_topic", occ_map_topic)){occ_map_topic="/map_downsampled";}
    if(!nh.getParam("dp_2d_node/min_iters", min_iters)){min_iters=100;}
    if(!nh.getParam("dp_2d_node/max_iters", max_iters)){max_iters=1000;}
    NodeLogic node_logic(nh, occ_map_topic, min_iters, max_iters);
    ros::spin();
    return 0;
}