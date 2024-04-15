#include "dp_2d.hpp"
#include <ros/ros.h>
#include <string>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
class NodeLogic{
    public:
        DynamicProgram2D dp_solver;
        ros::NodeHandle nh;
        ros::Subscriber occ_map_sub, ext_map_sub, init_pose_sub, goal_pose_sub;
        NodeLogic(ros::NodeHandle nh_, std::string occ_map_topic) : nh(nh_){
            occ_map_sub = nh.subscribe(occ_map_topic, 1, &NodeLogic::occ_map_cb, this);

        }
        void NodeLogic::occ_map_cb(const nav_msgs::OccupancyGridPtr& msg){dp_solver.set_occ_map(msg);}

};

int main(int argc, char **argv){
    ros::init(argc, argv, "dp_2d_node");
    ros::NodeHandle nh;

    ros::spin();
    return 0;
}