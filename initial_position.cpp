#include <string.h>
#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <nav_msgs/OccupancyGrid.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d_publisher.h>
#include <costmap_2d/cost_values.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <stdio.h>
#include <stdlib.h>

class InicialPosition
{
    public:
        InicialPosition()
        {
            nh.param<double>("/initial_position/x", init_x, 4.52);
            nh.param<double>("/initial_position/y", init_y, 2.37);
            nh.param<double>("/initial_position/z", init_z, 0.03);
            init_pose_map_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 10, true);

            publishInitialPosition();
        }
        void publishInitialPosition()
        {
            geometry_msgs::PoseWithCovarianceStamped init_position_msg;

            init_position_msg.pose.pose.position.x = init_x + 16.80;
            init_position_msg.pose.pose.position.y = init_y + 8.38;
            init_position_msg.pose.pose.position.z = init_z;

            init_pose_map_pub.publish(init_position_msg);
        }
    
    private:
        double init_x;
        double init_y;
        double init_z;

};
int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_initial_position_node");
    InicialPosition position;

    ros::spin();

    return 0;
}
