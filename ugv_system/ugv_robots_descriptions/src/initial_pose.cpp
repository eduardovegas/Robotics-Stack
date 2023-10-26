#include <string.h>
#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <stdio.h>
#include <stdlib.h>

class InitialPose
{
    public:
        InitialPose()
        {
            nh.param<double>("/l1br_initial_pose/x", init_x, 4.52);
            nh.param<double>("/l1br_initial_pose/y", init_y, 2.37);
            nh.param<double>("/l1br_initial_pose/z", init_z, 0.03);
            init_pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 10, true);

            publish_initial_pose();
        }
        void publish_initial_pose()
        {
            geometry_msgs::PoseWithCovarianceStamped init_position_msg;

            init_position_msg.pose.pose.position.x = init_x + 16.80;
            init_position_msg.pose.pose.position.y = init_y + 8.38;
            init_position_msg.pose.pose.position.z = init_z;

            init_pose_pub.publish(init_position_msg);
        }
    
    private:
        ros::NodeHandle nh;
        ros::Publisher init_pose_pub;
        double init_x;
        double init_y;
        double init_z;

};
int main(int argc, char** argv)
{
    ros::init(argc, argv, "l1br_initial_pose_node");
    InitialPose position;

    ros::spin();

    return 0;
}
