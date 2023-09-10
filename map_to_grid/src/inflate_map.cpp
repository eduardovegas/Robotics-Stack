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

class InflationNode
{
    public:
        InflationNode()
        {
            nh.param<std::string>("/inflate_map/map_name", map_name, "mapa_laser");
            nh.param<double>("/inflate_map/inflation_radius", radius_m, 0.385);
            nh.param<int>("/inflate_map/cell_size", cell_size_px, 5);
            map_sub = nh.subscribe("/map", 10, &InflationNode::subMapCallback, this);
            inflated_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/inflated_map", 10, true);
        }

        void subMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg)
        {
            calculate_radius_px(map_msg);
            cell_size_m = cell_size_px*map_msg->info.resolution;
            ROS_INFO("Read '%s' from /map topic successfully.", map_name.c_str());
            ROS_INFO("Dimensions (W x H): %d x %d.", map_msg->info.width, map_msg->info.height);
            ROS_INFO("Resolution (m/px): %.2f.", map_msg->info.resolution);
            ROS_INFO("Inflation radius (m): %.2lf.", radius_m);
            ROS_INFO("Inflation radius (px): %d.", radius_px);
            ROS_INFO("Cell size (m): %.2f.", cell_size_m);
            ROS_INFO("Cell size (px): %d.", cell_size_px);

            inflated_map = *map_msg;

            for (int y = 0; y < inflated_map.info.height; y++)
            {
                for (int x = 0; x < inflated_map.info.width; x++)
                {
                    int index = x + y * inflated_map.info.width;
                    if (map_msg->data[index] == 100 && is_an_edge(x, y, map_msg) && can_inflate(x, y))
                    {
                        inflate_circle(x, y);
                    }
                }
            }

            inflated_map_pub.publish(inflated_map);

            update_rviz_config(map_msg);
        }

        void calculate_radius_px(const nav_msgs::OccupancyGrid::ConstPtr& map_msg)
        {
            radius_px = (int) ceil(radius_m/map_msg->info.resolution);
        }

        bool is_an_edge(int x, int y, const nav_msgs::OccupancyGrid::ConstPtr& map_msg)
        {
            for (int y_aux = y - 1; y_aux <= y + 1; y_aux++)
            {
                for (int x_aux = x - 1; x_aux <= x + 1; x_aux++)
                {
                    int index = x_aux + y_aux * inflated_map.info.width;
                    if (map_msg->data[index] == 0)
                    {
                        return true;
                    }
                }
            }
            return false;
        }

        bool can_inflate(int center_x, int center_y)
        {
            return (center_x-radius_px >= 0 && center_y-radius_px >= 0 && center_x+radius_px <= inflated_map.info.width - 1 && center_y+radius_px <= inflated_map.info.height - 1);
        }

        void inflate_circle(int center_x, int center_y)
        {
            for (int y = center_y - radius_px; y <= center_y + radius_px; y++)
            {
                for (int x = center_x - radius_px; x <= center_x + radius_px; x++)
                {
                    if (is_inside_circle(x, y, center_x, center_y))
                    {
                        int index = x + y * inflated_map.info.width;
                        inflated_map.data[index] = 100;
                    }
                }
            }
        }

        bool is_inside_circle(int x, int y, int center_x, int center_y)
        {
            double distance_squared = (x - center_x)*(x - center_x) + (y - center_y)*(y - center_y);
            return distance_squared <= (double) radius_px*radius_px;
        }

        void update_rviz_config(const nav_msgs::OccupancyGrid::ConstPtr& map_msg)
        {
            int greater_dim = map_msg->info.height > map_msg->info.width ? map_msg->info.height : map_msg->info.width;
            float meters = greater_dim*map_msg->info.resolution;

            FILE* stream = popen("rospack find map_to_grid", "r");
            char c_buffer[1024];
            std::string package_path;
            while (fgets(c_buffer, sizeof(c_buffer), stream) != nullptr)
            {
                package_path += c_buffer;
            }
            pclose(stream);

            package_path = package_path.substr(0, package_path.length() - 1);
            std::string format_file_path = package_path + "/rviz/template_config.rviz";
            std::string config_file_path = package_path + "/rviz/rviz_config.rviz";

            std::ifstream format_file(format_file_path);
            std::ofstream config_file(config_file_path);

            std::stringstream buffer;
            buffer << format_file.rdbuf();
            format_file.close();

            std::string format_config = buffer.str();

            // std::string format_config = "Panels:\n  - Class: rviz/Displays\n    Help Height: 78\n    Name: Displays\n    Property Tree Widget:\n      Expanded: ~\n      Splitter Ratio: 0.5\n    Tree Height: 549\n  - Class: rviz/Selection\n    Name: Selection\n  - Class: rviz/Tool Properties\n    Expanded:\n      - /2D Pose Estimate1\n      - /2D Nav Goal1\n      - /Publish Point1\n    Name: Tool Properties\n    Splitter Ratio: 0.5886790156364441\n  - Class: rviz/Views\n    Expanded:\n      - /Current View1\n    Name: Views\n    Splitter Ratio: 0.5\n  - Class: rviz/Time\n    Experimental: false\n    Name: Time\n    SyncMode: 0\n    SyncSource: \"\"\nPreferences:\n  PromptSaveOnExit: true\nToolbars:\n  toolButtonStyle: 2\nVisualization Manager:\n  Class: \"\"\n  Displays:\n    - Alpha: 0.5\n      Cell Size: 70000\n      Class: rviz/Grid\n      Color: 160; 160; 164\n      Enabled: true\n      Line Style:\n        Line Width: 0.029999999329447746\n        Value: Lines\n      Name: Grid\n      Normal Cell Count: 0\n      Offset:\n        X: 21.25\n        Y: 21.25\n        Z: 0\n      Plane: XY\n      Plane Cell Count: 85\n      Reference Frame: <Fixed Frame>\n      Value: true\n    - Alpha: 0.699999988079071\n      Class: rviz/Map\n      Color Scheme: map\n      Draw Behind: false\n      Enabled: true\n      Name: Map\n      Topic: /inflated_map\n      Unreliable: false\n      Use Timestamp: false\n      Value: true\n  Enabled: true\n  Global Options:\n    Background Color: 48; 48; 48\n    Default Light: true\n    Fixed Frame: map\n    Frame Rate: 30\n  Name: root\n  Tools:\n    - Class: rviz/Interact\n      Hide Inactive Objects: true\n    - Class: rviz/MoveCamera\n    - Class: rviz/Select\n    - Class: rviz/FocusCamera\n    - Class: rviz/Measure\n    - Class: rviz/SetInitialPose\n      Theta std deviation: 0.2617993950843811\n      Topic: /initialpose\n      X std deviation: 0.5\n      Y std deviation: 0.5\n    - Class: rviz/SetGoal\n      Topic: /move_base_simple/goal\n    - Class: rviz/PublishPoint\n      Single click: true\n      Topic: /clicked_point\n  Value: true\n  Views:\n    Current:\n      Class: rviz/Orbit\n      Distance: 10\n      Enable Stereo Rendering:\n        Stereo Eye Separation: 0.05999999865889549\n        Stereo Focal Distance: 1\n        Swap Stereo Eyes: false\n        Value: false\n      Field of View: 0.7853981852531433\n      Focal Point:\n        X: 0\n        Y: 0\n        Z: 0\n      Focal Shape Fixed Size: true\n      Focal Shape Size: 0.05000000074505806\n      Invert Z Axis: false\n      Name: Current View\n      Near Clip Distance: 0.009999999776482582\n      Pitch: 0.3853984475135803\n      Target Frame: <Fixed Frame>\n      Yaw: 4.035403251647949\n    Saved: ~\nWindow Geometry:\n  Displays:\n    collapsed: false\n  Height: 846\n  Hide Left Dock: false\n  Hide Right Dock: false\n  QMainWindow State: 000000ff00000000fd000000040000000000000156000002b0fc0200000008fb0000001200530065006c0065006300740069006f006e00000001e10000009b0000005c00fffffffb0000001e0054006f006f006c002000500072006f007000650072007400690065007302000001ed000001df00000185000000a3fb000000120056006900650077007300200054006f006f02000001df000002110000018500000122fb000000200054006f006f006c002000500072006f0070006500720074006900650073003203000002880000011d000002210000017afb000000100044006900730070006c006100790073010000003d000002b0000000c900fffffffb0000002000730065006c0065006300740069006f006e00200062007500660066006500720200000138000000aa0000023a00000294fb00000014005700690064006500530074006500720065006f02000000e6000000d2000003ee0000030bfb0000000c004b0069006e0065006300740200000186000001060000030c00000261000000010000010f000002b0fc0200000003fb0000001e0054006f006f006c002000500072006f00700065007200740069006500730100000041000000780000000000000000fb0000000a00560069006500770073010000003d000002b0000000a400fffffffb0000001200530065006c0065006300740069006f006e010000025a000000b200000000000000000000000200000490000000a9fc0100000001fb0000000a00560069006500770073030000004e00000080000002e10000019700000003000004f30000003efc0100000002fb0000000800540069006d00650100000000000004f3000002eb00fffffffb0000000800540069006d0065010000000000000450000000000000000000000282000002b000000004000000040000000800000008fc0000000100000002000000010000000a0054006f006f006c00730100000000ffffffff0000000000000000\n  Selection:\n    collapsed: false\n  Time:\n    collapsed: false\n  Tool Properties:\n    collapsed: false\n  Views:\n    collapsed: false\n  Width: 1267\n  X: 72\n  Y: 27\n";
            char c_config_content[20000];
            sprintf(c_config_content, format_config.c_str(), cell_size_m, meters/2.0, meters/2.0, (int) std::ceil(meters/cell_size_m));

            std::string config_content(c_config_content);

            config_file << config_content;
            config_file.close();

            ROS_INFO("Updated %s successfully.", config_file_path.c_str());
        }

    private:
        int radius_px;
        int cell_size_px;
        float cell_size_m;
        double radius_m;
        std::string map_name;
        ros::NodeHandle nh;
        ros::Subscriber map_sub;
        ros::Publisher inflated_map_pub;
        nav_msgs::OccupancyGrid inflated_map;

        // tf2_ros::StaticTransformBroadcaster tf_broadcaster;
        // geometry_msgs::TransformStamped transformStamped;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "inflate_map_node");
    InflationNode node;

    // // Create a TF2 static transform broadcaster
    // tf_broadcaster = tf2_ros::StaticTransformBroadcaster();

    // // Create a TransformStamped message
    // transformStamped = geometry_msgs::TransformStamped();

    // // Set the frame IDs
    // transformStamped.header.frame_id = "map";      // Parent frame (source)
    // transformStamped.child_frame_id = "grid"; // Child frame (target)

    // // Set the transformation values (assuming no translation, only rotation)
    // transformStamped.transform.translation.x = 0.0;
    // transformStamped.transform.translation.y = 0.0;
    // transformStamped.transform.translation.z = 0.0;
    // transformStamped.transform.rotation.x = 0.0;
    // transformStamped.transform.rotation.y = 0.0;
    // transformStamped.transform.rotation.z = 0.0;
    // transformStamped.transform.rotation.w = 1.0;

    // // Set the timestamp
    // transformStamped.header.stamp = ros::Time::now();

    // // Publish the TF transform
    // tf_broadcaster.sendTransform(transformStamped);

    ros::spin();
    return 0;
}

