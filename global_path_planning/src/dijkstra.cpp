#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <cmath>
#include <queue>
#include <string>
#include <vector>

// Define a structure to represent a graph node
struct Node
{
    int x, y;
    int index;
    int parent_index;
    double accumulated_cost; // Best cost to reach this node (needed for Dijkstra's algorithm)

    Node(int x, int y, int index, int parent_index, double accumulated_cost) : x(x), y(y), index(index), parent_index(parent_index), accumulated_cost(accumulated_cost) {}
};

// Define the comparison operator for the priority queue
bool is_greater(const Node* a, const Node* b)
{
    return a->accumulated_cost > b->accumulated_cost;
}

class DijkstraAlgo
{
    public:
        DijkstraAlgo()
        {
            grid_subscriber_ = nh_.subscribe("/inflated_map", 1, &DijkstraAlgo::occupancyGridCallback, this);
            initial_pose_subscriber_ = nh_.subscribe("/initialpose", 1, &DijkstraAlgo::initialPoseCallback, this);
            goal_pose_subscriber_ = nh_.subscribe("/move_base_simple/goal", 1, &DijkstraAlgo::goalPoseCallback, this);
            path_publisher_ = nh_.advertise<nav_msgs::Path>("/dijkstra_path", 1, true);
            marker_publisher_ = nh_.advertise<visualization_msgs::Marker>("/dijkstra_path_marker", 1, true);
        }

        void clear_priority_queue()
        {
            for (int i = 0; i < pq.size(); i++)
            {
                delete pq[i]; // Delete the Node* objects
                // pq[i].reset(); // Delete the Node* objects
            }
            pq.clear(); // Clear the outer vector
        }

        void clear_adjacency_list()
        {
            for (int i = 0; i < adjacency_list.size(); i++)
            {
                // for (int j = 0; j < adjacency_list[i].size(); j++)
                // {
                    // adjacency_list[i][j].first.reset();
                // }
                adjacency_list[i].clear(); // Clear the inner vector
            }
            adjacency_list.clear(); // Clear the outer vector
        }

        void occupancyGridCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
        {
            // const std::vector<int8_t>& grid_data = msg->data;
            grid_data = msg->data;
            map_width = msg->info.width;
            map_height = msg->info.height;
            map_resolution = msg->info.resolution;
            nh_.param<int>("/inflate_map/cell_size", cell_size_px, 5);
            ROS_INFO("Cell size (px): %d", cell_size_px);

            has_initial_pose = false;
            has_goal_pose = false;

            process_grid();
        }

        void process_grid()
        {
            // int coarse_width = map_width / cell_size_px;
            // int coarse_height = map_height / cell_size_px;

            double std_edge_cost = (double) map_resolution * cell_size_px;
            double diagonal_edge_cost = std::sqrt(std_edge_cost*std_edge_cost + std_edge_cost*std_edge_cost);

            // Define directions for neighboring cells (up, down, left, right, diagonals)
            const int dx[] = {-cell_size_px, 0, cell_size_px, -cell_size_px, cell_size_px, -cell_size_px, 0, cell_size_px};
            const int dy[] = {-cell_size_px, -cell_size_px, -cell_size_px, 0, 0, cell_size_px, cell_size_px, cell_size_px};

            clear_priority_queue();
            int pq_size = 0;

            clear_adjacency_list();
            adjacency_list.resize(map_width*map_height);

            node_is_in_pq = std::vector<bool>(map_width*map_height, false);
            node_idx_to_pq_idx = std::vector<int>(map_width*map_height, -1);

            for (int y = 0; y < map_height; y+=cell_size_px)
            {
                for (int x = 0; x < map_width; x+=cell_size_px)
                {
                    int idx = y * map_width + x;
                    int cell_value = grid_data[idx];

                    // Only free cells added
                    if (cell_value == 0)
                    {
                        // Add adjacent free cells to the neighbors list
                        for (int dir = 0; dir < 8; dir++)
                        {
                            int nx = x + dx[dir];
                            int ny = y + dy[dir];

                            // Check if the neighboring cell is within bounds
                            if (nx >= 0 && nx < map_width && ny >= 0 && ny < map_height)
                            {
                                int neigh_idx = ny * map_width + nx;
                                int neigh_value = grid_data[neigh_idx];

                                // Add neighbor if it's a free cell
                                if (neigh_value == 0)
                                {
                                    double edge_cost = std_edge_cost;
                                    if (dx[dir] == dy[dir] || dx[dir] == -dy[dir])
                                    {
                                        edge_cost = diagonal_edge_cost; // Diagonally positioned
                                    }
                                    if (node_is_in_pq[neigh_idx])
                                    {
                                        Node* neigh_node = pq[node_idx_to_pq_idx[neigh_idx]];
                                        adjacency_list[idx].emplace_back(neigh_node, edge_cost);
                                        // std::shared_ptr<Node> neigh_node = pq[node_idx_to_pq_idx[neigh_idx]];
                                        // adjacency_list[idx].emplace_back(neigh_node, edge_cost);
                                    }
                                    else
                                    {
                                        Node* neigh_node = new Node(nx, ny, neigh_idx, -1, std::numeric_limits<double>::max());
                                        adjacency_list[idx].emplace_back(neigh_node, edge_cost);
                                        // std::shared_ptr<Node> neigh_node = std::make_shared<Node>(nx, ny, neigh_idx, -1, std::numeric_limits<double>::max());
                                        // adjacency_list[idx].emplace_back(neigh_node, edge_cost);

                                        pq.push_back(neigh_node);
                                        node_is_in_pq[neigh_idx] = true;
                                        node_idx_to_pq_idx[neigh_idx] = pq_size;
                                        pq_size += 1;
                                    }
                                }
                            }
                        }
                    }
                }
            }

            // int c = 0;
            // int s = 0;
            // for(int i = 0; i < map_width*map_height; i++)
            // {
            //     if(!adjacency_list[i].empty())
            //     {
            //         c++;
            //         s+=adjacency_list[i].size();
            //     }
            // }
            // ROS_INFO("Adj_list size:%d", (int) adjacency_list.size());
            // ROS_INFO("Ha %d nos que tem vizinhos", c);
            // ROS_INFO("Ha %d vizinhos na lista de adj", s);
            // ROS_INFO("PQ size: %d", (int) pq.size());
        }

        void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
        {
            start_pose_.header = msg->header;
            start_pose_.pose = msg->pose.pose;

            double x_m = start_pose_.pose.position.x;
            double y_m = start_pose_.pose.position.y;
            int x_px = (int) std::round(x_m/map_resolution);
            int y_px = (int) std::round(y_m/map_resolution);

            start_nearest_x_px = ((int) round((float)x_px/cell_size_px))*cell_size_px;
            start_nearest_y_px = ((int) round((float)y_px/cell_size_px))*cell_size_px;

            if(grid_data[start_nearest_y_px*map_width + start_nearest_x_px] != 0)
            {
                ROS_WARN("Initial pose not defined. It is either on an obstacle or out of bounds.");
                has_initial_pose = false;
            }
            else
            {
                ROS_INFO(
                    "Received initial pose:\n\tm: (%.2f, %.2f)\n\tpx: (%d, %d)",
                    x_m,
                    y_m,
                    start_nearest_x_px,
                    start_nearest_y_px
                );
                has_initial_pose = true;
            }
        }

        void goalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
        {
            goal_pose_ = *msg;

            double x_m = goal_pose_.pose.position.x;
            double y_m = goal_pose_.pose.position.y;
            int x_px = (int) std::round(x_m/map_resolution);
            int y_px = (int) std::round(y_m/map_resolution);

            goal_nearest_x_px = ((int) round((float)x_px/cell_size_px))*cell_size_px;
            goal_nearest_y_px = ((int) round((float)y_px/cell_size_px))*cell_size_px;

            if(grid_data[goal_nearest_y_px*map_width + goal_nearest_x_px] != 0)
            {
                ROS_WARN("Goal pose not defined. It is either on an obstacle or out of bounds.");
                has_goal_pose = false;
            }
            else
            {
                ROS_INFO(
                    "Received goal pose:\n\tm: (%.2f, %.2f)\n\tpx: (%d, %d)",
                    x_m,
                    y_m,
                    goal_nearest_x_px,
                    goal_nearest_y_px
                );
                has_goal_pose = true;
            }

            if(has_initial_pose && has_goal_pose)
            {
                run_dijkstra_algorithm();
            }
        }

        void run_dijkstra_algorithm()
        {
            int start_idx = start_nearest_y_px*map_width + start_nearest_x_px;
            int goal_idx = goal_nearest_y_px*map_width + goal_nearest_x_px;

            bool found_start;
            for(int i = 0; i < pq.size(); i++)
            {
                if(pq[i]->index == start_idx)
                {
                    pq[i]->accumulated_cost = 0.0;
                    pq[i]->parent_index = pq[i]->index;
                    found_start = true;
                    break;
                }
            }
            if(found_start)
            {
                ROS_INFO("Set start pose cost to 0.");
            }
            else
            {
                ROS_ERROR("Start pose not found in priority queue.");
                return;
            }

            // Perform Dijkstra's algorithm
            int pq_size = (int) pq.size();
            int nb_it = 0;
            Node* curr_node;
            ROS_INFO("Dijkstra algorithm started.");
            ros::Time start_time = ros::Time::now();
            std::make_heap(pq.begin(), pq.end(), is_greater);
            while (nb_it < pq_size)
            {
                std::pop_heap(pq.begin(), pq.end()-nb_it, is_greater);
                curr_node = pq[pq_size-1-nb_it];
                node_is_in_pq[curr_node->index] = false;
                if (curr_node->index == goal_idx)
                    break;

                nb_it++;

                // Visit neighbors of the current node
                int nb_neigh = adjacency_list[curr_node->index].size();
                for (int i = 0; i < nb_neigh; i++)
                {
                    Node* neigh = adjacency_list[curr_node->index][i].first;
                    double edge_cost = adjacency_list[curr_node->index][i].second;

                    // If a shorter path to the neighbor is found, update its distance
                    if (node_is_in_pq[neigh->index] &&
                        edge_cost + curr_node->accumulated_cost < neigh->accumulated_cost)
                    {
                        neigh->parent_index = curr_node->index;
                        neigh->accumulated_cost = edge_cost + curr_node->accumulated_cost;
                        std::make_heap(pq.begin(), pq.end()-nb_it, is_greater);
                    }
                }
            }
            nb_it++;
            ros::Time end_time = ros::Time::now();
            ros::Duration duration = end_time - start_time;
            ROS_INFO("Dijkstra algorithm finished.");
            ROS_INFO("Nb iterations: %d.", nb_it);
            ROS_INFO("Time: %lf ms.", duration.toSec()*1000.0);
            ros::Duration(3.0).sleep();

            std::vector<Node*> path;
            while(true)
            {
                path.push_back(curr_node);
                if (curr_node->parent_index == curr_node->index)
                    break;

                for(int i = 0; i < pq_size; i++)
                {
                    if (curr_node->parent_index == pq[i]->index)
                    {
                        curr_node = pq[i];
                        break;
                    }
                }
            }
            std::reverse(path.begin(), path.end());

            char node_info[255];
            std::string path_str = "\nPATH: [";
            for(int i = 0; i < path.size(); i++)
            {
                Node* n = path[i];
                sprintf(
                    node_info,
                    "\n    idx: %d, parent idx: %d, coords (px): [%d, %d], coords (m): [%.2lf, %.2lf], acc cost (m): %.2lf,",
                    n->index, n->parent_index, n->x, n->y, (n->x)*map_resolution, (n->y)*map_resolution, n->accumulated_cost
                );
                path_str += node_info;

                geometry_msgs::PoseStamped pose_msg;
                pose_msg.pose.position.x = n->x*map_resolution; // Set x coordinate in meters
                pose_msg.pose.position.y = n->y*map_resolution; // Set y coordinate in meters
                pose_msg.pose.position.z = 0.0; // Z coordinate is 0 (assuming 2D path)
                path_msg.poses.push_back(pose_msg);
            }
            path_str += "\n]";
            ROS_INFO("%s", path_str.c_str());

            publish_path(path);

            process_grid();
        }

        void publish_path(std::vector<Node*>& path)
        {
            path_msg = nav_msgs::Path();
            path_msg.header = goal_pose_.header;
            for(int i = 0; i < path.size(); i++)
            {
                geometry_msgs::PoseStamped pose_msg;
                pose_msg.pose.position.x = path[i]->x*map_resolution; // Set x coordinate in meters
                pose_msg.pose.position.y = path[i]->y*map_resolution; // Set y coordinate in meters
                pose_msg.pose.position.z = 0.0; // Z coordinate is 0 (assuming 2D path)
                path_msg.poses.push_back(pose_msg);
            }

            std::vector<geometry_msgs::Point> path_points;
            for (const auto& pose_stamped : path_msg.poses)
            {
                geometry_msgs::Point point;
                point.x = pose_stamped.pose.position.x;
                point.y = pose_stamped.pose.position.y;
                point.z = pose_stamped.pose.position.z;
                path_points.push_back(point);
            }

            // // Publish the path
            path_publisher_.publish(path_msg);

            // Publish markers for visualization in RViz
            marker = visualization_msgs::Marker();
            marker.header = goal_pose_.header;
            marker.ns = "path";
            marker.id = 0;
            marker.type = visualization_msgs::Marker::LINE_STRIP;
            marker.action = visualization_msgs::Marker::ADD;
            marker.scale.x = 0.08; // Line width
            marker.color.a = 1.0; // Alpha (transparency)
            marker.color.r = 1.0; // Red
            marker.points = path_points; // Use the path points for visualization
            marker_publisher_.publish(marker);
        }

    private:
        ros::NodeHandle nh_;

        // map
        int map_width;
        int map_height;
        float map_resolution;
        int cell_size_px;
        double cell_size_m;
        std::vector<int8_t> grid_data;
        std::vector<std::vector<std::pair<Node*, double>>>adjacency_list; // adjacency list of nodes
        std::vector<Node*> pq; // priority queue for dijkstra
        std::vector<bool> node_is_in_pq;
        std::vector<int> node_idx_to_pq_idx;

        // poses
        bool has_initial_pose;
        bool has_goal_pose;
        int start_nearest_x_px;
        int start_nearest_y_px;
        int goal_nearest_x_px;
        int goal_nearest_y_px;

        ros::Timer timer;
        ros::Subscriber grid_subscriber_;
        ros::Subscriber initial_pose_subscriber_;
        ros::Subscriber goal_pose_subscriber_;
        ros::Publisher path_publisher_;
        ros::Publisher marker_publisher_;
        geometry_msgs::PoseStamped start_pose_;
        geometry_msgs::PoseStamped goal_pose_;
        visualization_msgs::Marker marker;
        nav_msgs::Path path_msg;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dijkstra_node");
    DijkstraAlgo dijkstra;

    ros::spin();

    return 0;
}
