
/*
Used for teaching controller design for Turtlebot3
Lantao Liu
ISE, Indiana University
*/

#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <std_srvs/Empty.h>
#include <sstream>
#include "geometry.h"
#include "pid.h"

// global vars
tf::Point Odom_pos;    //odometry position (x, y, z)
double Odom_yaw;    //odometry orientation (yaw)
double Odom_v, Odom_w;    //odometry linear and angular speeds

// ROS Topic Publishers
ros::Publisher cmd_vel_pub;
// ros::Publisher marker_pub;

// ROS Topic Subscribers
ros::Subscriber odom_sub;
ros::Subscriber marker_sub;

Geometry geometry;

//Global variables for PID

int num_slice2 = 50;               // divide a circle into segments

double maxSpeed = 0.3;
double distanceConst = 0.5;
double dt = 0.1, maxT = M_PI, minT = -M_PI, Kp = 0.3, Ki = 0.05, Kd = 0.01;
double dtS = 0.1, maxS = maxSpeed, minS = 0.0, KpS = 0.08, KiS = 0.01, KdS = 0.005;


double getDistance(Point &p1, Point &p2);

/*
 * Callback function for odometry msg, used for odom subscriber
 */
void odomCallback(const nav_msgs::Odometry odom_msg) {
    /* upon "hearing" odom msg, retrieve its position and orientation (yaw) information.
     * tf utility functions can be found here: http://docs.ros.org/diamondback/api/tf/html/c++/namespacetf.html
     * odom data structure definition is here: https://mirror.umd.edu/roswiki/doc/diamondback/api/nav_msgs/html/msg/Odometry.html
     */
    tf::pointMsgToTF(odom_msg.pose.pose.position, Odom_pos);
    Odom_yaw = tf::getYaw(odom_msg.pose.pose.orientation);

    //update observed linear and angular speeds (real speeds published from simulation)
    Odom_v = odom_msg.twist.twist.linear.x;
    Odom_w = odom_msg.twist.twist.angular.z;

    //display on terminal screen
    //ROS_INFO("Position: (%f, %f); Yaw: %f", Odom_pos.x(), Odom_pos.y(), Odom_yaw);
}

void displayLane(bool isTrajectoryPushed, Geometry &geometry) {
    static visualization_msgs::Marker path;
    path.type = visualization_msgs::Marker::LINE_STRIP;

    path.header.frame_id = "base_link";  //NOTE: this should be "paired" to the fixed frame id entry in Rviz, the default setting in Rviz for tb3-fake is "odom". Keep this line as is if you don't have an issue.
    path.header.stamp = ros::Time::now();
    path.ns = "odom";
    path.id = 0;
    path.action = visualization_msgs::Marker::ADD; // use line marker
    path.lifetime = ros::Duration();

    // path line strip is blue
    path.color.b = 1.0;
    path.color.a = 1.0;

    path.scale.x = 0.02;
    path.pose.orientation.w = 1.0;


    static int slice_index2 = 0;


    VECTOR2D *prev = NULL, *current = NULL;

    while (path.points.size() <= num_slice2) {
        geometry_msgs::Point p;

        float angle = slice_index2 * 2 * M_PI / num_slice2;
        slice_index2++;
        p.x = 4 * cos(angle) - 0.5;       //some random circular trajectory, with radius 4, and offset (-0.5, 1, 0)
        p.y = 4 * sin(angle) + 1.0;
        p.z = 0;

        path.points.push_back(p);         //for drawing path, which is line strip type


        //Add points for PID use only in 1st execution
        if (!isTrajectoryPushed) {

            VECTOR2D *temp = new VECTOR2D(p.x, p.y);

            geometry.trajectory.push_back(*temp);

            current = temp;

            if (prev != NULL) {

                geometry.path.push_back(geometry.getLineSegment(*prev, *current));

            }
            prev = current;

        }

    }

    //If you want to connect start and END points
    if (prev != NULL && current != NULL && current != prev)
        geometry.path.push_back(geometry.getLineSegment(*prev, *current));


    // marker_pub.publish(path);
}

void dijkstraMarkerCallback(const visualization_msgs::Marker& msg)
{
    VECTOR2D *prev = NULL, *current = NULL;

    std::vector<geometry_msgs::Point> path_points = msg.points;

    for (const auto& p : path_points)
    {
        VECTOR2D *temp = new VECTOR2D(p.x - 16.80, p.y - 8.38);
        
        ROS_INFO("p: (%.2f, %.2f)", temp->x, temp->y);

        geometry.trajectory.push_back(*temp);

        current = temp;

        if (prev != NULL) {
            geometry.path.push_back(geometry.getLineSegment(*prev, *current));
        }
        prev = current;
    }
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "control");
    ros::NodeHandle n("~");
    tf::TransformListener m_listener;
    tf::StampedTransform transform;

    int frame_count= 0;

    cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    odom_sub = n.subscribe("odom", 10, odomCallback);
    marker_sub = n.subscribe("/dijkstra_path_marker", 10, dijkstraMarkerCallback);

    ros::Rate loop_rate(10); // ros spins 10 frames per second
    geometry_msgs::Twist tw_msg;

    double angleError = 0.0;
    double speedError = 0.0;

    PID pidTheta = PID(dt, maxT, minT, Kp, Kd, Ki);
    PID pidVelocity = PID(dtS, maxS, minS, KpS, KdS, KiS);
        
    while(ros::ok())
    {
        if (frame_count == 0)
            displayLane(false, geometry);
        else
            displayLane(true, geometry);
        //ROS_INFO("frame %d", frame_count);

        double omega = 0.0;
        double speed = 0.0;
        double prevDistError = 1000.0;

        /*Error calculation*/
        VECTOR2D current_pos, pos_error;
        current_pos.x = Odom_pos.x();
        current_pos.y = Odom_pos.y();
        //ROS_INFO("Nearest %f,%f ", current_pos.x, current_pos.y);

        Geometry::LineSegment *linesegment = geometry.getNearestLine(current_pos);

        Geometry::LineSegment linesegmentPerpen = geometry.getMinimumDistanceLine(*linesegment, current_pos);

        //Get Next LineSegment to do velocity PID

        Geometry::LineSegment *nextLinesegment = geometry.getNextLineSegment(linesegment);

        //this is true only if it arrived at destination
        if(linesegment == nextLinesegment)
        {
            //for linear speed, we only use the first component of 3D linear velocity "linear.x" to represent the speed "v"
            speed = 0.0;
            //for angular speed, we only use the third component of 3D angular velocity "angular.z" to represent the speed "w" (in radian)
            omega = 0.0;
        }
        else
        {
            double targetDistanceEnd = geometry.getDistance(current_pos, linesegment->endP);
            double targetDistanceStart = geometry.getDistance(current_pos, linesegment->startP);

            //Distance Error
            double distError = 0.0;

            double targetAnglePerpen = geometry.getGradient(current_pos, linesegmentPerpen.endP);

            VECTOR2D target = linesegment->endP;
            double targetAngle = geometry.getGradient(current_pos, target);
            double distanceToClosestPath = abs(linesegment->distanceToAObj);

            //Error calculation based on angles
            if (distanceToClosestPath < distanceConst) {

                // This goes towards the end point of the line segment-> Select vary small distanceConst

                //angleError = targetAngle - Odom_yaw;
                double directional = targetAngle;

                double discripancy = targetAnglePerpen - directional;
                discripancy = geometry.correctAngle(discripancy);

                //Adding some potion of perpendicular angle to the error
                discripancy = 0.5* discripancy / distanceConst * abs(distanceToClosestPath);

                double combined = targetAngle + discripancy;

                angleError = combined - Odom_yaw;

            } else {
                //This goes toward the minimum distance point of the path
                angleError = targetAnglePerpen - Odom_yaw;
            }

            speed = maxSpeed;

            //If lines are long and it has sharp edges
            if (nextLinesegment->distance > 3.0 && linesegment->distance > 3.0) {
                //angleError correction for sharp turns
                if (targetDistanceEnd < 0.5) {
                    double futureAngleChange = nextLinesegment->gradient - linesegment->gradient;
                    futureAngleChange = geometry.correctAngle(futureAngleChange);

                    //Adding some potion of perpendicular angle to the error
                    futureAngleChange = futureAngleChange / distanceConst * abs(targetDistanceEnd);

                    double combined = targetAngle + futureAngleChange;

                    angleError = combined - Odom_yaw;
                }

                //Velocity Error Calculation for sharp turns
                if (targetDistanceStart < 0.7 || targetDistanceEnd < 0.7) {

                    double targetDistance = targetDistanceEnd;

                    if (targetDistanceStart < targetDistanceEnd)
                        targetDistance = targetDistanceStart;

                    double speedError = 0.3 * maxSpeed * exp(-abs(targetDistance));

                    speed = pidVelocity.calculate(maxSpeed, -speedError);
                }

            }

            //Error Angle correction for large angles
            angleError = geometry.correctAngle(angleError);
            //PID for the angle
            omega = pidTheta.calculate(0, -angleError);
        }

        ROS_INFO("Odom_yaw %f, Angle Error: %f , omega: %f Speed %f, Speed Error: %f , speedSet: %f", Odom_yaw,
                    angleError, omega, Odom_v, speedError, speed);

        //for linear speed, we only use the first component of 3D linear velocity "linear.x" to represent the speed "v"
        tw_msg.linear.x = speed;
        //for angular speed, we only use the third component of 3D angular velocity "angular.z" to represent the speed "w" (in radian)
        tw_msg.angular.z = omega;

        //publish this message to the robot
        cmd_vel_pub.publish(tw_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}


/*
 * This function calculate euclidean distance between two given points
 * Remove sqrt if performance hungry
 **/
double getDistance(Point &p1, Point &p2) {
    return sqrt(pow((p1.x - p2.x), 2) + pow((p1.y - p2.y), 2));
}


