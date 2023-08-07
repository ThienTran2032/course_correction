#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Path.h>
#include <cmath>
#include <vector>

// External testing node variables. Replace this with second midpoint.
int third_point_x = 2;
int third_point_y = -2;
int third_point_z = 0;


// Control node variables
int control_mid_point_x = 3;
int control_mid_point_y = 1;
int control_mid_point_z = 0;


struct SplineNode
{
    double x;
    double y;
    double z;
};


// Function to compute Rom spline points
std::vector<SplineNode> computeSpline(const SplineNode& p0, const SplineNode& p1, const SplineNode& p2, const SplineNode& p3, int num_points)
{
    std::vector<SplineNode> points;
    double step_size = 1.0 / num_points;

    for (int i = 0; i <= num_points; ++i)
    {
        double t = step_size * i;
        double t2 = t * t;
        double t3 = t2 * t;

        SplineNode point;
        point.x = 0.5 * ((2 * p1.x) + (-p0.x + p2.x) * t + (2 * p0.x - 5 * p1.x + 4 * p2.x - p3.x) * t2 + (-p0.x + 3 * p1.x - 3 * p2.x + p3.x) * t3);
        point.y = 0.5 * ((2 * p1.y) + (-p0.y + p2.y) * t + (2 * p0.y - 5 * p1.y + 4 * p2.y - p3.y) * t2 + (-p0.y + 3 * p1.y - 3 * p2.y + p3.y) * t3);
        point.z = 0.5 * ((2 * p1.z) + (-p0.z + p2.z) * t + (2 * p0.z - 5 * p1.z + 4 * p2.z - p3.z) * t2 + (-p0.z + 3 * p1.z - 3 * p2.z + p3.z) * t3);

        points.push_back(point);
    }

    return points;
}

ros::Publisher spline_line_pub;
SplineNode midpoint;
bool has_midpoint = false; // Flag to check if the midpoint has been received

void midpoint_spline(const geometry_msgs::PointStamped::ConstPtr& node_msg)
{
    if (!has_midpoint)
    {
        // The first received point is assumed to be the midpoint
        midpoint.x = node_msg->point.x;
        midpoint.y = node_msg->point.y;
        midpoint.z = node_msg->point.z;
        has_midpoint = true;
    }

    else
    {
        // The second received point is assumed to be the third point
        // The third point will be defined by the second midpoint
        SplineNode third_point;
        third_point.x = third_point_x;
        third_point.y = third_point_y;
        third_point.z = third_point_z;

        // Additional control point
        SplineNode control_third_point;
        control_third_point.x = third_point.x;
        control_third_point.y = third_point.y;
        control_third_point.z = third_point.z;

        // Additional control point 
        SplineNode control_mid_point;
        control_mid_point.x = control_mid_point_x;
        control_mid_point.y = control_mid_point_y;
        control_mid_point.z = control_mid_point_z;


        // Compute spline
        std::vector<SplineNode> spline_points;
        std::vector<SplineNode> points;

        // spline between the origin and midpoint
        points = computeSpline(SplineNode{0.0, 0.0, 0.0}, SplineNode{0, 0, 0}, midpoint, control_third_point, 100);
        spline_points.insert(spline_points.end(), points.begin(), points.end());

        // spline between the midpoint and the third point
        points = computeSpline(midpoint, midpoint, third_point, control_mid_point, 100);
        spline_points.insert(spline_points.end(), points.begin(), points.end());

        // Publish the spline points as a trajectory
        nav_msgs::Path trajectory_msg;
        trajectory_msg.header.stamp = ros::Time::now();
        trajectory_msg.header.frame_id = "velodyne";

        for (const auto& spline_point : spline_points)
        {
            geometry_msgs::PoseStamped pose;
            pose.pose.position.x = spline_point.x;
            pose.pose.position.y = spline_point.y;
            pose.pose.position.z = spline_point.z;
            trajectory_msg.poses.push_back(pose);
        }

        spline_line_pub.publish(trajectory_msg);

        // Reset the has_midpoint flag for the next set of spline nodes
        has_midpoint = false;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "spline_node");
    ros::NodeHandle nh;

    ros::Subscriber spline_nodes_sub = nh.subscribe("cone_set_1_midpoint_coordinates", 10000, midpoint_spline);
    spline_line_pub = nh.advertise<nav_msgs::Path>("spline_trajectory", 10000);
    

    ros::spin();
    return 0;
}

