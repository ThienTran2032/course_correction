#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <cmath>
#include <vector>
#include <geometry_msgs/PointStamped.h>


// Define the ConeData structure to store cone information
struct ConeData
{
    double x;
    double y;
    double z;
};

// Function to calculate the distance between two points
double distance(double x1, double y1, double x2, double y2)
{
    return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
}

// Function to calculate the midpoint between two cones
ConeData calculate_midpoint(const ConeData& cone1, const ConeData& cone2)
{
    ConeData midpoint;
    midpoint.x = (cone1.x + cone2.x) / 2.0;
    midpoint.y = (cone1.y + cone2.y) / 2.0;
    midpoint.z = (cone1.z + cone2.z) / 2.0;
    
    return midpoint;
}


// Global vector to store cone clusters
std::vector<std::vector<ConeData>> cone_clusters;

// Publisher for the marker
ros::Publisher marker_pub;
ros::Publisher midpoint_coord_pub;


// Number of consecutive clusters to average
const int num_clusters_to_average = 10;

void cluster_input(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*cloud_msg, cloud);

    std::vector<ConeData> cones; 
    pcl::PointXYZ previous_point = cloud[0];
    for (size_t i = 1; i < cloud.size(); ++i)
    {
        pcl::PointXYZ current_point = cloud[i];
        double distance_to_last_point = distance(current_point.x, current_point.y, previous_point.x, previous_point.y);

        if (distance_to_last_point < 1)
        {
            ConeData cone_point;
            cone_point.x = current_point.x;
            cone_point.y = current_point.y;
            cone_point.z = current_point.z;
            cones.push_back(cone_point);

            ROS_INFO("Detected Cone: x=%.2f, y=%.2f, z=%.2f", cone_point.x, cone_point.y, cone_point.z);
        }
        else
        {
            if (!cones.empty())
            {

                cone_clusters.push_back(cones);
                cones.clear();
            }
        }

        previous_point = current_point;
    }

    // Store the last group of cones (if any) as a separate cluster
    if (!cones.empty())
    {
        cone_clusters.push_back(cones);
    }

    // Check if we have enough clusters to calculate the average midpoint
    if (cone_clusters.size() >= num_clusters_to_average)
    {
        // Calculate the average midpoint over num_clusters_to_average consecutive clusters
        ConeData avg_midpoint = {0.0, 0.0, 0.0};
        for (int i = 0; i < num_clusters_to_average; ++i)
        {
            avg_midpoint.x += cone_clusters[i][0].x;
            avg_midpoint.y += cone_clusters[i][0].y;
            avg_midpoint.z += cone_clusters[i][0].z;
        }
        avg_midpoint.x /= num_clusters_to_average;
        avg_midpoint.y /= num_clusters_to_average;
        avg_midpoint.z /= num_clusters_to_average;

        //ROS_INFO("Average Midpoint: x=%.2f, y=%.2f, z=%.2f", avg_midpoint.x, avg_midpoint.y, avg_midpoint.z);

        geometry_msgs::PointStamped midpoint_msg;
        midpoint_msg.header.frame_id = cloud_msg->header.frame_id;
        midpoint_msg.header.stamp = ros::Time::now();
        midpoint_msg.point.x = avg_midpoint.x;
        midpoint_msg.point.y = avg_midpoint.y;
        midpoint_msg.point.z = avg_midpoint.z;

        midpoint_coord_pub.publish(midpoint_msg);


        // Create and publish the marker for the average midpoint
        visualization_msgs::Marker marker;
        marker.header.frame_id = cloud_msg->header.frame_id;
        marker.header.stamp = ros::Time::now();
        marker.ns = "midpoint_marker";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = avg_midpoint.x;
        marker.pose.position.y = avg_midpoint.y;
        marker.pose.position.z = avg_midpoint.z;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.2; 
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        marker.color.a = 1.0; 
        marker.color.r = 1.0; 
        marker.color.g = 0.0;
        marker.color.b = 0.0; 

        marker_pub.publish(marker);

        // Clearing stored clusters
        cone_clusters.clear();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cone_midpoint_node");
    ros::NodeHandle nh;

    ros::Subscriber cloud_subscriber = nh.subscribe("/cone_clusters", 10000, cluster_input);

    marker_pub = nh.advertise<visualization_msgs::Marker>("/cone_set_1_midpoint_marker", 10000);
    midpoint_coord_pub = nh.advertise<geometry_msgs::PointStamped>("/cone_set_1_midpoint_coordinates", 10000);


    ros::spin();

    return 0;
}
