#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>
#include <pcl/common/distances.h>
#include <pcl/common/common.h>
#include <geometry_msgs/Point.h>
#include <cmath>
#include <visualization_msgs/Marker.h>
#include <pcl/filters/voxel_grid.h>

ros::Publisher cluster_publisher;
ros::Publisher marker_publisher;

// Cluster Extraction Parameters
float cluster_tolerance = 0.07; // Set the maximum distance between points within a cluster
int min_cluster_size = 10;     // Set the minimum number of points that must belong to a cluster
int max_cluster_size = 1500;     // Set the maximum number of points that can belong to a cluster

// Height and Base Diameter of Reference Cone
float cone_height_max = 0.38;  // 32cm
float cone_height_min = 0.15;
float cone_base_max = 0.2;
float cone_base_min = 0.05;    // 20cm

// Fixed Distance between Lidar and Camera.
float lidar_camera_distance = -1; // Changed based on the Car Length
int num_cones = 2;

// String Variables
std::string direction; // LHS or RHS of the lidar



void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
    // Convert ROS PointCloud2 to PCL PointCloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*cloud_msg, *cloud);

    // Cluster Extraction
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance(cluster_tolerance);
    ec.setMinClusterSize(min_cluster_size);
    ec.setMaxClusterSize(max_cluster_size);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    if (cluster_indices.empty())
    {
        ROS_INFO("No cones found.");
        return;
    }

    int num_detected_cones = 0;
    for (const auto& indices : cluster_indices)
    {
        if (num_detected_cones >= num_cones)
        {
            break;
        }


        pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        for (const auto& index : indices.indices)
        {
            cluster_cloud->push_back(cloud->points[index]);
        }

        // Calculate distance of each cluster centroid from a reference point
        pcl::PointXYZ lidar_reference_point(0.0, 0.0, 0.0);                         // These will vary based on the dispacement
        pcl::PointXYZ camera_reference_point(lidar_camera_distance, 0.0, 0.0);      // These will vary based on the dispacement. XYZ

        for (const auto& indices : cluster_indices)
        {
            pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZI>);
            for (const auto& index : indices.indices)
            {
                cluster_cloud->push_back((*cloud)[index]);
            }

            // Calculate centroid of the cluster
            pcl::CentroidPoint<pcl::PointXYZI> centroid;
            for (const auto& point : cluster_cloud->points)
            {
                centroid.add(point);
            }

            // Get the number of points in the cluster
            int num_points = cluster_cloud->size();

            pcl::PointXYZ cluster_centroid;
            centroid.get(cluster_centroid);

            // geometry_msgs::Point centroid_msg;
            // centroid_msg.x = cluster_centroid.x;
            // centroid_msg.y = cluster_centroid.y;
            // centroid_msg.z = cluster_centroid.z;
            //cone12_midpoint_publisher.publish(centroid_msg);

            // ROS_INFO("Centroid x: %f", centroid_msg.x);
            // ROS_INFO("Centroid y: %f", centroid_msg.y);


            // Calculate Euclidean distance from the reference point to the cluster centroid
            float lidar_distance = pcl::euclideanDistance(lidar_reference_point, cluster_centroid);
            //float camera_distance = pcl::euclideanDistance(camera_reference_point, cluster_centroid);
            

            // Pull cluster centroid height and multiply it by 4 (cone cluster centroid is 1/4th the height)
            // float cluster_height = cluster_centroid.z * 4;


            // Calculate the relative angle between camera and lidar (rads)
            //float theta_cam_lidar = (lidar_distance*lidar_distance - camera_distance*camera_distance - lidar_camera_distance*lidar_camera_distance)/ (2 * camera_distance * lidar_camera_distance);

            //float cluster_base_calibration_factor = (20.5/2) / 2; /


            // Calculate cluster base width and height
            pcl::PointXYZI min_pt, max_pt;
            pcl::getMinMax3D(*cluster_cloud, min_pt, max_pt);
            float cluster_base = max_pt.x - min_pt.x;
            float cluster_height = max_pt.z - min_pt.z;


            // Calculate cone ratio based on triangulation
            //float height_ratio = (cone_height_max / cluster_height);
            //float base_ratio = (cone_base_max / cluster_base);
        

            if ((cone_height_max >= cluster_height) && (cluster_height >= cone_height_min) && (cone_base_max >= cluster_base) && (cluster_base >= cone_base_min))
            {
                // Publish the cluster point cloud
                sensor_msgs::PointCloud2 cluster_msg;
                pcl::toROSMsg(*cluster_cloud, cluster_msg);
                cluster_msg.header = cloud_msg->header;
                cluster_publisher.publish(cluster_msg);

                // For debugging
                // ROS_INFO("Lidar Distance to cone cluster: %f", lidar_distance);
                // ROS_INFO("Camera Distance to cone cluster: %f", camera_distance);
                ROS_INFO("Cone height: %f", cluster_height);
                ROS_INFO("Cone base: %f", cluster_base);
                ROS_INFO("Number of points in the cluster: %d", num_points);
                
                // ROS_INFO("Centroid x: %f", cluster_centroid.x);
                // ROS_INFO("Centroid y: %f", cluster_centroid.y);

                // ROS_INFO("x min: %f", min_pt.x);
                // ROS_INFO("x max: %f", max_pt.x);


                // Calculate the average position of cone points
                pcl::PointXYZI avg_point;
                for (const auto& point : cluster_cloud->points) //the average can be removed if it slows down the system too much
                {
                    avg_point.x += point.x;
                    avg_point.y += point.y;
                    avg_point.z += point.z;
                }
                avg_point.x /= cluster_cloud->size();
                avg_point.y /= cluster_cloud->size();
                avg_point.z /= cluster_cloud->size();

                // Publish a single marker at the average position
                visualization_msgs::Marker marker;
                marker.header = cloud_msg->header;
                marker.ns = "cones";
                marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
                marker.action = visualization_msgs::Marker::ADD;
                marker.pose.orientation.w = 1.0;
                marker.scale.x = 0.1;  // Size of the cylinder
                marker.scale.y = 0.1;
                marker.scale.z = 0.2;

                marker.pose.position.x = avg_point.x;
                marker.pose.position.y = avg_point.y;
                marker.pose.position.z = avg_point.z + 0.5;

                //ROS_INFO("Maker X Position: %f", marker.pose.position.x);
                //ROS_INFO("Maker Y Position: %f", marker.pose.position.y);

                //marker.text = "Cluster " + std::to_string(num_detected_cones + 1);


                if (marker.pose.position.y <= 0)// Denote Right
                {
                    direction = "CRight ";
                    marker.color.r = 1.0;
                    marker.color.g = 0.0;
                    marker.color.b = 0.0;
                    marker.color.a = 1.0;
                    marker.text = direction + std::to_string(num_detected_cones + 1) + ": " + std::to_string(lidar_distance);
                    
                    ROS_INFO("CRight %d", num_detected_cones + 1);
                    //marker_publisher.publish(marker);
                }

                if (marker.pose.position.y >= 0) // Denote left cone
                {
                    direction = "CLeft ";
                    marker.color.r = 51.0;
                    marker.color.g = 51.0;
                    marker.color.b = 255.0;
                    marker.color.a = 1.0;
                    marker.text = direction + std::to_string(num_detected_cones + 1) + ": " + std::to_string(lidar_distance);
                    
                    ROS_INFO("CLeft %d", num_detected_cones + 1);
                    //marker_publisher.publish(marker);
                }

                ROS_INFO("%sCone Distance: %f", direction.c_str(), lidar_distance);

                marker_publisher.publish(marker);
                num_detected_cones++;

                // ROS_INFO("Centroid x: %f", centroid_msg.x);
                // ROS_INFO("Centroid y: %f", centroid_msg.y);
            }

            else
            {
                // For debugging purposes
                // ROS_INFO("No Cone Detected");
                // sensor_msgs::PointCloud2 cluster_msg;
                // pcl::toROSMsg(*cluster_cloud, cluster_msg);
                // cluster_msg.header = cloud_msg->header;
                // cluster_publisher.publish(cluster_msg);


                // ROS_INFO("Lidar Distance to cone cluster: %f", lidar_distance);
                // ROS_INFO("Lidar Distance to cone cluster: %f", lidar_distance);
                // ROS_INFO("Cone height: %f", cluster_height);
                // ROS_INFO("Cone base: %f", cluster_base);
                //ROS_INFO("Camera Distance to cone cluster: %f", camera_distance);
                //ROS_INFO("angle: %f", theta_cam_lidar);
            }
            
            
        }

        //num_detected_cones++;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cone_distance_node");
    ros::NodeHandle nh;

    ros::Subscriber cloud_subscriber = nh.subscribe<sensor_msgs::PointCloud2>("/floor_seg", 10000, pointCloudCallback);
    cluster_publisher = nh.advertise<sensor_msgs::PointCloud2>("/cone_clusters", 10000);
    marker_publisher = nh.advertise<visualization_msgs::Marker>("/cone_markers", 10000);


    ros::spin();

    return 0;
}