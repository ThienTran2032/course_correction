#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

ros::Publisher cone_pub;

double threshold = 0.08;
int max_iterations = 500;
double cone_cluster_tolerance = 0.07;
int cone_min_cluster_size = 50;
int cone_max_cluster_size = 1000;


void lidarCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    ROS_INFO("Running Segmetation");
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *cloud);

    // Remove ground plane using RANSAC
    pcl::PointIndices::Ptr ground_indices(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr ground_coefficients(new pcl::ModelCoefficients);
    pcl::SACSegmentation<pcl::PointXYZI> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(max_iterations);
    seg.setDistanceThreshold(threshold);
    seg.setInputCloud(cloud);
    seg.segment(*ground_indices, *ground_coefficients);

    // Extract non-ground points (remove ground plane)
    pcl::PointCloud<pcl::PointXYZI>::Ptr non_ground_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::ExtractIndices<pcl::PointXYZI> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(ground_indices);
    extract.setNegative(true);
    extract.filter(*non_ground_cloud);


    // Perform cone Clustering
    std::vector<pcl::PointIndices> cone_clusters;
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud(non_ground_cloud);
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance(cone_cluster_tolerance);
    ec.setMinClusterSize(cone_min_cluster_size);
    ec.setMaxClusterSize(cone_max_cluster_size);
    ec.setSearchMethod(tree);
    ec.setInputCloud(non_ground_cloud);
    ec.extract(cone_clusters);

    // Publish the cone clusters
    pcl::PointCloud<pcl::PointXYZI>::Ptr cone_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    for (const auto &cluster_indices : cone_clusters)
    {
        for (const auto &index : cluster_indices.indices)
        {
            cone_cloud->push_back(non_ground_cloud->points[index]);
        }
    }

    sensor_msgs::PointCloud2 cone_msg;
    pcl::toROSMsg(*cone_cloud, cone_msg);
    cone_msg.header = msg->header;
    cone_pub.publish(cone_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "floor_filter_segmentation_node");
    ros::NodeHandle nh;

    ros::Subscriber lidar_sub = nh.subscribe("/filtered_geometry", 10000, lidarCallback);
    cone_pub = nh.advertise<sensor_msgs::PointCloud2>("/floor_seg", 10000);

    ros::spin();
    return 0;
}