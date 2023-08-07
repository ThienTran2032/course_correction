#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/crop_box.h>

ros::Publisher filtered_cloud_publisher;
float min_intensity = 0;
float max_intensity = 200;

void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
    ROS_INFO("Geometrically Filtering Cloud Data");

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*cloud_msg, *cloud);

    // Filter the point cloud based on intensity values
    pcl::PointCloud<pcl::PointXYZI>::Ptr intensity_filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::ConditionAnd<pcl::PointXYZI>::Ptr intensity_cond(new pcl::ConditionAnd<pcl::PointXYZI>());
    intensity_cond->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(
        new pcl::FieldComparison<pcl::PointXYZI>("intensity", pcl::ComparisonOps::GT, min_intensity)));
    intensity_cond->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(
        new pcl::FieldComparison<pcl::PointXYZI>("intensity", pcl::ComparisonOps::LT, max_intensity)));

    pcl::ConditionalRemoval<pcl::PointXYZI> intensity_cond_removal;
    intensity_cond_removal.setInputCloud(cloud);
    intensity_cond_removal.setCondition(intensity_cond);
    intensity_cond_removal.setKeepOrganized(true);
    intensity_cond_removal.filter(*intensity_filtered_cloud);

    // Apply crop box filtering
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);

    pcl::CropBox<pcl::PointXYZI> crop;
    crop.setInputCloud(intensity_filtered_cloud);
    crop.setMin(Eigen::Vector4f(-0, -1.5, -1, 0)); // Set minimum XYZ values of the ROI
    crop.setMax(Eigen::Vector4f(2.5, 1.5, 2, 0));   // Set X, Y, Z values
    crop.filter(*filtered_cloud);


    sensor_msgs::PointCloud2 filtered_cloud_msg;
    pcl::toROSMsg(*filtered_cloud, filtered_cloud_msg);
    filtered_cloud_msg.header = cloud_msg->header;

    filtered_cloud_publisher.publish(filtered_cloud_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "geometry_filter_node");
    ros::NodeHandle nh;

    ros::Subscriber cloud_subscriber = nh.subscribe("/velodyne_points", 10000, pointCloudCallback);
    filtered_cloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("/filtered_geometry", 10000);

    ros::spin();

    return 0;
}


