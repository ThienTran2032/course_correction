#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include <iostream>

// the following can be used to handle point clouds
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// for path planning
#include <geometry_msgs/Point.h>


// Global variables for path planning
double x_path_planning;
double y_path_planning;


// Global variables for pointclouds


void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  // Convert the PointCloud2 message to a PCL point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *cloud);

  // Access the point cloud data
  for (const auto& point : cloud->points)
  {
    // Process the point cloud values
    float x = point.x;
    float y = point.y;
    float z = point.z;

    // Print the point cloud values
    ROS_INFO("Point: x=%f, y=%f, z=%f", x, y, z);
  }
}

void pathPlanningCallback(const geometry_msgs::Point::ConstPtr& msg)
{
  x_path_planning = msg->x;
  y_path_planning = msg->y;

  ROS_INFO("Received path planning coordinates: x=%f, y=%f", x_path_planning, y_path_planning);
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "flag_indicator");
  ros::NodeHandle n;
  
  // Setting up the publisher to advertise to the master
  ros::Publisher flag_publisher = n.advertise<std_msgs::String>("flag_pub", 1000); // Flag


  // Setting up a subscriber to the velodyne topic. wda
  //ros::Subscriber velodyne_pt_subscriber = n.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 10, pointCloudCallback);


  // Setting up a subscriber to the OrbSLAM algorthim and Path Planning algorithm
  // ros::Subscriber orbSLAM_subscriber =
  // ros::Subscriber path_planning_coord_subscriber = n.subscribe<geometry_msgs::Point>("[path_planning_topic]", 10, pathPlanningCallback);


  ros::Rate loop_rate(10);

  std::string start_condition; // Input start to begin
  ROS_INFO("Send Start Condition"); 
  std::cin >> start_condition;

  char user_input = 'r'; // Change this to test the inputs 



  //int count = 0;
  while (ros::ok() && (start_condition == "begin"))
  {
    // std::string flag_input;
    // cin << flag_input;

    // --- lidar_input function --- //
    // Needs to have a set of conditions to measure deviations
    // how can this be worked out.


    // --- reading from algorithms --- //
    // should take inputs from the path planner or SLAM.
    // currently, it is assumed xy.


    // --- post processing --- //
    // compare lidar output to the inputs from the path planner or SLAM.


    // --- Writing the Flags --- //
    // these need to comply with the flags and correct the car.
    std_msgs::String msg;
    std::stringstream ss;

    if (user_input == 'r') // severely off course
    {
      ss << "RED";// << count;
      msg.data = ss.str();      
    }

    if (user_input == 'y') // slight deviation but not critcal
    {
      ss << "YELLOW";// << count;
      msg.data = ss.str();      
    }

    if (user_input == 'g') // On course
    {
      ss << "GREEN";// << count;
      msg.data = ss.str();      
    }

    ROS_INFO("%s", msg.data.c_str());




    //publishing over ROS
    flag_publisher.publish(msg); 


    ros::spinOnce();

    loop_rate.sleep();
    //++count;
  }


  return 0;
}