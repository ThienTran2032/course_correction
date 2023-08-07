
	Resources to get Started
It is recommended to read into the following resources. 

ROS Tutorials
Point Cloud Library (PCL)
Velodyne ROS Drivers
VLP16 Velodyne


To be included in future if the team decides to further develop it.

Camera-LiDAR extrinsic calibration model
RTKLIB for RTK System
Swift Nav GNSS RTK
XSENS IMU


Alternative solution to current GNSS RTK.

GNSS RTK Dual Antenna through Arduino (faster and easier solution to set up. Works for dual antenna application).
Uses ZED F9P.


	Running the Code
Refer to the following GitHUB repository to obtain the code for course correction. The repo should be cloned into the ‘catkin_ws\src’ location. 

GITHUB Code

Likewise, the velodyne ROS driver linked above. Once these two files are in the workspace and workspace is built (catkin build), run the following launch function.

roslaunch course_correction VLP16_course_correction.launch

In doing so, should launch all the node structures with the terminal showing the most important information from the LiDAR (not done yet).


	ROS Nodes, Publishers and Subscriber Diagram
The node structure of the system is as follows.


Figure 1: ROS Nodes and Topics. For any point, you can subscribe to the following topics to observe its output.
Known Issues/Items/bugs that Need Addressing or Revisiting
A list of things that need to be fixed and explored later on.

Improve the reliability of the cone cluster detection algorithm.
Currently, the code classifies the cone based on the difference in the maximum and minimum point cloud value in the x and y axis. Other factors include the expected cluster size.
The node only publishes the clusters that fit this criteria. Note other objects may meet this criteria and thus may need to be modified to improve reliability. Some form of machine/deep learning algorithm can be used here.

Due to the use of RANSAC to remove the ground element, information of the cone is lost. Lowest LiDAR beam on the base of the cone. If a more realistic method of cone detection is needed, cone reconstruction is needed - haven't worked out the best way to do it.


The current code is capable of clustering two cones and finding its midpoint.
Look into how the clustering will change with more cones (up to 6). This would involve some form of triangulation. A means to store the additional clusters for evaluation - this may involve labelling the clusters and pairing them.

Updating the spline code to include additional control points from the midpoints of the cluster pairs. The preliminary code has been set up and just needs to be adjusted based on the input pairs.


Internet/ethernet issues when using LiDAR.
When using the LiDAR and based on the tutorials provided by ROS, my network was set on the same DHCP network and thus would reset whenever connecting to the LiDAR - internet dropout. 
To avoid this, two things can be done, assigning a static IP (which didn't work for me) or not accessing the sensor network address (192.168.1.201).

I found that by connecting to the sensor address this issue would occur. Note that the information on this page isn't necessary as it gets filtered out anyways.
Make sure you do not change any parameters on this page. Altering any configuration and will permanently save the settings. Only thing I recommend changing is the motor RPM (scan speed).






	Other elements that still needs to be developed or Implemented
These are components that should be considered for course correction.

Transposing LiDAR point cloud data onto the image data from the camera.
In doing so, LiDAR can determine the difference between the colour of cones and thus can tell left from right.
Though this information can be determined by viewing the intensity of the cones. The idea behind this stems from FSG but the rules differ for FSAE. Cones in FSG have reflective tape making it easier to characterise the differences between the cones through an intensity vs distance curve. FSAE lacks this and thus, places emphasis on bounding boxes for discerning colour.

Implement the GNSS RTK + IMU system.
Look into dual antennas and its potential use for pose. The idea behind this is that we know the point and the orientation of the car which can then be used to compare with the path planner to evaluate the course.
Implement a Kalman filter for sensor fusion (calibrating the IMU based on the GNSS RTK).

Develop an extrinsic calibration model based on the camera and lidar pose of the car.
Camera mount is above the drivers head and LiDAR at the front of the car. With the fixed distance, a relative pose of the car can be determined and gives the heading information of the car which can be used for course correction.
Transposing bounding boxes from the camera to the LiDAR point clouds.
The pose can be compared to the GNSS RTK + IMU system.

Low level control based on the spline curve. 
Once the definite pose of the car is known, relative control measures can be put in place to steer the car back on course.
If the spline curve is too sharp, the car should stop.
Heavily depends on the constraints set by the mechanical team.



