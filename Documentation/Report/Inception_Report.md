Inception Report
=====


Aim
-------
The main goal of the project is to pilot a drone,  upon detection of a potential “threat”,  from one location to another in an environment, based on input from a setup involving (fixed) stereo cameras.

 The stereo cameras will be used to detect threats and provide depth information of objects in the environment to the drone. This information will be used by the drone to navigate it’s way.

Background Material 
----
It is well-known that depth information ( 3D coordinates ) of an object can be approximately recovered based on 2D images from  a stereo camera system ,ie two or more cameras viewing the same object.

The first step is to find internal parameters ( focal length, principal point, distortion coefficients ) of both the cameras. This information is then used to find the relative pose ( Rotation and Translation ) between the two cameras ( coordinate systems ). After this process the images obtained by both cameras are rectified by projecting their image planes onto a common plane parallel to their baseline ( line connecting the two cameras centres ) in order to ease the process of stereo matching. 

Once the correspondences are obtained, a triangulation is done, by finding intersection of lines (in theory ) from camera centers through the projected points, in 3D space. This intersection has to be the 3D point whose image is captured in both cameras. This is only true if all other information, such as interest point detection, stereo matching, etc are not error prone ( which is usually not the case ). In theory, the result of the triangulation is the depth ( 3D coordinates in camera coordinate system ) for each pixel.

This depth information can be used to construct a 3D map detect threats and obstacles in the environment which will be used to guide the drone, avoiding obstacles and navigating to the “threat” location. “threat” here has not been precisely stated as this will be decided in the future stages of the project.  

Navigation of the drone may also be error prone and hence, if the map of the environment is known correctly, then the drone can be localised using probabilistic particle filters like Monte Carlo Localisation ( MCL ).. The probabilistic model is based on bayesian methods which requires input from a sensor which is then used to “kill” particles that incorrectly represent the drones location.This will require the use of onboard sensors, which may be an extension to the project.



Project deliverables, fallbacks & extensions 
-------- 

##### Project Deliverables
 
1) Stereo camera calibration
2) Evaluation of the system with an easily detectable object ( eg: red ball )
3) Integrate ROS with openCV C++ API.
4) Drone control using ROS 
5) Mapping of environment using stereo cameras.
6) Waypoint-navigation
7) Object classification.
8) Localisation using particle filter ( extension? ) 



##### Fallbacks 

If the project is difficult to extend to practical environment, then might restrict to lab environment.


##### Extensions

1.  Currently the drone is not relying on any on-board sensors (eg:- sonar sensor ,or laser-scanner for depth measurement ). The reliability of using just the camera for information may not be sufficient in a practical situation. Hence localisation using particle filter ( may be required ) can be implemented.

2. If the image processing computations are a bottleneck for computation on the current device ( my Laptop ), may offload computation to a more powerful device ( like a GPU ). 



##### Summary of Risks

1. Drone localisation may not be as simple since it is in 3D space ( compared to localisation in 2D space ) environment.


2. Triangulation process sounds trivial in theory but may be prone to several types of noise such a geometric noise from interest point detection, which may mean that the lines do not intersect at a 3D point and hence the triangulation is estimated based on some optimality criteria. For example in openCV it is done using Least squares ( using SVD ) 


3. As mentioned in Extension 3 a), the drone may be required to be equipped with an on-board  sensor to improve the confidence in the information from the cameras. 


4. The throughput of information transmitted by the wireless stereo camera setup might pose as a bottleneck in processing information using the cameras and may be much slower than drone response time.

