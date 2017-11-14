ROS Package GPS-SIM
===================
This ROS package, gps-sim, provides a ROS service for mapping from a local projected (specifically UTM) coordinate reference frame to WGS-84 geographic coordinates and representing them as pseudo (faked) NMEA sentances for simulating field robots. 

Projected coordinate systems express position a North, East, Up (NEU) reference frame from a reference location. The reference location must be defined as a ROS parameter. as shown in the follow python code snippet:

    rospy.set_param('/geodesy/LatOrigin',43.0)
    rospy.set_param('/geodesy/LonOrigin',-70.0)

Gps-sim assumes the proejcted coordinate system is the UTM zone of the LatOrigin and LonOrigin. When a PoseStamped message is sent to the gps-sim service with coordinates in the given UTM zone the service will respond by publishing a pseudo GPS NMEA string to nmea_msgs/Sentence 
 
