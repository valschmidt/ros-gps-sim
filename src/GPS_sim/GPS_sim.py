#!/usr/bin/env python
#
# Add license and copyright.

import rospy
import gps_sim.srv
import datetime
import math
import re
from operator import xor
from nmea_msgs.msg import Sentence
import geodesy.utm

class gps_sim_node:

    def __init__(self):
        self.serv = None
        self.default_UTM_zone = None
        self.UTM_origin = None
        self.geographic_Origin = None

    def init_service(self):

        # The model implemented here is that the vehicle will operate in a 
        # single UTM zone, in meters from a geographic origin. The zone and 
        # origin will be stored in the parameter server. The 
        # choice of zone will be determined either by the user, the mission 
        # plan, or the first good GPS fix. 
 
#        if rospy.has_param('/geodesy/default_UTM_zone'):
#            self.default_UTM_zone = rospy.get_param('/geodesy/default_UTM_zone')
#        else:
#            rospy.logwarn('No Default UTM Zone set!')
#            self.default_UTM_zone = None
#            
        if rospy.has_param('/geodesy/LatOrigin') and rospy.has_param('/geodesy/LonOrigin'):
            self.geographic_Origin = geodesy.utm.GeoPoint(rospy.get_param('/geodesy/LatOrigin'),rospy.get_param('/geodesy/LonOrigin'),0)
            self.UTM_origin = geodesy.utm.fromMsg(self.geographic_Origin)
        else:
            rospy.logdebug('No Geographic Origin found in Parameter Server, setting Lat:0,Lon;0,Alt:0')
            self.geographic_Origin = geodesy.utm.GeoPoint(0,0,0)
            self.UTM_origin = geodesy.utm.fromMsg(self.geographic_Origin)
            
        # Set a parameter for the UTM origin. Useful moving forward. 
        # Q: Is it faster to calculate the UTM origin or retrieve it? Dunno.
        rospy.set_param('/geodesy/UTMOrigin',self.UTM_origin)            

        rospy.loginfo('Setting /geodesy/UTMOrigin to %0.9f, %0.9f, Zone: %s' % 
        (self.UTM_origin.easting, 
         self.UTM_origin.northing, 
         str(self.UTM_origin.zone) + self.UTM_origin.band))        
        
        rospy.init_node('gps_sim_server',log_level = rospy.DEBUG)
        
        # Need to update the service message to use standard pose, geometry or nmea msgs.
        self.serv = rospy.Service('gps_sim', gps_sim.srv.latlon_from_xy, self.sub_generateNMEA_callback)
        rospy.logdebug("GPS Sim Service Running...")

    def sub_generateNMEA_callback(self,data):
        ''' Service message type is 

        @param    geometry_msgs/PostStamped Pose
        
        Returns 
        nmea_msgs/Sentence GPS_raw
        '''
        #rospy.logdebug(data)
        
        pose = data.PoseStamped.pose
        hdr = data.PoseStamped.header
        
        # Get time stamp from sent data (or from PC)
        dt = datetime.datetime.utcfromtimestamp(hdr.stamp.secs + hdr.stamp.nsecs/1.0e9)
        
        if self.UTM_origin is None:
            rospy.logerr('UTM Origin is not set.')
            return None

        # Add local coordinate frame to UTM origin. 
        # The assumption here is that navigation is done in a local reference frame
        # in Eastings and Northings from UTM_origin. This keeps numeric values 
        # small and prevents confounding precision errors. 
        # Convert UTM to LAT/LON
        z, b = self.UTM_origin.gridZone()

        easting_to_convert = self.UTM_origin.easting + pose.position.x
        northing_to_convert = self.UTM_origin.northing + pose.position.y

        rospy.logdebug("Got %s and %s, UTMOrigin is %0.2f,%0.2f,%s, Converting %0.2f, %0.2f" %
        (pose.position.x, pose.position.y,
         self.UTM_origin.easting,self.UTM_origin.northing, str(z)+b,
        easting_to_convert, northing_to_convert ))
        
        p = geodesy.utm.UTMPoint(easting_to_convert,
                                 northing_to_convert,
                                 pose.position.z, 
                                zone = z, 
                                band = b)

        if p.valid:
            pLL = p.toMsg()
        else:
            rospy.logerr('Error converting UTM position to Lat/Lon')
            return None
        
        # Format timestamp for NMEA string
        timestr = dt.strftime('%H%M%S.%f')
        # Format LAT/LON for NMEA string.
        latstr, lonstr = self.format_latlon_for_NMEA(pLL.latitude,pLL.longitude)
        # Assemble faked NMEA string
        nmea_str = ('$GPGGA,' + 
        timestr + ',' +
        latstr + ',' +
        lonstr + ',' +
        '08,0.9,1.0,M,32.0,M,1,0,*')
        
        G = Sentence()
        G.sentence = nmea_str + self.checksum(nmea_str)
        #G.header.seq = 1
        #G.header.frame_id = 1
        #unixtime = (dt - datetime.datetime.utcfromtimestamp(0)).total_seconds()
        #G.header.stamp.secs = int(unixtime)
        #G.header.stamp.nsecs = int( (unixtime - math.floor(unixtime)) * 1e9 )
        rospy.loginfo('SENDING: ' + G.sentence)
        
        return G
        # Return NMEA string.
        #        return "%s, %s and %s" % (timestr,pose.position.x, pose.position.y)
        #return "{0:%0.6f}\t{1:%0.6f}".format(self.x,self.y)

    def xy_to_latlon(self,latitude,longitude):
        pass
        #UTM = geodesy.utm.fromLatLong
        #return "{0:%0.6f}\t{1:%0.6f}".format(self.x, self.y)

    def format_latlon_for_NMEA(self,latitude,longitude):
        ''' Format decimal latitude and longitude in NMEA format
        
        @param: latitude:  latitude in decimal degrees
        @param: longitude: longitude in decimal degrees
        
        '''
        rospy.loginfo('Formatting %0.9f,%0.9f' % (latitude,longitude))
        if latitude < 0:
            latHem = 'S'
            latitude = abs(latitude)
        else:
            latHem = 'N'
        if longitude < 0:
            lonHem = 'W'
            longitude = abs(longitude)
        else:
            lonHem = 'E'

        
        # Handle possible rounding issue that results in minutes = 60
        # TODO: Create a better python function that checks for equality.
        lat_min =  (latitude - math.floor(latitude)) * 60.0
        if abs((latitude - math.floor(latitude))-1) < 1e-10:
            latitude = latitude + 1.0
            lat_min = 0.0
        lon_min = (longitude - math.floor(longitude)) * 60.0
        if abs((longitude - math.floor(longitude))-1) < 1e-10:
            longitude = longitude + 1.0
            lon_min = 0.0
            
        
        latstr = '%d%012.9f,%s' % (math.floor(latitude), lat_min, latHem)
        lonstr = '%d%012.9f,%s' % (math.floor(longitude), lon_min , lonHem)

        return (latstr,lonstr)
        
    def checksum(self, msg, verify = False):
        ''' 
        A function to calculate and return a checksum of a NMEA string.
 
        @param verify: When specified as True, checksum returns True/False
        rather than the acutal checksum value. 

        '''
        # exp = '(?P<match>\$.*)\*(?P<chksum>..)'
        exp = '(?P<match>\$.*)\*'
        
        m = re.search(exp,msg)
        if m:                
            data = m.group('match')
            tmp = map(ord, data[1:])
            checksum = hex(reduce(xor, tmp))
            if checksum.__len__() == 3:
                checksum = checksum[0:2] + '0' + checksum[2] 
            if verify:
                return checksum[2:4].upper() == m.group('chksum')
            else:
                return checksum[2:4].upper()

        else:
            return None

    def isclose(a, b, tol=1e-10):
        '''https://stackoverflow.com/questions/5595425/what-is-the-best-way-to-compare-floats-for-almost-equality-in-python'''
        return abs(a-b) <= tol

    def run(self):
            rospy.spin()

#if __name__ == '__main__':
#
#    if rospy.has_param('/LatOrigin') and rospy.has_param('/LonOrigin'):
#        LatOrigin = rospy.get_param('/LatOrigin')
#        LonOrigin = rospy.get_param('/LonOrigin')
#    else:
#        LatOrigin = 43.072255
#        LonOrigin = -70.710948
#        rospy.set_param('/LatOrigin',LatOrigin)
#        rospy.set_param('/LonOrigin',LonOrigin)
#
#    print "LatOrigin: %f" % LatOrigin
#    print "LonOrigin: %f" % LonOrigin
#
#    g = gps_sim_node()
#    try:
#        g.init_service()
#        g.run()
#    except rospy.ROSInterruptException:
#        print "Error of unknown origin"
#        pass


