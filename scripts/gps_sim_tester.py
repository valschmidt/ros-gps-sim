#!/usr/bin/env python

# A script to test the gps simulation service in ROS

import sys
import rospy
import gps_sim.srv
from geometry_msgs.msg import PoseStamped
import time
import math
#import geodesy.utm

def format_latlon_for_NMEA(latitude,longitude):
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

def gps_sim_tester(P):
    rospy.wait_for_service('gps_sim')
    try:
        gps_sim_srv = rospy.ServiceProxy('gps_sim',gps_sim.srv.latlon_from_xy)
        respl = gps_sim_srv(P)
        return respl.GPS_raw
        
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

if __name__ == "__main__":

    # First we need to set define a default UTM zone in the parameter server.
    rospy.set_param('/geodesy/LatOrigin',43.0)
    rospy.set_param('/geodesy/LonOrigin',-70.0)
    # Currently default_UTM_zone is not used. Instead the UTM zone of the lat/lon origin is used as default.
    #z, b = geodesy.utm.gridZone(rospy.get_param('/geodesy/LatOrigin'),rospy.get_param('/geodesy/LonOrigin'))
    #rospy.set_param('/geodesy/default_UTM_zone',str(z)+b)    

    
    if len(sys.argv) == 3:
        x = float(sys.argv[1])
        y = float(sys.argv[2])

        P = PoseStamped()
        P.pose.position.x = x
        P.pose.position.y = y        
        P.pose.position.z = 0.0
        dts = time.time()
        P.header.stamp.secs = int(math.floor(dts))
        P.header.stamp.nsecs = int((dts - P.header.stamp.secs) * 1e9)
        P.header.seq = 1
        
        print "Requesting %s,%s" %(x,y)
        #print "Got: %s" % gps_sim_tester(P)
        print "Got:"
        print gps_sim_tester(P)
    else:
        #print usage()
        eastings = [-10000,
                    -9000,
                    -8000,
                    -7000,
                    -6000,
                    -5000,
                    -4000,
                    -3000,
                    -2000,
                    -1000,
                    0,
                    1000,
                    2000,
                    3000,
                    4000,
                    5000,
                    6000,
                    7000,
                    8000,
                    9000,
                    10000]
        northings = eastings
        
        latitude = [  42.908828113374774,
                    42.917950946628174,   
                    42.927072528907587,
                    42.936192858992008,
                    42.945311935659937,
                    42.954429757689411,
                    42.963546323857976,
                    42.972661632942682,
                    42.981775683720109,
                    42.990888474966347,
                    43.000000005456997,
                    43.009110273967174,
                    43.018219279271520,
                    43.027327020144156,
                    43.036433495358800,
                    43.045538703688564,
                    43.054642643906192,
                    43.063745314783866,
                    43.072846715093320,
                    43.081946843605778,
                    43.091045699091993]
        longitude = [ -70.121029028615055,
                     -70.108942190780567,
                     -70.096851787292564,
                     -70.084757816430269,
                     -70.072660276472774,
                     -70.060559165699033,
                     -70.048454482387882,
                     -70.036346224818033,
                     -70.024234391268081,
                     -70.012118980016524,
                     -69.999999989341703,
                     -69.987877417521872,
                     -69.975751262835175,
                     -69.963621523559610,
                     -69.951488197973106,
                     -69.939351284353450,
                     -69.927210780978314,
                     -69.915066686125314,
                     -69.902918998071911,
                     -69.890767715095450,
                     -69.878612835473234]

        for i in range(eastings.__len__()):
       
            P = PoseStamped()
            P.pose.position.x = eastings[i]
            P.pose.position.y = northings[i]        
            P.pose.position.z = 0.0
            dts = time.time()
            P.header.stamp.secs = int(math.floor(dts))
            P.header.stamp.nsecs = int((dts - P.header.stamp.secs) * 1e9)
            P.header.seq = 1
            latstr,lonstr = format_latlon_for_NMEA(latitude[i],longitude[i])
    
            #print "Requesting %0.2f,%0.2f for %s,%s " % (eastings[i],northings[j],latstr,lonstr)
            #print "Got: %s" % gps_sim_tester(P)
            S = gps_sim_tester(P)
            #print "Got: %s" % S.sentence
            F = S.sentence.split(',')
            print "%s,%s" % (latstr[:-2],lonstr[:-2])
            print "%s,%s" % (F[2],F[4])
            time.sleep(.5)

        