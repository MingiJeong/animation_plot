#!/usr/bin/env python

"""
gds_tools based catabot realtime plot
"""
import rospy
import rospkg
import csv
import numpy as np
import matplotlib.pyplot as plt
import message_filters
import matplotlib.animation as animation
from matplotlib import style

from ysi_exo.msg import Sonde
from sensor_msgs.msg import NavSatFix, Range, Temperature
from mavros_msgs.msg import VFR_HUD, WaypointReached, StatusText
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from timeit import default_timer as timer
from std_msgs.msg import Float64


# designation of csv file
#global CSV_FILE_PATH
CSV_FILE_PATH = ''
#global data_read
data_read = False
initial_time = None

your_list = []
row = None

# initialization of compass (original msg some error)
cur_compass = Range()


# csv file save
def append_file(data_array):
    with open(CSV_FILE_PATH, 'a') as csv_file:
        global data_read
        writer = csv.writer(csv_file)
        writer.writerow(data_array)
        data_read = True
        #print("csv file saved!", data_read, CSV_FILE_PATH)


def gps_sonde_callback(sonde_msg, gps_msg, local_velocity_msg):
    data_array = [gps_msg.header.stamp.to_sec(), cur_compass.header.stamp.to_sec(), float(cur_compass.range),
                float(local_velocity_msg.header.stamp.to_sec()),
                float(local_velocity_msg.twist.twist.linear.x), float(local_velocity_msg.twist.twist.linear.y), float(local_velocity_msg.twist.twist.linear.z),
                float(local_velocity_msg.twist.twist.angular.x), float(local_velocity_msg.twist.twist.angular.y), float(local_velocity_msg.twist.twist.angular.z),
                float(sonde_msg.header.stamp.to_sec())]
    data_array.extend(sonde_msg.data)
    append_file(data_array)

def compass_callback(compass_msg):
    cur_compass.header.stamp = rospy.Time.now()
    cur_compass.range = compass_msg.data

def spin(fig,ax):
    """ROS node to listen to robot global position, depth, and EXO sonde measurements and write to csv file."""
    global CSV_FILE_PATH
    CSV_FILE_PATH = r'//home/minkbrook/Desktop/boat-house-turning.csv'
    # ROS nitialization and parameters.


    #rospack = rospkg.RosPack()
    #CSV_FILE_PATH = rospack.get_path('gds_tools') + "/data/" + rospy.get_param('~csv_file_name')

    # catabot
    rospy.Subscriber('mavros/global_position/compass_hdg', Float64, compass_callback)
    sonde_sub = message_filters.Subscriber('sonde', Sonde)
    gps_sub = message_filters.Subscriber('mavros/global_position/global', NavSatFix)
    #airspeed_sub = message_filters.Subscriber('mavros/vfr_hud', VFR_HUD)
    #sonar_sub = message_filters.Subscriber('sonar', Range)
    #gps_velocity_sub = message_filters.Subscriber('mavros/global_position/raw/gps_vel', TwistStamped)
    local_velocity_sub = message_filters.Subscriber('mavros/local_position/odom', Odometry)

    # catabot
    #ts = message_filters.ApproximateTimeSynchronizer([sonde_sub, gps_sub, airspeed_sub, sonar_sub, gps_velocity_sub, local_velocity_sub], 100, 0.5, allow_headerless=True)
    ts = message_filters.ApproximateTimeSynchronizer([sonde_sub, gps_sub, local_velocity_sub], 100, 0.5, allow_headerless=True)
    ts.registerCallback(gps_sonde_callback)

    #data_plot(fig,ax1)
    rospy.spin()


if __name__ == "__main__":
    try:
        rospy.init_node("real_time_saver", anonymous=False)
        rospy.sleep(1)
        """
        definition of the figures
        """
        """# todo remove unnecessary"""
        #boundary of plot
        #style.use('fivethirtyeight')
        global fig, ax1
        fig = plt.figure()
        ax1 = plt.subplot2grid((2, 2), (0, 0), rowspan=1, colspan=1)
        ax2 = plt.subplot2grid((2, 2), (1, 0), rowspan=1, colspan=1)
        ax3 = plt.subplot2grid((2, 2), (0, 1), rowspan=2, colspan=1)
        #ax1 = fig.add_subplot(1,1,1)
        plt.ion()
        mng = plt.get_current_fig_manager()
        mng.full_screen_toggle()
        ax = (ax1, ax2, ax3)
        """
        main plotter
        """
        while not rospy.is_shutdown():
            spin(fig, ax)

    except rospy.ROSInterruptException:
        pass
