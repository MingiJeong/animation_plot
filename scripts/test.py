#!/usr/bin/env python
"""ROS node to listen to robot global position, depth, and EXO sonde measurements and write to csv file."""

import csv
import rospy
import rospkg
import message_filters

from ysi_exo.msg import Sonde
from sensor_msgs.msg import NavSatFix, Range, Temperature
from mavros_msgs.msg import VFR_HUD
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
#from ping_nodelet.msg import Ping
from std_msgs.msg import Float64

CSV_FILE_PATH = ""

cur_compass = Range()

def append_file(data_array):
    with open(CSV_FILE_PATH, 'a') as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(data_array)

# bluerov2
# def gps_sonde_callback(sonde_msg, gps_msg, depth_msg, velocity_msg):
#     data_array = [gps_msg.header.stamp.to_sec(), gps_msg.latitude, gps_msg.longitude,
#                 depth_msg.header.stamp.to_sec(), depth_msg.altitude,
#                 # temp_msg.header.stamp.to_sec(), temp_msg.temperature,
#                 # echosounder_msg.header.stamp.to_sec(), echosounder_msg.distance, echosounder_msg.confidence,
#                 velocity_msg.header.stamp.to_sec(),
#                 velocity_msg.twist.linear.x, velocity_msg.twist.linear.y, velocity_msg.twist.linear.z,
#                 velocity_msg.twist.angular.x, velocity_msg.twist.angular.y, velocity_msg.twist.angular.z,
#                 sonde_msg.header.stamp.to_sec()]
#     data_array.extend(sonde_msg.data)
#     append_file(data_array)


# catabot
#

# TODO: now groundspeed!!!
def gps_sonde_callback(sonde_msg, gps_msg, airspeed_msg, sonar_msg, gps_velocity_msg, local_velocity_msg):
    data_array = [gps_msg.header.stamp.to_sec(), gps_msg.latitude, gps_msg.longitude,
                cur_compass.header.stamp.to_sec(), cur_compass.range,
                airspeed_msg.header.stamp.to_sec(), airspeed_msg.airspeed,
                sonar_msg.header.stamp.to_sec(), sonar_msg.range,
                gps_velocity_msg.header.stamp.to_sec(),
                gps_velocity_msg.twist.linear.x, gps_velocity_msg.twist.linear.y,
                local_velocity_msg.header.stamp.to_sec(),
                local_velocity_msg.twist.twist.linear.x, local_velocity_msg.twist.twist.linear.y, local_velocity_msg.twist.twist.linear.z,
                local_velocity_msg.twist.twist.angular.x, local_velocity_msg.twist.twist.angular.y, local_velocity_msg.twist.twist.angular.z,
                sonde_msg.header.stamp.to_sec()]
    data_array.extend(sonde_msg.data)
    append_file(data_array)

def compass_callback(compass_msg):
    cur_compass.header.stamp = rospy.Time.now()
    cur_compass.range = compass_msg.data


def gps_sonde_node():
    """ROS node to listen to robot global position, depth, and EXO sonde measurements and write to csv file."""
    global CSV_FILE_PATH

    # ROS nitialization and parameters.
    rospy.init_node('gps_sonde_csv_writer', anonymous=False)
    rospy.sleep(1)

    rospack = rospkg.RosPack()
    CSV_FILE_PATH = r'//home/minkbrook/Desktop/boat-house-turning.csv'

    # Initialization of subscribers.
    # bluerov2
    # sonde_sub = message_filters.Subscriber('sonde', Sonde)
    # gps_sub = message_filters.Subscriber('mavros/global_position/global', NavSatFix)
    # depth_sub = message_filters.Subscriber('mavros/vfr_hud', VFR_HUD)
    # # temp_sub = message_filters.Subscriber('mavros/temperature/data_raw', Temperature)
    # # echosounder_sub = message_filters.Subscriber('ping_nodelet/ping', Ping)
    # velocity_sub = message_filters.Subscriber('mavros/local_position/velocity_body', TwistStamped)

    # catabot
    rospy.Subscriber('mavros/global_position/compass_hdg', Float64, compass_callback)
    sonde_sub = message_filters.Subscriber('sonde', Sonde)
    gps_sub = message_filters.Subscriber('mavros/global_position/global', NavSatFix)
    airspeed_sub = message_filters.Subscriber('mavros/vfr_hud', VFR_HUD)
    sonar_sub = message_filters.Subscriber('sonar', Range)
    gps_velocity_sub = message_filters.Subscriber('mavros/global_position/raw/gps_vel', TwistStamped)
    local_velocity_sub = message_filters.Subscriber('mavros/local_position/odom', Odometry)

    # bluerov2
    # ts = message_filters.ApproximateTimeSynchronizer([sonde_sub, gps_sub, depth_sub, velocity_sub], 100, 0.5)
    # ts = message_filters.ApproximateTimeSynchronizer([sonde_sub, gps_sub, depth_sub, velocity_sub], 100, 0.5)
    # ts.registerCallback(gps_sonde_callback)

    # catabot
    ts = message_filters.ApproximateTimeSynchronizer([sonde_sub, gps_sub, airspeed_sub, sonar_sub, gps_velocity_sub, local_velocity_sub], 100, 0.5, allow_headerless=True)
    ts.registerCallback(gps_sonde_callback)

    rospy.spin()


if __name__ == '__main__':
    try:
        gps_sonde_node()
    except rospy.ROSInterruptException:
        pass
