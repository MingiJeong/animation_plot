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

import os.path
from os import path

# designation of csv file
#global CSV_FILE_PATH
CSV_FILE_PATH = ''
#global data_read
data_read = False
initial_time = None
delta_time = None

your_list = []
row = None

# initialization of compass (original msg some error)
cur_compass = Range()
#path = None


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

    #rospack = rospkg.RosPack()
    #CSV_FILE_PATH = rospack.get_path('gds_tools') + "/data/" + rospy.get_param('~csv_file_name')

    data_plot(fig,ax)
    #rospy.spin()



def data_plot(fig,ax):
    """
    data plotting function
    """
    global data_read
    global initial_time
    global delta_time
    global time
    global heading
    global angular_z, ground_speed
    #global ODO
    #global your_list
    global row
    global path

    ax1 = ax[0]
    ax2 = ax[1]
    ax3 = ax[2]
    ax4 = ax[3]

    path_check = path.exists("/home/minkbrook/Desktop/boat-house-turning.csv")

    if path_check:
        print("now data exist")

        # initialization process
        """
        wb for fast processing if there are too many
        if it is "a" it should be here; otherwise 'wb', it should be above
        """

        time = []
        heading = []
        angular_z = []
        ground_speed = []
        ODO = []

        with open(CSV_FILE_PATH, 'r') as csv_file:
            reader = csv.reader(csv_file)
            your_list = np.array(list(reader))
            # initial time setup
            #time = your_list[:, 1].astype('float32')
            #heading = your_list[:,2].astype('float32')

        # numpy and list thing separate (Both didn't work at the same time)
        with open(CSV_FILE_PATH, 'r') as csv_file2:

            another_reader = csv.reader(csv_file2)
            for row in another_reader:
                if initial_time is None:
                    initial_time = float(row[1])
                    print("initial time set up", initial_time)

                else:
                    current_time = float(row[1])
                    delta_time = float(current_time-initial_time)
                    time.append(delta_time)
                    heading.append(float(row[2]))
                    angular_z.append(float(row[11]))
                    ODO.append(float(row[24]))
                    ground_speed.append(float(row[4]))


        #print("initial time", initial_time)
        #print("your list", your_list)
        #print("time", time)
        #print("heading", heading)

        ax1.clear() # not that necessary if we fix the color
        ax1.plot(time, heading, color='green')
        ax1.set_ylim([0,360])
        ax1.tick_params(axis='y', labelsize=20)
        ax1.tick_params(labelbottom=False)
        ax1.set_ylabel('Heading \n (deg)', fontsize=20)
        ax1.yaxis.set_ticks(np.arange(0, 361,100))

        ax2.clear()
        ax2.plot(time, angular_z, color ='blueviolet')
        ax2.set_ylim([-1.6,1.6])
        ax2.set_ylabel('Yaw velocity \n (rad/s)', fontsize=20)
        #ax2.set_xlabel('Time (sec)', fontsize=20)
        #ax2.tick_params(axis='both', which='major', labelsize=20)
        ax2.tick_params(labelbottom=False)
        ax2.tick_params(axis='x', labelsize=20)
        ax2.tick_params(axis='y', labelsize=20)

        ax3.clear()
        ax3.plot(time, ground_speed, color='blue')
        ax3.set_ylim([0, 1.3])
        ax3.set_ylabel('Ground \n Speed \n (m/s)', fontsize=20)
        ax3.set_xlabel('Time (s)', fontsize=20)
        ax3.tick_params(axis='x', labelsize=20)
        ax3.tick_params(axis='y', labelsize=20)

        ax4.clear()
        ax4.plot(time, ODO, color='red')
        ax4.set_ylabel('Dissolved Oxygen (mg/L)', fontsize=20)
        ax4.set_xlabel('Time (s)', fontsize=20)
        ax4.tick_params(axis='both', which='major', labelsize=20)
        ax4.tick_params(axis='x', labelsize=20)
        ax4.tick_params(axis='y', labelsize=20)
        ax4.set_ylim([8.74, 8.88])
        ax4.set_yticks(np.arange(8.74, 8.88, 0.02))
        #ax4.set_aspect('equal')

        if delta_time >= 0:
            ax1.axvline(x=0, color='gray', linestyle='--', LineWidth=1)
            ax2.axvline(x=0, color='gray', linestyle='--', LineWidth=1)
            ax3.axvline(x=0, color='gray', linestyle='--', LineWidth=1)
            ax4.axvline(x=0, color='gray', linestyle='--', LineWidth=1)
        if delta_time >= 82:
            ax1.axvline(x=82, color='gray', linestyle='--', LineWidth=1)
            ax2.axvline(x=82, color='gray', linestyle='--', LineWidth=1)
            ax3.axvline(x=82, color='gray', linestyle='--', LineWidth=1)
            ax4.axvline(x=82, color='gray', linestyle='--', LineWidth=1)

        if delta_time >= 95:
            ax1.axvline(x=95, color='orange', linestyle='--', LineWidth=3)
            ax2.axvline(x=95, color='orange', linestyle='--', LineWidth=3)
            ax3.axvline(x=95, color='orange', linestyle='--', LineWidth=3)
            ax4.axvline(x=95, color='orange', linestyle='--', LineWidth=3)


        plt.show()
        plt.pause(0.00001)



if __name__ == "__main__":
    try:

        rospy.init_node("real_time_plotter", anonymous=False)
        rospy.sleep(1)
        """
        definition of the figures
        """
        #boundary of plot
        #style.use('fivethirtyeight')
        global fig, ax
        fig = plt.figure()
        ax1 = plt.subplot2grid((3, 2), (0, 0), rowspan=1, colspan=1)
        ax2 = plt.subplot2grid((3, 2), (1, 0), rowspan=1, colspan=1)
        ax3 = plt.subplot2grid((3, 2), (2, 0), rowspan=1, colspan=1)
        ax4 = plt.subplot2grid((3, 2), (0, 1), rowspan=3, colspan=1)
        #ax1 = fig.add_subplot(1,1,1)
        plt.ion()
        mng = plt.get_current_fig_manager()
        mng.full_screen_toggle()
        ax = (ax1, ax2, ax3, ax4)
        """
        main plotter
        """
        while not rospy.is_shutdown():
            #data_plot(fig,ax)
            spin(fig, ax)

    except rospy.ROSInterruptException:
        pass
