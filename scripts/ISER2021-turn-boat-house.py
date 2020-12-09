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

def spin(fig,ax1):
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



def data_plot(fig,ax):
    """
    data plotting function
    """
    global data_read
    global initial_time
    global time
    global heading
    global angular_z
    #global ODO
    #global your_list
    global row

    #print("Here")
    ax1 = ax[0]
    ax2 = ax[1]
    ax3 = ax[2]
    #print("here2", data_read)
    if data_read:
        print("now data exist")

        # initialization process
        # if it is "a" it should be here; otherwise 'wb', it should be above
        time = []
        heading = []
        angular_z = []
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
                    time.append(float(current_time-initial_time))
                    heading.append(float(row[2]))
                    angular_z.append(float(row[9]))
                    ODO.append(float(row[22]))


        #print("initial time", initial_time)
        #print("your list", your_list)
        #print("time", time)
        #print("heading", heading)

        ax1.clear() # not that necessary if we fix the color
        ax1.plot(time, heading, color='green')
        ax1.set_ylim([0,360])
        ax1.tick_params(axis='y', labelsize=20)
        ax1.tick_params(labelbottom=False)
        ax1.set_ylabel('Heading (degree)', fontsize=20)
        ax1.yaxis.set_ticks(np.arange(0, 361,100))

        ax2.clear()
        ax2.plot(time, angular_z, color ='blueviolet')
        ax2.set_ylim([-1.6,1.6])
        ax2.set_ylabel('Yaw velocity (rad/sec)', fontsize=20)
        ax2.set_xlabel('Time (sec)', fontsize=20)
        #ax2.tick_params(axis='both', which='major', labelsize=20)
        ax2.tick_params(axis='x', labelsize=20)
        ax2.tick_params(axis='y', labelsize=20)

        ax3.clear()
        ax3.set_ylim([7.565,7.614])
        ax3.plot(time, ODO, color='red')
        ax3.set_ylabel('Dissolved Oxygen (mg/L)', fontsize=20)
        ax3.set_xlabel('Time (sec)', fontsize=20)
        ax3.tick_params(axis='both', which='major', labelsize=20)
        ax3.tick_params(axis='x', labelsize=20)
        ax3.tick_params(axis='y', labelsize=20)

        if time[-1] >= 10:
            for each in ax:
                each.axvline(x=10, color='gray', linestyle='--')

        if time[-1] >= 72:
            for each in ax:
                each.axvline(x=72, color='orange', linestyle='--', LineWidth=3)


        plt.show()
        plt.pause(0.00001)



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
