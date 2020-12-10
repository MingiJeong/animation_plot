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
import matplotlib.font_manager as fm
import matplotlib.colors as colors
from mpl_toolkits.axes_grid1.anchored_artists import AnchoredSizeBar
from matplotlib import ticker, cm
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
#CSV_FILE_PATH_done = r'//home/minkbrook/Desktop/Sunapee-20200715-path-1.csv'
#CSV_FILE_PATH_done = r'//home/minkbrook/Desktop/Sunapee-20200715-path-2.csv'
CSV_FILE_PATH_done = r'//home/minkbrook/Desktop/Sunapee-20200715-path-3.csv'
#global data_read
initial_time = None
delta_time = None
time = []
heading = []
angular_z = []
chlorophyll = []

your_list = []
row = None

global fig, ax
fig = None
ax = None

background = False
x_done = None
y_done = None
z_done = None
"""
WP_time = [573.4196488857269,756.5946922302246,
818.7112998962402,1097.242814540863,1144.8628597259521,
1486.472694158554,1543.6516244411469,1914.4799513816833,1959.658656835556,2293.1281819343567,
2347.8482568264008,2694.837943315506,2736.1956074237823,
3056.5659227371216,3177.8834385871887,3414.856861114502]
"""
"""
WP_time = [568.3993563652039, 573.4196488857269,751.5762028694153,756.5946922302246,813.6927320957184,
818.7112998962402,1092.2240586280823,1097.242814540863,1139.8434052467346,1144.8628597259521,1481.4532732963562,
1486.472694158554,1538.6305129528046,1543.6516244411469,1909.4609701633453,1914.4799513816833,1954.6403217315674,1959.658656835556,2288.109833240509,2293.1281819343567,
2342.827885389328,2347.8482568264008,2689.8184719085693,2694.837943315506,2731.1767921447754,2736.1956074237823,3051.548160791397,
3056.5659227371216,3172.8644704818726,3177.8834385871887,3414.856861114502]
"""
# initialization of compass (original msg some error)
cur_compass = Range()

chloro_x_1 = None
chloro_y_1 = None

chloro_x_2 = None
chloro_y_2 = None

chloro_x_3 = None
chloro_y_3 = None

def spin(fig,ax):
    """ROS node to listen to robot global position, depth, and EXO sonde measurements and write to csv file."""
    global CSV_FILE_PATH_done
    global CSV_FILE_PATH
    CSV_FILE_PATH = r'//home/minkbrook/Desktop/Sunapee-path.csv'

    data_plot(fig,ax)

    #RATE.sleep()
    #rospy.spin() # spin use because rospy while gave


def data_plot(fig,ax):
    """
    data plotting function
    """
    global background
    global initial_time
    global delta_time
    global time
    global heading
    global angular_z
    global chlorophyll
    #global your_list
    global row
    global x_done
    global y_done
    global z_done
    global WP_time
    global chloro_x, chloro_y

    #print("Here")
    #print("here2", data_read)
    path_1 = path.exists("/home/minkbrook/Desktop/Sunapee-path.csv")
    if path_1:
        print("now data exist")

        # initialization process
        # if it is "a" it should be here; otherwise 'wb', it should be above
        """
        time = []
        heading = []
        angular_z = []
        chlorophyll = []
        """

        if background is False:
            with open(CSV_FILE_PATH_done, 'r') as csv_file_done:
                reader_done = csv.reader(csv_file_done)
                your_list_done = np.array(list(reader_done))
                # initial time setup
                #time = your_list[:, 1].astype('float32')
                #heading = your_list[:,2].astype('float32')
                x_done = your_list_done[:,2].astype('float32')
                y_done = your_list_done[:,1].astype('float32')
                z_done = your_list_done[:,29].astype('float32')
                background = True

                """
                ax initial stuff here plot
                """
                print("initial time set up", initial_time)
                #ax.set_title('Oct 06 China Lake: Chlorophyll (RFU)', fontsize=20)
                vmax=0.20
                vmin=0.02
                levels = np.linspace(vmin,vmax, 50)

                # contour
                cs = ax.tricontourf(x_done,y_done,z_done, 10, norm=colors.SymLogNorm(linthresh=0.03, linscale=0.03), levels=levels,vmax=vmax,vmin=vmin)
                cb_ticklabel = np.linspace(vmin, vmax, 5)
                #cb = fig.colorbar(cs, ticks=cb_ticklabel, orientation='horizontal', format='%.2f')

                # color bar
                fontprops = fm.FontProperties(size=20)
                bar = AnchoredSizeBar(ax.transData, 0.00046, '40 m', 'upper right', pad=0.6, frameon=False, fontproperties=fontprops)
                ax.add_artist(bar)
                #cb.ax.tick_params(labelsize=16)
                #cb.set_label(label='Chlorophyll (RFU)', size=14)
                #cb.ax.xaxis.set_ticks_position('bottom')


                # auxiliary markers
                ax.plot(-72.0375, 43.4119, '^', color='k', markersize=22)
                ax.text(-72.03755, 43.4117, 'N', fontsize=22)

                # summer buoy
                ax.plot(np.array([-72.033128]), np.array([43.4096079]), color='k', marker='o', markersize=18)
                ax.plot(np.array([-72.033128]), np.array([43.4096079]), color='yellow', marker='o', markersize=13)
                # boathouse
                ax.plot(np.array([-72.0369625]), np.array([43.4100466]), color='k', marker=(5,1), markersize=21)
                ax.plot(np.array([-72.0369625]), np.array([43.4100466]), color='#FF4500', marker=(5,1), markersize=13)
                # winter buoy
                ax.plot(np.array([-72.0365116]), np.array([43.410345]), color='k', marker='o', markersize=18)
                ax.plot(np.array([-72.0365116]), np.array([43.410345]), color='m', marker='o', markersize=13)

                ax.set_aspect('equal')

        with open(CSV_FILE_PATH, 'r') as csv_file:
            reader = csv.reader(csv_file)
            your_list = np.array(list(reader))
            # initial time setup
            #time = your_list[:, 1].astype('float32')
            #heading = your_list[:,2].astype('float32')
            x = your_list[:,2].astype('float32')
            y = your_list[:,1].astype('float32')
            #z = your_list[:,22].astype('float32')


        #ax.clear() # due to N mark and behind background
        #ax.plot(x, y, marker='o',  color='k', markersize='1')
        #ax.plot(x, y, marker='o',  color='b', markersize='1')
        ax.plot(x, y, marker='o',  color='r', markersize='1')

        # ============= x-axis =============
        #ax.set_xlabel('Longitude', fontsize=28)
        plt.xlim([-72.0377, -72.0327])
        ax.set_xticks(np.arange(-72.0377, -72.0326, 0.0025))
        #f.canvas.draw()
        ax.set_xticklabels(['-72.0377', '-72.0352', '-72.0327'])

        # ============= y-axis =============
        #ax.set_ylabel('Latitude', fontsize=28)
        plt.ylim([43.4095, 43.4120])
        ax.set_yticks(np.arange(43.4095, 43.4121, 0.0025))
        #f.canvas.draw()
        ax.set_yticklabels(['43.4095', '43.4120'])

        ax.tick_params(axis='x', labelsize=24)
        ax.tick_params(axis='y', labelsize=24, pad=55, labelrotation=90)

        # remove the tick
        frame = plt.gca()
        frame.axes.xaxis.set_ticklabels([])
        frame.axes.yaxis.set_ticklabels([])

        plt.show()
        plt.pause(0.005)



if __name__ == "__main__":
    try:
        rospy.init_node("real_time_plotter", anonymous=False)
        rospy.sleep(1)
        RATE = rospy.Rate(1)
        """
        definition of the figures
        """
        #boundary of plot
        #style.use('fivethirtyeight')
        #global fig, ax
        fig, ax = plt.subplots()
        #fig = plt.figure()
        #ax1 = plt.subplot2grid((2, 2), (0, 0), rowspan=1, colspan=1)
        #ax2 = plt.subplot2grid((2, 2), (1, 0), rowspan=1, colspan=1)
        #ax = plt.subplot2grid((2, 2), (0, 1), rowspan=2, colspan=1)

        #ax1 = fig.add_subplot(1,1,1)
        plt.ion()
        mng = plt.get_current_fig_manager()
        mng.full_screen_toggle()
        #ax = (ax1, ax2, ax)
        """
        main plotter
        """

        while not rospy.is_shutdown():
            #data_plot(fig,ax)
            spin(fig, ax)

    except rospy.ROSInterruptException:
        pass
