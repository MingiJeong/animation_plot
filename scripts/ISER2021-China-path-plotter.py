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
CSV_FILE_PATH2 = ''
CSV_FILE_PATH_done = r'//home/minkbrook/Desktop/China-catabot-20201006.csv'
#global data_read
data_read = True
data_read2 = True
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

WP_time = [573.4196488857269,756.5946922302246,
818.7112998962402,1097.242814540863,1144.8628597259521,
1486.472694158554,1543.6516244411469,1914.4799513816833,1959.658656835556,2293.1281819343567,
2347.8482568264008,2694.837943315506,2736.1956074237823,
3056.5659227371216,3177.8834385871887,3414.856861114502]

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
    global CSV_FILE_PATH2
    CSV_FILE_PATH = r'//home/minkbrook/Desktop/china-chlorophyll.csv'
    CSV_FILE_PATH2 = r'//home/minkbrook/Desktop/china-chlorophyll-oneline.csv'

    data_plot(fig,ax)

    #RATE.sleep()
    #rospy.spin() # spin use because rospy while gave


def data_plot(fig,ax):
    """
    data plotting function
    """
    global data_read
    global data_read2
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
    ax1 = ax[0]
    ax2 = ax[1]
    ax3 = ax[2]
    #print("here2", data_read)
    path_1 = path.exists("/home/minkbrook/Desktop/china-chlorophyll.csv")
    path_2 = path.exists("/home/minkbrook/Desktop/china-chlorophyll-oneline.csv")
    if path_1 and path_2:
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
                ax3 initial stuff here plot
                """
                print("initial time set up", initial_time)
                ax3.set_title('Oct 06 China Lake: Chlorophyll (RFU)', fontsize=20)
                vmax=0.80
                vmin=0.40
                levels = np.linspace(vmin,vmax, 50)

                # contour
                cs = ax3.tricontourf(x_done,y_done,z_done, 5, norm=colors.SymLogNorm(linthresh=0.03, linscale=0.03), levels=levels,vmax=vmax,vmin=vmin)
                cb_ticklabel = np.linspace(vmin, vmax, 5)
                cb = fig.colorbar(cs, ticks=cb_ticklabel, orientation='horizontal', format='%.2f')

                # color bar
                fontprops = fm.FontProperties(size=14)
                bar = AnchoredSizeBar(ax3.transData, 0.00046, '40 m', 'upper right', pad=0.6, frameon=False, fontproperties=fontprops)
                ax3.add_artist(bar)
                cb.ax.tick_params(labelsize=16)
                cb.set_label(label='Chlorophyll (RFU)', size=14)
                cb.ax.xaxis.set_ticks_position('bottom')

                # auxiliary markers
                ax3.plot(-69.6045, 44.44585, '^', color='k', markersize=20)
                ax3.text(-69.60458, 44.44555, 'N', fontsize=20)
                ax3.set_aspect('equal')

        with open(CSV_FILE_PATH, 'r') as csv_file:
            reader = csv.reader(csv_file)
            your_list = np.array(list(reader))
            # initial time setup
            #time = your_list[:, 1].astype('float32')
            #heading = your_list[:,2].astype('float32')
            x = your_list[:,2].astype('float32')
            y = your_list[:,1].astype('float32')
            #z = your_list[:,22].astype('float32')

        # numpy and list thing separate (Both didn't work at the same time)
        with open(CSV_FILE_PATH2, 'r') as csv_file2:

            another_reader = csv.reader(csv_file2)
            for row in another_reader:
                if initial_time is None:
                    initial_time = float(row[0])

                else:
                    current_time = float(row[0])
                    delta_time = float(current_time-initial_time)
                    time.append(delta_time)
                    heading.append(float(row[4]))
                    angular_z.append(float(row[11]))
                    #chlorophyll.append(float(row[22]))

                    """
                    if 1144 <= current_time <= 1481:
                        if chloro_x_1 is not None:
                            chloro_x_1.append(float(row[2]))
                            chloro_y_1.append(float(row[2]))
                        else:
                            chloro_x_1 = []
                            chloro_y_1 = []
                            chloro_x_1.append(float(row[2]))
                            chloro_y_1.append(float(row[2]))

                    if 1959 <= current_time <= 2288:
                        if chloro_x_2 is not None:
                            chloro_x_2.append(float(row[2]))
                            chloro_y_2.append(float(row[2]))
                        else:
                            chloro_x_2 = []
                            chloro_y_2 = []
                            chloro_x_2.append(float(row[2]))
                            chloro_y_2.append(float(row[2]))


                    if 2736 <= current_time <= 3051:
                        if chloro_x_3 is not None:
                            chloro_x_3.append(float(row[2]))
                            chloro_y_3.append(float(row[2]))
                        else:
                            chloro_x_3 = []
                            chloro_y_3 = []
                            chloro_x_3.append(float(row[2]))
                            chloro_y_3.append(float(row[2]))
                    """

        #print("initial time", initial_time)
        #print("your list", your_list)
        #print("time", time)
        #print("heading", heading)

        """
        # NW directions
        NW_1_x = x[806:1045]
        NW_1_y = y[806:1045]
        ax.plot(NW_1_x,NW_1_y,marker='o', color='r', markersize=0.1)

        NW_2_x = x[1384:1620]
        NW_2_y = y[1384:1620]
        ax.plot(NW_2_x,NW_2_y,marker='o', color='r', markersize=0.1)

        NW_3_x = x[1940:2162]
        NW_3_y = y[1940:2162]
        ax.plot(NW_3_x,NW_3_y,marker='o', color='r', markersize=0.1)

        each.axvline(x=wp, color='darkgrey', linestyle='--')
        each.axvspan(1144, 1481, facecolor="r", alpha=0.01)
        each.axvspan(1959, 2288, facecolor='r', alpha=0.01)
        each.axvspan(2736, 3051, facecolor="r", alpha=0.01)
        """

        # WP position
        for index, wp in enumerate(WP_time):
            if delta_time >= wp:
                ax1.axvline(x=wp, color='darkgrey', linestyle='--')
                ax2.axvline(x=wp, color='darkgrey', linestyle='--')


        #ax1.clear() # not that necessary if we fix the color
        ax1.plot(time, heading, color='green')
        ax1.set_ylim([0,360])
        ax1.tick_params(axis='y', labelsize=20)
        ax1.tick_params(labelbottom=False)
        ax1.set_ylabel('Heading (degree)', fontsize=20)
        ax1.yaxis.set_ticks(np.arange(0, 361,100))

        #ax2.clear()
        ax2.plot(time, angular_z, color ='blueviolet')
        ax2.set_ylim([-1.6,1.6])
        ax2.set_ylabel('Yaw velocity (rad/sec)', fontsize=20)
        ax2.set_xlabel('Time (sec)', fontsize=20)
        #ax2.tick_params(axis='both', which='major', labelsize=20)
        ax2.tick_params(axis='x', labelsize=20)
        ax2.tick_params(axis='y', labelsize=20)

        """
        shaded area
        """
        if 1144 <= delta_time <= 1481:
            ax1.axvspan(1144, delta_time, facecolor="r", alpha=0.2)
            ax2.axvspan(1144, delta_time, facecolor="r", alpha=0.2)
            ax3.plot(x[-1], y[-1], marker='o',  color='r', markersize='5')

        if 1959 <= delta_time <= 2288:
            ax1.axvspan(1959, delta_time, facecolor="r", alpha=0.2)
            ax2.axvspan(1959, delta_time, facecolor="r", alpha=0.2)
            ax3.plot(x[-1], y[-1], marker='o',  color='r', markersize='5')

        if 2736 <= delta_time <= 3051:
            ax1.axvspan(2736, delta_time, facecolor="r", alpha=0.2)
            ax2.axvspan(2736, delta_time, facecolor="r", alpha=0.2)
            ax3.plot(x[-1], y[-1], marker='o',  color='r', markersize='5')


        if delta_time >1481:
            ax1.axvspan(1144, 1481, facecolor="r", alpha=0.2)
            ax2.axvspan(1144, 1481, facecolor="r", alpha=0.2)

        if delta_time >2288:
            ax1.axvspan(1959, 2288, facecolor="r", alpha=0.2)
            ax2.axvspan(1959, 2288, facecolor="r", alpha=0.2)

        if delta_time >3051:
            ax1.axvspan(2736, 3051, facecolor="r", alpha=0.2)
            ax2.axvspan(2736, 3051, facecolor="r", alpha=0.2)

        #ax3.clear() # due to N mark and behind background
        ax3.plot(x, y, marker='o',  color='k', markersize='1')


        if 1144 <= delta_time <= 1481:
            ax3.plot(x[-1], y[-1], marker='o',  color='r', markersize='5')

        if 1959 <= delta_time <= 2288:
            ax3.plot(x[-1], y[-1], marker='o',  color='r', markersize='5')

        if 2736 <= delta_time <= 3051:
            ax3.plot(x[-1], y[-1], marker='o',  color='r', markersize='5')

        #ax3.set_ylim([7.565,7.614])
        ax3.set_xlabel('Longitude', fontsize=18)
        ax3.set_xlim([-69.6050, -69.6000])
        ax3.set_xticks(np.arange(-69.6050, -69.6000, 0.0025))
        #f.canvas.draw()
        ax3.set_xticklabels(['-69.6050', '-69.6025', '-69.6000'])

        ax3.set_ylabel('Latitude', fontsize=20)
        ax3.set_ylim([44.4435, 44.4460])
        ax3.set_yticks(np.arange(44.4435, 44.4461, 0.0025))
        #ax3.set_yticks(['44.4435', '44.4440', '44.4460'])
        #f.canvas.draw()
        #ax.set_yticklabels(['44.4435', '44.4440', '44.4445', '44.4450', '44.4455', '44.4460'])
        ax3.set_yticklabels(['44.4435','44.4460'])
        ax3.tick_params(axis='x', labelsize=18)
        ax3.tick_params(axis='y', labelsize=20, pad=35, labelrotation=90)
        #ax3.tick_params(axis='y', )
        #ax3.tick_params(axis='both', which='major', labelsize=20)
        #ax3.tick_params(axis='x', labelsize=20)
        #ax3.tick_params(axis='y', labelsize=20)


        plt.show()
        plt.pause(0.005)

        ax1.clear()
        ax2.clear()



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
            #data_plot(fig,ax)
            spin(fig, ax)

    except rospy.ROSInterruptException:
        pass
