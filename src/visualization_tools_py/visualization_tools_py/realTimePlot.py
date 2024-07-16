#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl

# import rospy
import rclpy
from std_msgs.msg import Float32
from sensor_msgs.msg import PointCloud2
from rclpy.qos import QoSProfile
import time
mpl.rcParams['toolbar'] = 'None'
plt.ion()
import pickle
import os
time_duration = 0
start_time_duration = 0
first_iteration = 'True'

explored_volume = 0;
traveling_distance = 0;
run_time = 0;
max_explored_volume = 0
max_traveling_diatance = 0
max_run_time = 0

time_list1 = np.array([])
time_list2 = np.array([])
time_list3 = np.array([])
run_time_list = np.array([])
explored_volume_list = np.array([])
traveling_distance_list = np.array([])
mission_over_flag = False
trajectory = np.array([])
width_count = 0
def timeDurationCallback(msg):
    global time_duration, start_time_duration, first_iteration
    time_duration = msg.data
    if first_iteration == 'True':
        start_time_duration = time_duration
        first_iteration = 'False'

def runTimeCallback(msg):
    global run_time
    run_time = msg.data

def mission_overCallback(msg):
    global mission_over_flag 
    mission_over_flag = True

def trajectorycallback(msg):
    global trajectory 
    data = msg.data
    # data.insert(0, msg.width)
    trajectory = np.array(data)
    trajectory = np.insert(trajectory, 0, msg.width)
    global width_count
    width_count = msg.width
    # trajectory = np.append(trajectory, points)

def exploredVolumeCallback(msg):
    global explored_volume
    explored_volume = msg.data


def travelingDistanceCallback(msg):
    global traveling_distance
    traveling_distance = msg.data
    # print("msg")

def main():
    global time_duration, start_time_duration, explored_volume, traveling_distance, run_time, max_explored_volume, max_traveling_diatance, max_run_time, time_list1, time_list2, time_list3, run_time_list, explored_volume_list, traveling_distance_list
    rclpy.init()
    node = rclpy.create_node('realTimePlot')
    time_duration_record = 100000
    global width_count
    # rospy.init_node('realTimePlot')
    # qos_profile = QoSProfile(depth=10)
    time_sub = node.create_subscription(Float32, '/time_duration', timeDurationCallback, 10)
    # rospy.Subscriber("/time_duration", Float32, timeDurationCallback)
    runtime_sub = node.create_subscription(Float32, '/runtime', runTimeCallback, 10)
    # rospy.Subscriber("/runtime", Float32, runTimeCallback)
    explored_volume_sub = node.create_subscription(Float32, '/explored_volume', exploredVolumeCallback, 10)
    # rospy.Subscriber("/explored_volume", Float32, exploredVolumeCallback)
    traveling_distance_sub = node.create_subscription(Float32, '/traveling_distance', travelingDistanceCallback, 10)
    
    # rospy.Subscriber("/traveling_distance", Float32, travelingDistanceCallback)
    mission_over_sig_sub = node.create_subscription(Float32, '/mission_over', mission_overCallback, 10)
    trajectory_sub = node.create_subscription(PointCloud2, '/trajectory_color', trajectorycallback, 10)

    fig=plt.figure(figsize=(8,7))
    fig1=fig.add_subplot(311)
    plt.title("Exploration Metrics\n", fontsize=14)
    plt.margins(x=0.001)
    fig1.set_ylabel("Explored\nVolume (m$^3$)", fontsize=12)
    l1, = fig1.plot(time_list2, explored_volume_list, color='r', label='Explored Volume')
    fig2=fig.add_subplot(312)
    fig2.set_ylabel("Traveling\nDistance (m)", fontsize=12)
    l2, = fig2.plot(time_list3, traveling_distance_list, color='r', label='Traveling Distance')
    fig3=fig.add_subplot(313)
    fig3.set_ylabel("Algorithm\nRuntime (s)", fontsize=12)
    fig3.set_xlabel("Time Duration (s)", fontsize=12) #only set once
    l3, = fig3.plot(time_list1, run_time_list, color='r', label='Algorithm Runtime')
    # print("00000")
    count = 0
    # r = rospy.Rate(100) # 100hz
    # r = node.create_rate(100)
    # duration = Duration(seconds=2)
    data_collection_stop_flag = False
    work_done_flag1 = False
    work_done_flag2 = False
    time_limited = 100000
    time_limited_for_de_mission = 300
    traveling_distance_list_for_de_misiion = []
    time_list_for_de_misiion = []
    global mission_over_flag 
    while rclpy.ok():
        # print("zzzz")
        # r.sleep()
        rclpy.spin_once(node)
        time.sleep(0.01)
        count = count + 1
        # print("11111")
        if count % 25 == 0:
            # print("22222")
            max_explored_volume = explored_volume
            max_traveling_diatance = traveling_distance
            if run_time > max_run_time:
                max_run_time = run_time

            time_list2 = np.append(time_list2, time_duration)
            explored_volume_list = np.append(explored_volume_list, explored_volume)
            time_list3 = np.append(time_list3, time_duration)
            traveling_distance_list = np.append(traveling_distance_list, traveling_distance)
            time_list1 = np.append(time_list1, time_duration)
            run_time_list = np.append(run_time_list, run_time)

        if count >= 100:
            count = 0
            l1.set_xdata(time_list2)
            l2.set_xdata(time_list3)
            l3.set_xdata(time_list1)
            l1.set_ydata(explored_volume_list)
            l2.set_ydata(traveling_distance_list)
            l3.set_ydata(run_time_list)

            fig1.set_ylim(0, max_explored_volume + 500)
            fig1.set_xlim(start_time_duration, time_duration + 10)
            fig2.set_ylim(0, max_traveling_diatance + 20)
            fig2.set_xlim(start_time_duration, time_duration + 10)
            fig3.set_ylim(0, max_run_time + 0.2)
            fig3.set_xlim(start_time_duration, time_duration + 10)

            fig.canvas.draw()
  
        # if mission_stop_flag:
        if (not work_done_flag1) and mission_over_flag:
            time_duration_record = time_duration
            print("finished time(s): ", time_duration_record,   "path width: " ,width_count)
            work_done_flag1 = True
            data = {
            "time_list1": time_list1,
            "explored_volume_list": explored_volume_list,
            "traveling_distance_list": traveling_distance_list,
            "run_time_list": run_time_list,
            "trajectory": trajectory
                }
            print(trajectory[0])
            order = 1
            while True:
                filename = "OUR_data_new_garage_v2_{}.pickle".format(order)
                if not os.path.exists(filename):
                    with open(filename, "wb") as f:
                        pickle.dump(data, f)
                    break
                else:
                    order += 1
            
        # if time_duration>time_limited:
        if time_duration>2*time_duration_record:
        
            data_collection_stop_flag = True
            
            data = {
            "time_list1": time_list1,
            "explored_volume_list": explored_volume_list,
            "traveling_distance_list": traveling_distance_list,
            "run_time_list": run_time_list,
            "trajectory": trajectory
                }
            if (not work_done_flag1):
                order = 1
                while True:
                    filename = "OUR_data_new_garage_v2_{}.pickle".format(order)
                    if not os.path.exists(filename):
                        with open(filename, "wb") as f:
                            pickle.dump(data, f)
                        break
                    else:
                        order += 1
            order = 1
            while True:
                filename = "OUR_data_new_garage_v2_longer_{}.pickle".format(order)
                if not os.path.exists(filename):
                    with open(filename, "wb") as f:
                        pickle.dump(data, f)
                    break
                else:
                    order += 1
            work_done_flag1 = True
            work_done_flag2 = True
        if time_duration>(time_limited_for_de_mission+1):
            traveling_distance_list_for_de_misiion.append(traveling_distance)
            time_list_for_de_misiion.append(time_duration)
            if (time_duration-time_list_for_de_misiion[0])>time_limited_for_de_mission:
                if traveling_distance-traveling_distance_list_for_de_misiion[0]<10:
                    mission_over_flag = True
                traveling_distance_list_for_de_misiion.pop(0)
                time_list_for_de_misiion.pop(0)
            



        if data_collection_stop_flag:
            work_done_flag2 = True
        if work_done_flag1 and work_done_flag2:
            print("reach time limited")
            break

if __name__ == '__main__':
  main()
  print("------------job done------------")
