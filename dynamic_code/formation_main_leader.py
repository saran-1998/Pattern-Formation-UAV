# This is the main function for leader drone.
# Version 2.1

import itertools
import __builtin__
from formation_function import *
import time
from datetime import datetime
import netifaces as ni
import dronekit
from dronekit import connect
from dronekit import VehicleMode
from dronekit import LocationGlobalRelative
from dronekit import mavutil
import os
import sys
sys.path.append(os.getcwd())

# Get local host IP.
local_host = sys.argv[1]
port = sys.argv[2]
total_no_of_drones = int(sys.argv[3])
routerIPAddress = sys.argv[4]
missionPlannerIPAddress = sys.argv[5]
host_specifier = local_host[-1]

# Set log.
flight_log_bufsize = 1  # 0 means unbuffered, 1 means line buffered.
flight_log_filename = 'FlightLog_iris' + host_specifier + \
    '_' + '{:%Y%m%d_%H-%M-%S}'.format(datetime.now()) + '.txt'
flight_log_path_filename = str('Log/'+flight_log_filename)
flight_log = open(flight_log_path_filename, 'w', flight_log_bufsize)
sys.stdout = flight_log

# Specify whether a leader or a follower.
is_leader = True
if is_leader:
    print('{} - This is a leader drone.'.format(time.ctime()))
    leader_host = local_host
else:
    print('{} - This is a follower drone.'.format(time.ctime()))

print('{} - local_host = {}.'.format(time.ctime(), local_host))
print('{} - This drone is iris{}'.format(time.ctime(), host_specifier))

# Get local host IP.
local_host = sys.argv[1]
print('{} - local_host = {}.'.format(time.ctime(), local_host))
host_specifier = local_host[-1]
print('{} - This drone is iris{}'.format(time.ctime(), host_specifier))

# Reserved port.
# The port number should be exactly the same as that in follower drone.
__builtin__.port_gps = 60191
__builtin__.port_status = 60192
__builtin__.port_immediate_command = 60193
__builtin__.port_heading = 60194

# Connect to the Vehicle
print('{} - Connecting to vehicle...'.format(time.ctime()))
connection_address = local_host + ":" + port
vehicle_temp = connect(connection_address, baud=57600, wait_ready=True)
while not 'vehicle_temp' in locals():
    print('{} - Waiting for vehicle connection...'.format(time.ctime()))
    time.sleep(5)
__builtin__.vehicle = vehicle_temp
print('{} - Vehicle is connected!'.format(time.ctime()))
# Enable safety switch(take effect after reboot pixhawk).
__builtin__.vehicle.parameters['BRD_SAFETYENABLE'] = 1  # Enable
# vehicle.parameters['BRD_SAFETYENABLE'] = 0 # Disable

# Start server services.
start_SERVER_service(is_leader, local_host)

# Start connection checker. Drone will return home once lost connection.
router_host = routerIPAddress
threading.Thread(target=CHECK_network_connection, args=(
    router_host,), kwargs={'wait_time': 10}).start()

# Arm drone without RC.
arm_no_RC()

# IP list:
iris_host = list()
host_prefix = local_host[:10]
drone_address_identifier = int(local_host.split('.')[-1])
currentIPAddress = host_prefix + str(drone_address_identifier)

for i in range(total_no_of_drones):
    iris_host.append(currentIPAddress)
    drone_address_identifier += 1
    currentIPAddress = host_prefix + str(drone_address_identifier) 
    if currentIPAddress == missionPlannerIPAddress:
        drone_address_identifier += 1
        currentIPAddress = host_prefix + str(drone_address_identifier)
    print(drone_address_identifier)

followers = iris_host[1:]
print(followers)
follower_host_tuple = tuple(followers)

# Wait untill all followers are ready(armed).
wait_for_follower_ready(follower_host_tuple)  # This is a blocking call.

# Get GPS coordinate of leader's launch location.
leader_gps_home = __builtin__.vehicle.location.global_relative_frame
leader_lat_home = leader_gps_home.lat
leader_lon_home = leader_gps_home.lon
leader_alt_home = leader_gps_home.alt
print('{} - Home GPS coordinate :'.format(time.ctime()))
print('     leader_lat_home = {}'.format(leader_lat_home))
print('     leader_lon_home = {}'.format(leader_lon_home))
print('     leader_alt_home = {} (relative)'.format(leader_alt_home))

# DOUBLE CHECK the following 4 parameters before each flight mission.
leader_hover_height = 20  # In meter.
leader_fly_distance = 20  # In meters.
# (use current) # In degree, 0~360. 90=East
leader_aim_heading_direction = __builtin__.vehicle.heading

# Fixed parameters.
# fly_follow() parameters for followers.
follower_followee = '\''+leader_host+'\''  # The string must contain ''.
follower_frame_to_followee = '\''+'body'+'\''  # 'body' or 'local'.

# When all members are ready.
# Leader takeoff and hover (in square shape).
threading.Thread(target=takeoff_and_hover, args=(leader_hover_height,)).start()
# Send takeoff command to all followers.
# Immediate command must be in string type.

for i in range(1, no_of_followers + 1):
    print(
        '{} - Sending immediate command to : {}.'.format(time.ctime(), followers[i-1]))
    CLIENT_send_immediate_command(
        followers[i-1], 'takeoff_and_hover({})'.format(follower_hover_height[i-1]))

# Wait for follower ready. Blocking function.
wait_for_follower_ready(follower_host_tuple)

# Get leader current location.
leader_current_gps = __builtin__.vehicle.location.global_relative_frame
leader_current_lat = leader_current_gps.lat
leader_current_lon = leader_current_gps.lon
leader_current_alt = leader_current_gps.alt
print('{} - After taking off and hover, Leader\'s GPS coordinate : lat={}, lon={}, alt_relative={}'.format(
    time.ctime(), leader_current_lat, leader_current_lon, leader_current_alt))
# Get leader current heading.
leader_current_heading = __builtin__.vehicle.heading
print('{} - Leader current heading is {} degree.'.format(time.ctime(),
                                                         leader_current_heading))
#interrupt based handling
#prelude for servo based programming

some_condition_1 = 1        #For square pattern formation
some_condition_2 = 0        #For diamond pattern formation
some_condition_3 = 0        #For triangle pattern formation
some_condition_4 = 0        #For return to home

while(True):
    
    # #input the shape ----temporary code will be changed to include the servo parameters -----
    # #accomodates 4 conditions for square, triangle, diamond, home
    # #once return to home is given the code owould exit and will not accept further inputs
    # #the conditional variable is set as 1 to form that particular pattern

    # print('Please enter the character for the shape to be formed (s-square, d-diamond, t-triangle, r-return to home):')
    # shape = input()
    # if shape == 's':
    #     some_condition_1 = 1
    #     some_condition_2 = 0
    #     some_condition_3 = 0
    #     some_condition_4 = 0
    # elif shape == 'd':
    #     some_condition_1 = 0
    #     some_condition_2 = 1
    #     some_condition_3 = 0
    #     some_condition_4 = 0
    # elif shape == 't':
    #     some_condition_1 = 0
    #     some_condition_2 = 0
    #     some_condition_3 = 1
    #     some_condition_4 = 0
    # elif shape == 'r':
    #     some_condition_1 = 0
    #     some_condition_2 = 0
    #     some_condition_3 = 0
    #     some_condition_4 = 1

    # #Based on the condition variables that are updated above the pattern would be changed

    if __builtin__.vehicle.parameters['RC5_FUNCTION'] == 1:
        # ===================== Formation 1 (square) =====================
        # When taking off, drones are already in this formation.

        no_of_followers = total_no_of_drones-1
        follower_hover_height = [20]*no_of_followers  # In meter
        no_of_drones_per_edge = total_no_of_drones // 4
        remainder_no_of_drones = total_no_of_drones % 4

        edge1_angles = list()
        edge1_displacements = list()
        edge2_angles = list()
        edge2_displacements = list()
        edge3_angles = list()
        edge3_displacements = list()
        edge4_angles = list()
        edge4_displacements = list()

        if remainder_no_of_drones == 0:
            edge_length = 10 * no_of_drones_per_edge
            edge1_displacements = [i*10 for i in range(1, no_of_drones_per_edge+1)]
            edge1_angles = [90] * no_of_drones_per_edge
            edge2_displacements = [math.sqrt(
                (edge_length ** 2) + ((i*10) ** 2)) for i in range(1, no_of_drones_per_edge+1)]
            edge2_angles = [90 + int(math.degrees(math.atan2((i*10), edge_length)))
                            for i in range(1, no_of_drones_per_edge+1)]
            edge3_displacements = [math.sqrt((edge_length ** 2)+(((no_of_drones_per_edge-i)*10) ** 2))
                                for i in range(1, no_of_drones_per_edge+1)]
            edge3_angles = [90 + int(math.degrees(math.atan(edge_length/((no_of_drones_per_edge-i)*10))))
                            for i in range(1, no_of_drones_per_edge)]
            edge3_angles.append(180)
            edge4_displacements = [(no_of_drones_per_edge-i) *
                                10 for i in range(1, no_of_drones_per_edge)]
            edge4_angles = [180] * (no_of_drones_per_edge-1)
        elif remainder_no_of_drones == 1:
            edge_length = 10 * (no_of_drones_per_edge + 1)
            edge1_displacements = [i * (edge_length/no_of_drones_per_edge)
                                for i in range(1, no_of_drones_per_edge+1)]
            edge1_angles = [90] * no_of_drones_per_edge
            edge2_displacements = [math.sqrt((edge_length ** 2) + ((i*10) ** 2))
                                for i in range(1, no_of_drones_per_edge+2)]
            edge2_angles = [90 + int(math.degrees(math.atan2((i*10), edge_length)))
                            for i in range(1, no_of_drones_per_edge+2)]
            edge3_displacements = [math.sqrt((edge_length ** 2)+(((no_of_drones_per_edge-i)*(
                edge_length/no_of_drones_per_edge)) ** 2)) for i in range(1, no_of_drones_per_edge+1)]
            edge3_angles = [90 + int(math.degrees(math.atan(no_of_drones_per_edge/((no_of_drones_per_edge-i)))))
                            for i in range(1, no_of_drones_per_edge)]
            edge3_angles.append(180)
            edge4_displacements = [(no_of_drones_per_edge-i)*(edge_length/no_of_drones_per_edge)
                                for i in range(1, no_of_drones_per_edge)]
            edge4_angles = [180] * (no_of_drones_per_edge-1)
        elif remainder_no_of_drones == 2:
            edge_length = 10 * (no_of_drones_per_edge + 1)
            edge1_displacements = [i * (edge_length/no_of_drones_per_edge)
                                for i in range(1, no_of_drones_per_edge+1)]
            edge1_angles = [90] * no_of_drones_per_edge
            edge2_displacements = [math.sqrt((edge_length ** 2) + ((i*10) ** 2))
                                for i in range(1, no_of_drones_per_edge+2)]
            edge2_angles = [90 + int(math.degrees(math.atan2((i*10), edge_length)))
                            for i in range(1, no_of_drones_per_edge+2)]
            edge3_displacements = [math.sqrt((edge_length ** 2)+(((no_of_drones_per_edge-i)*(
                edge_length/no_of_drones_per_edge)) ** 2)) for i in range(1, no_of_drones_per_edge+1)]
            edge3_angles = [90 + int(math.degrees(math.atan(no_of_drones_per_edge/((no_of_drones_per_edge-i)))))
                            for i in range(1, no_of_drones_per_edge)]
            edge3_angles.append(180)
            edge4_displacements = [
                (no_of_drones_per_edge-i+1)*10 for i in range(1, no_of_drones_per_edge+1)]
            edge4_angles = [180] * (no_of_drones_per_edge)
        else:
            edge_length = 10 * (no_of_drones_per_edge + 1)
            edge1_displacements = [i * (edge_length/no_of_drones_per_edge)
                                for i in range(1, no_of_drones_per_edge+1)]
            edge1_angles = [90] * no_of_drones_per_edge
            edge2_displacements = [math.sqrt((edge_length ** 2) + ((i*10) ** 2))
                                for i in range(1, no_of_drones_per_edge+2)]
            edge2_angles = [90 + int(math.degrees(math.atan2((i*10), edge_length)))
                            for i in range(1, no_of_drones_per_edge+2)]
            edge3_displacements = [math.sqrt((edge_length ** 2)+(((no_of_drones_per_edge-i+1)*10) ** 2))
                                for i in range(1, no_of_drones_per_edge+2)]
            edge3_angles = [90 + int(math.degrees(math.atan(edge_length/((no_of_drones_per_edge-i+1)*10))))
                            for i in range(1, no_of_drones_per_edge+1)]
            edge3_angles.append(180)
            edge4_displacements = [
                (no_of_drones_per_edge-i+1)*10 for i in range(1, no_of_drones_per_edge+1)]
            edge4_angles = [180] * (no_of_drones_per_edge)

        followers_distance_to_followee = list(itertools.chain(
            edge1_displacements, edge2_displacements, edge3_displacements, edge4_displacements))  # In meter
        followers_azimuth_to_followee = list(itertools.chain(
            edge1_angles, edge2_angles, edge3_angles, edge4_angles))  # In meter


        print("Length of followers: ", len(followers))
        print("Length of followers height: ", len(follower_hover_height))
        print("Length of followers distance: ", len(followers_distance_to_followee))
        print("Length of followers azimuth: ", len(followers_azimuth_to_followee))

        # Generate a point, leader will fly to this point.
        # 0=Forward, 90=Right, 180=Backward, 270=Left.
        pointA = new_gps_coord_after_offset_inBodyFrame(
            (leader_current_lat, leader_current_lon), leader_fly_distance, leader_current_heading, 0)
        print('{} - Leader is going to pointA : {}'.format(time.ctime(), pointA))

        # Leader go to new location. Followers fly follow in square shape.
        threading.Thread(target=goto_gps_location_relative, args=(
            pointA[0], pointA[1], leader_hover_height,), kwargs={'groundspeed': 1}).start()
        # When leader is not at destination location, keep sending follow fly command to followers.
        # You can use threading to reduce the delay.
        # Function prototype : fly_follow(followee_host, frame, height, radius_2D, azimuth)
        while ((distance_between_two_gps_coord((__builtin__.vehicle.location.global_relative_frame.lat, __builtin__.vehicle.location.global_relative_frame.lon), (pointA[0], pointA[1])) > 1.0) or (abs(__builtin__.vehicle.location.global_relative_frame.alt - leader_hover_height) > 0.3)):
            for i in range(1, no_of_followers + 1):
                print('{} - Sending command fly_follow() to follower {} with angle {} and distance {}.'.format(
                    time.ctime(), i, followers_azimuth_to_followee[i-1], followers_distance_to_followee[i-1]))
                CLIENT_send_immediate_command(followers[i-1], 'fly_follow({}, {}, {}, {}, {})'.format(follower_followee, follower_frame_to_followee,
                                                                                                    follower_hover_height[i-1], followers_distance_to_followee[i-1], followers_azimuth_to_followee[i-1]))
            time.sleep(0.5)

        # When leader has reached destination, execute air_break().
        # At the same time, send air_break command to all followers immediately.
        threading.Thread(target=air_break, args=()).start()
        for iter_follower in follower_host_tuple:
            print(iter_follower)
            CLIENT_send_immediate_command(iter_follower, 'air_break()')
    elif __builtin__.vehicle.parameters['RC6_FUNCTION'] == 1:
        # ===================== Formation 2 (Diamond) =====================
        time.sleep(3)
        # Shape 2 definition(Diamond).
        follower_hover_height = [20]*no_of_followers  # In meter
        no_of_drones_per_edge = total_no_of_drones // 4
        remainder_no_of_drones = total_no_of_drones % 4

        edge1_angles = list()
        edge1_displacements = list()
        edge2_angles = list()
        edge2_displacements = list()
        edge3_angles = list()
        edge3_displacements = list()
        edge4_angles = list()
        edge4_displacements = list()

        if remainder_no_of_drones == 0:
            edge_length = 10 * no_of_drones_per_edge
            edge1_displacements = [i*10 for i in range(1, no_of_drones_per_edge+1)]
            edge1_angles = [135] * no_of_drones_per_edge
            edge2_displacements = [math.sqrt(
                (edge_length ** 2) + ((i*10) ** 2)) for i in range(1, no_of_drones_per_edge+1)]
            edge2_angles = [135 + int(math.degrees(math.atan2((i*10), edge_length)))
                            for i in range(1, no_of_drones_per_edge+1)]
            edge3_displacements = [math.sqrt((edge_length ** 2)+(((no_of_drones_per_edge-i)*10) ** 2))
                                for i in range(1, no_of_drones_per_edge+1)]
            edge3_angles = [225 - int(math.degrees(math.atan2(((no_of_drones_per_edge-i)*10), edge_length)))
                            for i in range(1, no_of_drones_per_edge+1)]
            edge4_displacements = [(no_of_drones_per_edge-i) *
                                10 for i in range(1, no_of_drones_per_edge)]
            edge4_angles = [225] * (no_of_drones_per_edge-1)
        elif remainder_no_of_drones == 1:
            edge_length = 10 * (no_of_drones_per_edge + 1)
            edge1_displacements = [i * (edge_length/no_of_drones_per_edge)
                                for i in range(1, no_of_drones_per_edge+1)]
            edge1_angles = [135] * no_of_drones_per_edge
            edge2_displacements = [math.sqrt((edge_length ** 2) + ((i*10) ** 2))
                                for i in range(1, no_of_drones_per_edge+2)]
            edge2_angles = [135 + int(math.degrees(math.atan2((i*10), edge_length)))
                            for i in range(1, no_of_drones_per_edge+2)]
            edge3_displacements = [math.sqrt((edge_length ** 2)+(((no_of_drones_per_edge-i)*(
                edge_length/no_of_drones_per_edge)) ** 2)) for i in range(1, no_of_drones_per_edge+1)]
            edge3_angles = [225 - int(math.degrees(math.atan(((no_of_drones_per_edge-i)/no_of_drones_per_edge))))
                            for i in range(1, no_of_drones_per_edge+1)]
            edge4_displacements = [(no_of_drones_per_edge-i)*(edge_length/no_of_drones_per_edge)
                                for i in range(1, no_of_drones_per_edge)]
            edge4_angles = [225] * (no_of_drones_per_edge-1)
        elif remainder_no_of_drones == 2:
            edge_length = 10 * (no_of_drones_per_edge + 1)
            edge1_displacements = [i * (edge_length/no_of_drones_per_edge)
                                for i in range(1, no_of_drones_per_edge+1)]
            edge1_angles = [135] * no_of_drones_per_edge
            edge2_displacements = [math.sqrt((edge_length ** 2) + ((i*10) ** 2))
                                for i in range(1, no_of_drones_per_edge+2)]
            edge2_angles = [135 + int(math.degrees(math.atan2((i*10), edge_length)))
                            for i in range(1, no_of_drones_per_edge+2)]
            edge3_displacements = [math.sqrt((edge_length ** 2)+(((no_of_drones_per_edge-i+1)*10) ** 2))
                                for i in range(1, no_of_drones_per_edge+2)]
            edge3_angles = [225 - int(math.degrees(math.atan2(((no_of_drones_per_edge-i+1)*10), edge_length)))
                            for i in range(1, no_of_drones_per_edge+2)]
            edge4_displacements = [(no_of_drones_per_edge-i)*(edge_length/no_of_drones_per_edge)
                                for i in range(1, no_of_drones_per_edge)]
            edge4_angles = [225] * (no_of_drones_per_edge-1)
        else:
            edge_length = 10 * (no_of_drones_per_edge + 1)
            edge1_displacements = [i * (edge_length/no_of_drones_per_edge)
                                for i in range(1, no_of_drones_per_edge+1)]
            edge1_angles = [135] * no_of_drones_per_edge
            edge2_displacements = [math.sqrt((edge_length ** 2) + ((i*10) ** 2))
                                for i in range(1, no_of_drones_per_edge+2)]
            edge2_angles = [135 + int(math.degrees(math.atan2((i*10), edge_length)))
                            for i in range(1, no_of_drones_per_edge+2)]
            edge3_displacements = [math.sqrt((edge_length ** 2)+(((no_of_drones_per_edge-i+1)*10) ** 2))
                                for i in range(1, no_of_drones_per_edge+2)]
            edge3_angles = [225 - int(math.degrees(math.atan2(((no_of_drones_per_edge-i+1)*10), edge_length)))
                            for i in range(1, no_of_drones_per_edge+2)]
            edge4_displacements = [
                (no_of_drones_per_edge-i+1)*10 for i in range(1, no_of_drones_per_edge+1)]
            edge4_angles = [225] * (no_of_drones_per_edge)


        followers_distance_to_followee = list(itertools.chain(
            edge1_displacements, edge2_displacements, edge3_displacements, edge4_displacements))  # In meter
        followers_azimuth_to_followee = list(itertools.chain(
            edge1_angles, edge2_angles, edge3_angles, edge4_angles))  # In meter


        # Change formation.
        for i in range(no_of_followers, 0, -1):
            print('{} - Sending command fly_follow() to follower {} with angle {} and distance {}.'.format(
                time.ctime(), i, followers_azimuth_to_followee[i-1], followers_distance_to_followee[i-1]))
            CLIENT_send_immediate_command(followers[i-1], 'fly_follow({}, {}, {}, {}, {})'.format(
                follower_followee, follower_frame_to_followee, follower_hover_height[i-1], followers_distance_to_followee[i-1], followers_azimuth_to_followee[i-1]))
            time.sleep(5)  # Give drone 5 seconds to get to its position.

        # Get leader current location.
        leader_current_gps = __builtin__.vehicle.location.global_relative_frame
        leader_current_lat = leader_current_gps.lat
        leader_current_lon = leader_current_gps.lon
        leader_current_alt = leader_current_gps.alt
        print('{} - In formation 2 (diamond), leader\'s GPS coordinate : lat={}, lon={}, alt_relative={}'.format(
            time.ctime(), leader_current_lat, leader_current_lon, leader_current_alt))
        # Get leader current heading.
        leader_current_heading = __builtin__.vehicle.heading
        print('{} - Leader current heading is {} degree.'.format(time.ctime(),
                                                                leader_current_heading))

        # Generate a point, leader will fly to this point.
        # 0=Forward, 90=Right, 180=Backward, 270=Left.
        pointA = new_gps_coord_after_offset_inBodyFrame(
            (leader_current_lat, leader_current_lon), leader_fly_distance, leader_current_heading, 0)
        print('{} - Leader is going to pointA : {}'.format(time.ctime(), pointA))

        # Leader go to new location.
        threading.Thread(target=goto_gps_location_relative, args=(
            pointA[0], pointA[1], leader_hover_height,), kwargs={'groundspeed': 1}).start()
        # When leader is not at destination location, keep sending follow fly command to followers.
        # You can use threading to reduce the delay.
        # Function prototype : fly_follow(followee_host, frame, height, radius_2D, azimuth)
        while ((distance_between_two_gps_coord((__builtin__.vehicle.location.global_relative_frame.lat, __builtin__.vehicle.location.global_relative_frame.lon), (pointA[0], pointA[1])) > 1.5) or (abs(__builtin__.vehicle.location.global_relative_frame.alt - leader_hover_height) > 0.3)):
            for i in range(1, no_of_followers+1):
                print('{} - Sending command fly_follow() to follower {} with angle {} and distance {}.'.format(
                    time.ctime(), i, followers_azimuth_to_followee[i-1], followers_distance_to_followee[i-1]))
                CLIENT_send_immediate_command(followers[i-1], 'fly_follow({}, {}, {}, {}, {})'.format(follower_followee, follower_frame_to_followee,
                                                                                                    follower_hover_height[i-1], followers_distance_to_followee[i-1], followers_azimuth_to_followee[i-1]))
            time.sleep(0.5)

        # When leader has reached destination, execute air_break().
        # At the same time, send air_break command to all followers immediately.
        threading.Thread(target=air_break, args=()).start()
        for iter_follower in follower_host_tuple:
            CLIENT_send_immediate_command(iter_follower, 'air_break()')
    elif __builtin__.vehicle.parameters['RC7_FUNCTION'] == 1:
        # ===================== Formation 3 (triangle) =====================
        time.sleep(3)
        # Shape 3 (triangle).
        follower_hover_height = [20]*no_of_followers  # In meter
        no_of_drones_per_edge = total_no_of_drones // 4
        remainder_no_of_drones = total_no_of_drones % 4

        edge1_angles = list()
        edge1_displacements = list()
        edge2_angles = list()
        edge2_displacements = list()
        edge3_angles = list()
        edge3_displacements = list()
        edge4_angles = list()
        edge4_displacements = list()

        if remainder_no_of_drones == 0:
            edge_length = 10 * no_of_drones_per_edge
            edge1_displacements = [
                i*10*1.414 for i in range(1, no_of_drones_per_edge+1)]
            edge1_angles = [135] * no_of_drones_per_edge
            edge2_displacements = [math.sqrt((edge_length ** 2)+(((no_of_drones_per_edge-i)*10) ** 2))
                                for i in range(1, no_of_drones_per_edge+1)]
            edge2_angles = [180 - int(math.degrees(math.atan2((no_of_drones_per_edge-i), no_of_drones_per_edge)))
                            for i in range(1, no_of_drones_per_edge+1)]
            edge3_displacements = [math.sqrt(
                (edge_length ** 2) + ((i*10) ** 2)) for i in range(1, no_of_drones_per_edge+1)]
            edge3_angles = [180 + int(math.degrees(math.atan2(i, no_of_drones_per_edge)))
                            for i in range(1, no_of_drones_per_edge+1)]
            edge4_displacements = [(no_of_drones_per_edge-i) *
                                10 * 1.414 for i in range(1, no_of_drones_per_edge)]
            edge4_angles = [225] * (no_of_drones_per_edge-1)
        elif remainder_no_of_drones == 1:
            edge_length = 10 * (no_of_drones_per_edge + 1)
            edge1_displacements = [i * 1.414 * (edge_length/no_of_drones_per_edge)
                                for i in range(1, no_of_drones_per_edge+1)]
            edge1_angles = [135] * no_of_drones_per_edge
            edge2_displacements = [math.sqrt((edge_length ** 2) + (((no_of_drones_per_edge-i+1)*10) ** 2))
                                for i in range(1, no_of_drones_per_edge+2)]
            edge2_angles = [180 - int(math.degrees(math.atan2((no_of_drones_per_edge-i+1), (no_of_drones_per_edge+1))))
                            for i in range(1, no_of_drones_per_edge+2)]
            edge3_displacements = [math.sqrt((edge_length ** 2)+((i*(
                edge_length/no_of_drones_per_edge)) ** 2)) for i in range(1, no_of_drones_per_edge+1)]
            edge3_angles = [180 + int(math.degrees(math.atan2(i, no_of_drones_per_edge)))
                            for i in range(1, no_of_drones_per_edge+1)]
            edge4_displacements = [(no_of_drones_per_edge-i)*1.414*(edge_length/no_of_drones_per_edge)
                                for i in range(1, no_of_drones_per_edge)]
            edge4_angles = [225] * (no_of_drones_per_edge-1)
        elif remainder_no_of_drones == 2:
            edge_length = 10 * (no_of_drones_per_edge + 1)
            edge1_displacements = [i * 1.414 * (edge_length/no_of_drones_per_edge)
                                for i in range(1, no_of_drones_per_edge+1)]
            edge1_angles = [135] * no_of_drones_per_edge
            edge2_displacements = [math.sqrt((edge_length ** 2) + (((no_of_drones_per_edge-i+1)*10) ** 2))
                                for i in range(1, no_of_drones_per_edge+2)]
            edge2_angles = [180 - int(math.degrees(math.atan2((no_of_drones_per_edge-i+1), (no_of_drones_per_edge+1))))
                            for i in range(1, no_of_drones_per_edge+2)]
            edge3_displacements = [math.sqrt(
                (edge_length ** 2)+((i*10) ** 2)) for i in range(1, no_of_drones_per_edge+2)]
            edge3_angles = [180 + int(math.degrees(math.atan2(i, (no_of_drones_per_edge+1))))
                            for i in range(1, no_of_drones_per_edge+2)]
            edge4_displacements = [(no_of_drones_per_edge-i)*1.414*(edge_length/no_of_drones_per_edge)
                                for i in range(1, no_of_drones_per_edge)]
            edge4_angles = [225] * (no_of_drones_per_edge-1)
        else:
            edge_length = 10 * (no_of_drones_per_edge + 1)
            edge1_displacements = [i * 1.414 * (edge_length/no_of_drones_per_edge)
                                for i in range(1, no_of_drones_per_edge+1)]
            edge1_angles = [135] * no_of_drones_per_edge
            edge2_displacements = [math.sqrt((edge_length ** 2) + (((no_of_drones_per_edge-i+1)*10) ** 2))
                                for i in range(1, no_of_drones_per_edge+2)]
            edge2_angles = [180 - int(math.degrees(math.atan2((no_of_drones_per_edge-i+1), (no_of_drones_per_edge+1))))
                            for i in range(1, no_of_drones_per_edge+2)]
            edge3_displacements = [math.sqrt(
                (edge_length ** 2)+((i*10) ** 2)) for i in range(1, no_of_drones_per_edge+2)]
            edge3_angles = [180 + int(math.degrees(math.atan2(i, (no_of_drones_per_edge+1))))
                            for i in range(1, no_of_drones_per_edge+2)]
            edge4_displacements = [(no_of_drones_per_edge-i+1)*1.414*10
                                for i in range(1, no_of_drones_per_edge+1)]
            edge4_angles = [225] * (no_of_drones_per_edge)


        followers_distance_to_followee = list(itertools.chain(
            edge1_displacements, edge2_displacements, edge3_displacements, edge4_displacements))  # In meter
        followers_azimuth_to_followee = list(itertools.chain(
            edge1_angles, edge2_angles, edge3_angles, edge4_angles))  # In meter

        # move followers.

        for i in range(1, no_of_followers+1):
            print('{} - Sending command fly_follow() to follower {} with angle {} and distance {}.'.format(
                time.ctime(), i, followers_azimuth_to_followee[i-1], followers_distance_to_followee[i-1]))
            CLIENT_send_immediate_command(followers[i-1], 'fly_follow({}, {}, {}, {}, {})'.format(follower_followee, follower_frame_to_followee,
                                                                                                follower_hover_height[i-1], followers_distance_to_followee[i-1], followers_azimuth_to_followee[i-1]))
            time.sleep(5)  # Give drone 5 seconds to get to its position.

        # Get leader current location.
        leader_current_gps = __builtin__.vehicle.location.global_relative_frame
        leader_current_lat = leader_current_gps.lat
        leader_current_lon = leader_current_gps.lon
        leader_current_alt = leader_current_gps.alt
        print('{} - In formation 3 (triangle), leader\'s GPS coordinate : lat={}, lon={}, alt_relative={}'.format(
            time.ctime(), leader_current_lat, leader_current_lon, leader_current_alt))
        # Get leader current heading.
        leader_current_heading = __builtin__.vehicle.heading
        print('{} - Leader current heading is {} degree.'.format(time.ctime(),
                                                                leader_current_heading))

        # Generate a point, leader will fly to this point.
        # 0=Forward, 90=Right, 180=Backward, 270=Left.
        pointA = new_gps_coord_after_offset_inBodyFrame(
            (leader_current_lat, leader_current_lon), leader_fly_distance, leader_current_heading, 0)
        print('{} - Leader is going to pointA : {}'.format(time.ctime(), pointA))

        # Leader go to new location.
        threading.Thread(target=goto_gps_location_relative, args=(
            pointA[0], pointA[1], leader_hover_height,), kwargs={'groundspeed': 1}).start()
        # When leader is not at destination location, keep sending follow fly command to followers.
        # You can use threading to reduce the delay.
        # Function prototype : fly_follow(followee_host, frame, height, radius_2D, azimuth)
        while ((distance_between_two_gps_coord((__builtin__.vehicle.location.global_relative_frame.lat, __builtin__.vehicle.location.global_relative_frame.lon), (pointA[0], pointA[1])) > 1.5) or (abs(__builtin__.vehicle.location.global_relative_frame.alt - leader_hover_height) > 0.3)):
            for i in range(1, no_of_followers+1):
                print('{} - Sending command fly_follow() to follower {} with angle {} and distance {}.'.format(
                    time.ctime(), i, followers_azimuth_to_followee[i-1], followers_distance_to_followee[i-1]))
                CLIENT_send_immediate_command(followers[i-1], 'fly_follow({}, {}, {}, {}, {})'.format(follower_followee, follower_frame_to_followee,
                                                                                                    follower_hover_height[i-1], followers_distance_to_followee[i-1], followers_azimuth_to_followee[i-1]))
            time.sleep(0.5)

        # When leader has reached destination, execute air_break().
        # At the same time, send air_break command to all followers immediately.
        threading.Thread(target=air_break, args=()).start()
        for iter_follower in follower_host_tuple:
            CLIENT_send_immediate_command(iter_follower, 'air_break()')
    elif __builtin__.vehicle.parameters['RC8_FUNCTION'] == 1:
        # ===================== Mission completed, leader and followers go home =====================
        # Wait for follower ready.
        wait_for_follower_ready(follower_host_tuple)
        print('{} - Mission completed. Return home.'.format(time.ctime()))

        # Followers go home.
        for i in range(1, no_of_followers+1):
            print('{} - Command follower {} return home.'.format(time.ctime(), i))
            CLIENT_send_immediate_command(followers[i-1], 'return_to_launch()')
            time.sleep(2)

        # Leader drone go home.
        print('{} - Followers have returned home, Leader is returning...'.format(time.ctime()))
        return_to_launch()
        print('{} - Leader has returned home.'.format(time.ctime()))
        break