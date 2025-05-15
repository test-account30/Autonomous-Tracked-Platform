from thread import *
import _thread
import micropython
from drive_system import *
vehicle_driver.stop()
from machine import Pin, I2C, Timer
import utime
from navigation import *
import buttons
import math
import encoder
import gc
i=0
print("Driving")
navigation.update_gps()
utime.sleep(1)
navigation.get_gps_fix()



thread = threading()
_thread.start_new_thread(thread.thread_loop, ())
drive_speed = 0.15 #m/s

count5 = 0
# while True:
#     vehicle_driver.tick()
#     count5 += 1
#     if count5 == 100:
#         count5 = 0
#         try:
#             print(navigation.varience())
#         except KeyboardInterrupt:
#             break

stuck_en = 1
while True: #Waypoint Loop
    navigation.calculate_waypoints()
    for i in range(navigation.waypoint_length()): #Start Waypoint loop
        print("Driving to waypoint")
        vehicle_driver.drive(speed = drive_speed, direction = 1, is_angle_corrected = 1, val = i)
        utime.sleep(1)
        counter = 0
        counter1 = 0
        while True:
            counter+=1
            vehicle_driver.tick()
#             if navigation.is_stuck(stuck_en):
#                 stuck_en = 0
#                 print("Stuck!")
#                 vehicle_driver.drive(speed = drive_speed, direction = 0, is_angle_corrected = 0, val = i)
#                 counter1=15
                
            if counter > 50:
                if counter1 == 1:
                    stuck_en = 1
                    vehicle_driver.drive(speed = drive_speed, direction = 1, is_angle_corrected = 1, val = i)
                    counter1=0
                gc.collect()
                #print(navigation.gps.latitude, navigation.gps.longitude)
                
                print(navigation.planar_coordinates[i], navigation.get_pos())
                #print(vehicle_driver.duty_a,vehicle_driver.duty_b)
                #print(vehicle_driver.duty_limit_a,vehicle_driver.duty_limit_b)
                #print(vehicle_driver.corrective_angle)
                #print(navigation.coord, navigation.planar_coordinates[i])
                #print("Precision", navigation.gps.pdop)
                #print("Speeds", encoders.speed())
                #print("Pitch", navigation.get_pitch())
                print("Angles", navigation.target_angle, navigation.get_angle())
                counter = 0
                vehicle_driver.update_angle(i)
                
                if navigation.have_arrived(i): #if at the gps location, next waypoint
                    print("Arrived")
                    vehicle_driver.stop()
                    utime.sleep(1)
                    break
                
                if counter1 > 1:
                    counter1 -= 1
                    
#                 if counter1 == 10:
#                     vehicle_driver.stop()
#                     utime.sleep(0.1)
#                     vehicle_driver.drive(speed = 0.5, direction = 1, is_angle_corrected = 0, val = i)
#                 
                    
                    
                    
                if (buttons.collision1 or buttons.collision2):
                    vehicle_driver.stop()
                    #navigation.collision_update() # save current gps location as collision site
                    #navigation.update_waypoints() # generate new set of waypoints circumventing collision site
                    utime.sleep(0.5)
                    vehicle_driver.drive(speed = drive_speed, direction = 0, is_angle_corrected = 0, val = i)
                    buttons.collision1 = 0
                    buttons.collision2 = 0
                    counter1=10
