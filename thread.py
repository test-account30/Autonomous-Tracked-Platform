from navigation import *
from kalman_filter import *
import math
import utime
from sensor_fusion import *
from drive_system import *

class threading:
    def __init__(self):
        self.update_timer = 0
        sensor_fuse.update()
        self.time_prev = utime.ticks_ms()
        self.gps_update_time = 0
    def thread_loop(self):
        while True:
            time_now = utime.ticks_ms()
            dt = (time_now - self.time_prev)/1000
            self.update_timer += dt
            self.time_prev = time_now
            wheel_speed = vehicle_driver.avg_speed
            heading = math.radians(navigation.get_angle())
            pitch = math.radians(0)
            #pitch = math.radians(navigation.get_pitch())
            ax, ay = (0,0)
            x_pred, P_pred = kf.predict(dt, wheel_speed, heading, -ax, -ay, pitch)
            if navigation.uart.any()>=8:
                navigation.update_gps()
                self.gps_update_time += self.update_timer
                gps_data = navigation.get_gps_coordinate()
                if (navigation.is_gps_new() and navigation.gps_error > 0 and navigation.gps_error < 10):
                    self.gps_update_time = 0

                #print("Update Time:", self.update_timer)
                if self.gps_update_time > 8:
                    self.gps_update_time = 8
                    
                kf.update(x_pred, P_pred, gps_data, self.gps_update_time, navigation.gps_error)
                self.update_timer = 0
            





