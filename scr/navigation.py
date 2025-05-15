import machine
import utime
from math import sin, cos, sqrt, atan2, tan, atan, asin, radians, degrees
from micropyGPS import *
from i2c_mux_driver import *
from sensor_fusion import *
from kalman_filter import *
from ulab import numpy as np
from micropython import const
import gc


class nav:
    def __init__(self, filename, method):
        self.file_name = filename
    
        
        assert method in ['waypoints', 'area_fill']
        self.method = method
        
        self.waypoints=[]
        
        self.collision_site=[]
        self.gps_error = 0
        
        self.uart = machine.UART(0, rx=1, tx=0, baudrate=9600, bits=8, parity=None, stop=1, timeout=5000)
 
        self.gps = MicropyGPS()
      
        self.start_point_coords = (0,0)

        self.gps_tolerance=const(0.00010)
        
        with open(self.file_name,'r') as file:
            for line in file:
                line=line.rstrip('\n')
                line=line.rstrip('\r')
                self.waypoints.append(line.split(','))
                
        self.planar_coordinates = []
        
        self.gps_long_prev = self.gps.longitude
        self.gps_lat_prev = self.gps.latitude
        
        self.is_calibrated = False
        
    def haversine_distance(self, coord1, coord2):
        # Earth's radius in meters
        earth_radius = 6371000

        # Convert coordinates to radians
        lat1, lon1 = map(radians, coord1)
        lat2, lon2 = map(radians, coord2)

        # Calculate angular differences
        d_lat = lat2 - lat1
        d_lon = lon2 - lon1

        # Calculate central angle
        a = 2 * asin(sqrt(sin(d_lat / 2)**2 + cos(lat1) * cos(lat2) * sin(d_lon / 2)**2))

        # Calculate distance
        distance = earth_radius * a

        return round(distance,2)
      


    def calculate_distance(self, coord1, coord2):
        coord1 = [float(coord) for coord in coord1]
        coord2 = [float(coord) for coord in coord2]
        coord3 = [coord1[0], coord2[1]]
        coord4 = [coord2[0], coord1[1]]
        
        dx = self.haversine_distance(coord1, coord3)
        dy = self.haversine_distance(coord1, coord4)

        # Determine directional indicators
        if coord2[0] < coord1[0]:
            dy *= -1  # If the second latitude is smaller, it's southwards
        if coord2[1] < coord1[1]:
            dx *= -1  # If the second longitude is smaller, it's westwards

        return dx, dy
            
    def calculate_waypoints(self):
        if (self.method == 'waypoints'):
            for ways in self.waypoints:
                self.planar_coordinates.append(self.calculate_distance(self.start_point_coords,ways))
            
            
                    
    def coordinates(self, waypoint_num):
        self.calculate_waypoints()
        return(self.planar_coordinates[waypoint_num])

        
    def waypoint_length(self):
        return len(self.planar_coordinates)
    
        
    def _get_bearing(self, coord1, coord2):
        # Calculate the differences in coordinates
        delta_x = coord2[0] - coord1[0]
        delta_y = coord2[1] - coord1[1]
        
        # Calculate the angle in radians
        angle_rad = atan2(delta_y, delta_x)
        
        # Convert radians to degrees
        angle_deg = degrees(angle_rad)
        
        # Ensure angle is between 0 and 360 degrees
        angle_deg = angle_deg % 360
        
        return angle_deg

    def get_angle(self):
        return sensor_fuse.data()['heading']
    
    def _remove_gravity(self, ax, ay, az, pitch, roll, yaw):
        # Define gravity vector in local coordinates (assuming z-axis is vertical)
        self.gravity_local = np.array([0, 0, -9.81])  # Change -9.8 if needed

        # Convert angles to radians
        self.pitch_rad = np.radians(pitch)
        self.roll_rad = np.radians(roll)
        self.yaw_rad = np.radians(yaw)

        # Compute rotation matrices
        self.rotation_x = np.array([[1, 0, 0],
                                [0, np.cos(self.pitch_rad), -np.sin(self.pitch_rad)],
                                [0, np.sin(self.pitch_rad), np.cos(self.pitch_rad)]])
        
        self.rotation_y = np.array([[np.cos(self.roll_rad), 0, np.sin(self.roll_rad)],
                                [0, 1, 0],
                                [-np.sin(self.roll_rad), 0, np.cos(self.roll_rad)]])

        self.rotation_z = np.array([[np.cos(self.yaw_rad), -np.sin(self.yaw_rad), 0],
                                [np.sin(self.yaw_rad), np.cos(self.yaw_rad), 0],
                                [0, 0, 1]])

        # Combine rotations
        self.rotation_matrix = np.dot(self.rotation_z, np.dot(self.rotation_y, self.rotation_x))

        # Rotate gravity vector to global coordinates
        self.gravity_global = np.dot(self.rotation_matrix.T, self.gravity_local)

        # Subtract rotated gravity from accelerometer readings
        self.external_fx = ax - self.gravity_global[0]
        self.external_fy = ay - self.gravity_global[1]
        self.external_fz = az - self.gravity_global[2]

        return self.external_fx, self.external_fy, self.external_fz
        
    
    def get_accel(self):
        ax, ay, az = sensor_fuse.data()['accel']
        pitch = sensor_fuse.data()['pitch']
        roll = sensor_fuse.data()['roll']
        yaw = sensor_fuse.data()['heading']
        normalised_accel = self._remove_gravity(ax*-9.81, ay*-9.81, az*-9.81, pitch, roll, yaw)
        normal_list = list(normalised_accel)
        return normal_list

            
            
            
    
    def get_pitch(self):
        return sensor_fuse.data()['pitch']

    def get_gps_coordinate(self):
        self.x = float(self.gps.latitude)
        self.y = float(self.gps.longitude)
        self.current_pos = [self.x, self.y]
        
        self.coord = self.calculate_distance(self.start_point_coords, self.current_pos)
        
        return self.coord
    def get_pos(self):
        self.dis = kf.x[0:2]
        return self.dis
        
        
    def turn_angle(self,waypoint_no):
        self.target_angle=self._get_bearing(self.get_pos(), self.planar_coordinates[waypoint_no])
        return self.target_angle

    def have_arrived(self,waypoint_no, tolerence = 1): #in M
        self.coord = self.get_pos()
        self.arrival_condition = math.isclose(self.coord[0], self.planar_coordinates[waypoint_no][0], rel_tol = tolerence) and math.isclose(self.coord[1], self.planar_coordinates[waypoint_no][1], abs_tol = tolerence)
        return self.arrival_condition

    def _is_valid(self,x, y):
        return (x != 0 and y != 0)

        
    def update_gps(self):
        machine.Pin("LED", machine.Pin.OUT).on()
        self.buf = self.uart.readline()
        for self.char in self.buf:
            self.gps.update(chr(self.char))
        if self._is_valid(self.gps.latitude, self.gps.longitude):
            machine.Pin("LED", machine.Pin.OUT).off()
        self.gps_error = self.gps.pdop
        
    def varience(self):
        pos_now = self.coord
        if not (self.pos_prev == None):
            dm = pos_now - self.pos_prev
        self.pos_prev = pos_now
            
        

                        
    def get_gps_fix(self):
        machine.Pin("LED", machine.Pin.OUT).on()
        print("Getting GPS Fix")
        while True:
            if self.uart.any():
                self.buf = self.uart.readline()
                for self.char in self.buf:
                    self.gps.update(chr(self.char))
                if self._is_valid(self.gps.latitude, self.gps.longitude):
                    machine.Pin("LED", machine.Pin.OUT).off()
                    print("Got it!")
                    break
                
        self.start_point_coords = [self.gps.latitude, self.gps.longitude]
        gc.collect()
        #self.uart.write(b'\0xB5\0x62\0x06\0x08\0x06\0x00\0xC8\0x00\0x01\0x00\0x01\0x00\0xDE\0x6A') # change refresh to 5hz.
        #self.uart.write(b'\0xB5\0x62\0x06\0x08\0x06\0x00\0xE8\0x03\0x01\0x00\0x01\0x00\0x01\0x39') # change refresh to 1hz.
        #self.uart.write(b'\0xB5\0x62\0x06\0x00\0x14\0x00\0x01\0x00\0x00\0x00\0xD0\0x08\0x00\0x00\0x00\0xC2\0x01\0x00\0x07\0x00\0x03\0x00\0x00\0x00\0x00\0x00\0xC0\0x7E')
        #self.uart = machine.UART(0, rx=1, tx=0, baudrate=115200, bits=8, parity=None, stop=1, timeout=5000)
        
    def is_gps_new(self):
        condition = not((self.gps_long_prev == self.gps.longitude) and (self.gps_lat_prev == self.gps.latitude))
        self.gps_long_prev = self.gps.longitude
        self.gps_lat_prev = self.gps.latitude
        return condition
                
                
    def collision_update(self, radius = 0.5): # in M 
        self.update_gps()
        self.collision_site.append([self.get_pos()[0], self.get_pos()[1], radius])
    
    def distance_from_line(self, lat1, long1, lat2, long2, lat_p1, long_p1):
        self.dist = ((lat2-lat1)*(long1-long_p1)-(lat1-lat_p1)*(long2-long1))/(math.sqrt((lat2-lat1)**2+(long2-long1)**2))
        return self.dist
    
    def calculate_new_waypoint(self, lat1, long1, lat2, long2, lat_p1, long_p1, clearance_radius = 3): #https://www.desmos.com/calculator/egyrlzalox
        m = -(lat2-lat1)/(long2-long1)
        c = long_p1 + ((lat2-lat1)/(long2-long1)) * lat_p1
        
        dist = ((lat2-lat1)*(long1-long_p1)-(lat1-lat_p1)*(long2-long1))/(math.sqrt((lat2-lat1)**2+(long2-long1)**2))
        
        #Quadratic Formula. Good luck if you want to maintain this :)  
        
        new_lat_1 = (-(-2*c*m+2*lat_p1+2*long_p1*m) + math.sqrt((-2*c*m+2*lat_p1+2*long_p1*m)**2 -4*(-(m**2)-1)*(-(lat_p1**2)-(c**2)+(clearance_radius**2)+(2*long_p1*c)-(long_p1**2))))/(2*((-m**2)-1))
        new_lat_2 = (-(-2*c*m+2*lat_p1+2*long_p1*m) - math.sqrt((-2*c*m+2*lat_p1+2*long_p1*m)**2 -4*(-(m**2)-1)*(-(lat_p1**2)-(c**2)+(clearance_radius**2)+(2*long_p1*c)-(long_p1**2))))/(2*((-m**2)-1))
        
        new_lat = min((new_lat_1 * dist), (new_lat_2 * dist))/dist
        
        new_long = -((lat2-lat1)/(long2-long1))*new_lat + long_p1 + ((lat2-lat1)/(long2-long1))*lat_p1
        
        return ([new_lat,new_long])
    
        
    def update_waypoints(self):
        for collisions in self.collision_site:
            for i in range(self.waypoint_length()-1):
                if (abs(self.distance_from_line(self.planar_coordinates(i)[0], self.planar_coordinates(i)[1], self.planar_coordinates(i+1)[0], self.planar_coordinates(i+1)[1], collisions[0], collisions[1])) > collisions[2]):
                    self.new_coordinates = self.calculate_new_waypoint(coordinates(i)[0], coordinates(i)[1], coordinates(i+1)[0], coordinates(i+1)[1], collisions[0], collisions[1])
                    self.planar_coordinates.insert(i, self.new_coordinates)
                    
                
    def is_voltage_low(self, limit=12.3):
        analog_value = machine.ADC(26)
        reading = (analog_value.read_u16() / 65025) * 16.15
        return reading <= limit  # Low batt voltage


    def distance_2_waypoint(self, waypoint_no, units='mm'):
        assert units in ['mm', 'm', 'km']
        r = {'mm': 6, 'm': 3, 'km': 0}
        
        # Convert latitude and longitude from degrees to radians
        lat1, lon1, lat2, lon2 = map(math.radians, [self.gps.latitude, self.gps.longitude, float(self.coordinates(waypoint_no)[0]), float(self.coordinates(waypoint_no)[1])])  # Fixed map function
        

        dlat = lat2 - lat1
        dlon = lon2 - lon1
        a = sin(dlat / 2)**2 + cos(lat1) * cos(lat2) * sin(dlon / 2)**2
        c = 2 * atan2(sqrt(a), sqrt(1 - a))
        distance = 6371 * self.c  # Earth radius in kilometers (approximate)

        return self.distance * 10**self.r[units]
             

navigation = nav('waypoints.csv', 'waypoints')

                
                
