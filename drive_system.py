import time
from motor_driver import *
from machine import Pin, Timer
import math
from navigation import *
from encoder import *
from sensor_fusion import *


m2 = motor(pwm=8, dir1=6, dir2=2)
m1 = motor(pwm=7, dir1=9, dir2=3)


# Initialize PID controllers
m1_pid = PIDController(Kp=500000, Ki=10, Kd=10000)
m2_pid = PIDController(Kp=500000, Ki=10, Kd=10000)
angle_correction = PIDController(Kp=0.01, Ki=0.0001, Kd=0.001)
current_limit_A = PIDController(Kp=1000, Ki=10, Kd=20)
current_limit_B = PIDController(Kp=1000, Ki=10, Kd=20)


class DriveSystem:
    def __init__(self):
        # Initialize Variables
        self.duty_a = 0
        self.duty_b = 0
        self.current_angle = navigation.get_angle()
        self.target_speed_a = 0
        self.target_speed_b = 0
        
        self.is_driving = 0
        self.motor_dir = 0
        self.target_angle = 0
        self.angle_corrected = 0
        self.waypoint_value = 0
        self.target_speed = 0
        
        self.angle_tolerance = 0.01
         
        
        self.current_limit = 6.5 #Current limit for motor driver
        self.max_duty_limit = 50000
        
        self.duty_limit_a = self.max_duty_limit
        self.duty_limit_b = self.max_duty_limit
        
        self.current_sensor = machine.ADC(27)
        
        self.I_a = 0
        self.I_b = 0
        
        self.avg_speed = 0
        
    def update_angle(self, waypoint_value):
        # Update target angle based on waypoint
        self.target_angle_waypoint = navigation.turn_angle(waypoint_value)
    
    def compute_current(self): # moving calculation out of interrupt.
        pass
        
    def tick(self):  # Timer Interrupt
        
        sensor_fuse.update()
    
        encoders.Update()
        
        if not self.is_driving:
            m1.stop()
            m2.stop()
        
        if self.is_driving:
            
            voltage = (self.current_sensor.read_u16()/65535)*5
            current = abs(voltage*15.1515-37.8787) #calculate current from sensor
            self.I_a = current*(self.duty_a/(self.duty_a+self.duty_b+1))
            self.I_b = current*(self.duty_b/(self.duty_a+self.duty_b+1))
            
            self.duty_limit_a = int(current_limit_A.compute(self.I_a, self.current_limit, low=0, up=self.max_duty_limit, sum_max=5000))
            self.duty_limit_b = int(current_limit_B.compute(self.I_b, self.current_limit, low=0, up=self.max_duty_limit, sum_max=5000))
            self.duty_a = int(m1_pid.compute(encoders.speed()[0], abs(self.target_speed_a), low=0, up=self.duty_limit_a, sum_max=2000))
            self.duty_b = int(m2_pid.compute(encoders.speed()[1], abs(self.target_speed_b), low=0, up=self.duty_limit_b, sum_max=2000))
            
            
            current_angle = float(navigation.get_angle())
            
            
            if self.motor_dir:  # State Machine
                self.target_speed_a = self.target_speed_b = self.target_speed
                
                if self.angle_corrected:
                    self.corrective_angle = angle_correction.compute(current_angle, self.target_angle_waypoint, low=-self.target_speed*2, up=self.target_speed*2, sum_max = 360)
                    if self.corrective_angle > 0:
                        self.target_speed_b = abs(self.target_speed_b - abs(self.corrective_angle))
                        if (self.target_speed - abs(self.corrective_angle)) > 0:
                            m1.forward(self.duty_a)
                            m2.forward(self.duty_b)
                            self.avg_speed = (encoders.speed()[1])
                        else:
                            m1.forward(self.duty_a)
                            m2.reverse(self.duty_b)
                            self.avg_speed = (0)
                    else:
                        self.target_speed_a = abs(self.target_speed_a - abs(self.corrective_angle))
                        if (self.target_speed - abs(self.corrective_angle)) > 0:
                            m1.forward(self.duty_a)
                            m2.forward(self.duty_b)
                            self.avg_speed = (encoders.speed()[0])
                        else:
                            m2.forward(self.duty_b)
                            m1.reverse(self.duty_a)
                            self.avg_speed = (0)
                else:
                    m1.forward(self.duty_a)
                    m2.forward(self.duty_b)
                    self.avg_speed = sum(encoders.speed())/2    
                    
            else:
      
                self.target_speed_a = self.target_speed_b = self.target_speed
                
                if self.angle_corrected:
                    print(current_angle, self.target_angle_waypoint, self.target_speed)
                    self.corrective_angle = angle_correction.compute(current_angle, self.target_angle_waypoint, low=-self.target_speed*2, up=self.target_speed*2,sum_max=355)
                    
                    if self.corrective_angle > 0:
                        self.target_speed_b = abs(self.target_speed_b - abs(self.corrective_angle)) #subtract pid value from speed
                        if ((self.target_speed - abs(self.corrective_angle))>0):
                            m1.reverse(self.duty_a)
                            m2.reverse(self.duty_b)
                        else:
                            m1.reverse(self.duty_a)
                            m2.forward(self.duty_b)     
                            
                    else:
                        self.target_speed_a = abs(self.target_speed_a - abs(self.corrective_angle))
                        if ((self.target_speed - abs(self.corrective_angle))>0):
                            m1.reverse(self.duty_a)
                            m2.reverse(self.duty_b)
                        else:
                            m2.reverse(self.duty_b)
                            m1.forward(self.duty_a)
                else:
                    m1.reverse(self.duty_a)
                    m2.reverse(self.duty_b)
                    
                
            


    def drive(self, speed, direction, is_angle_corrected, val=0):
        # Method to drive the vehicle
        vehicle_driver.update_angle(val)
        self.is_driving = 1
        self.target_speed = speed
        self.motor_dir = direction
        self.angle_corrected = is_angle_corrected
        self.waypoint_value = val
          
    def stop(self):
        self.is_driving = 0
        self.target_speed = 0
        m1.stop()
        m2.stop()
        
    def turn(self, speed, angle):
        self.is_driving = 1
        self.target_speed = speed
        self.angle_corrected = 1
        self.referance_angle = float(navigation.get_angle())
        self.target_angle_waypoint = self.referance_angle + angle
        while not(math.isclose(float(navigation.get_angle()), self.target_angle_waypoint, rel_tol = 1)):
            pass
        

    def collision(self, speed, direction):
        if direction:
            self.drive(speed, 0, 0) #Reverse
            utime.sleep(1) 
            self.turn(speed, -90) #Turn left
            self.drive(speed, 1, 0) # Drive forward
            utime.sleep(1)
            self.turn(speed, 90) #Turn back
            self.drive(speed, 1, 0) # Drive forward
            utime.sleep(1)
        else:
            self.drive(speed, 0, 0) #Reverse
            utime.sleep(1) 
            self.turn(get_angle()+90,speed) #Turn right
            self.drive(speed, 1, 0) # Drive forward
            utime.sleep(2)
            self.turn(get_angle()-90,speed) #Turn back
            self.drive(speed, 1, 0) # Drive forward
            utime.sleep(1)

# Instantiate DriveSystem class and initialize Timer interrupt
vehicle_driver = DriveSystem()



