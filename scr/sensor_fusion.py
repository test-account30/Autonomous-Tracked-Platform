from machine import Pin, I2C
import utime as time
import i2c_mux_driver
from mpu6050 import MPU6050
from PiicoDev_QMC6310 import *
from fusion import Fusion
import utime




class sensor_fusion:
    def __init__(self):
        
        self.mux = i2c_mux_driver.i2c_mux(0x70)
        self.mux.change_channel(0)

        self.compass = PiicoDev_QMC6310(bus=1, sda=Pin(10), scl=Pin(11), range=800)
        self.fuse = Fusion()
        
        self.mux.change_channel(3)
        self.i2c = I2C(1, scl=Pin(11), sda=Pin(10), freq=400000)
        self.mpu = MPU6050(1, 10, 11, (3083, 20155, -832, -15, -35, -217))
        
        self.accel = []

    def update(self):
        self.mux.change_channel(0)
        self.compass_data = self.compass.read()
        while (self.compass_data == 'NaN'):
            self.compass_data = self.compass.read()
            print("NaN")
        
        self.mag = [self.compass_data['x'],self.compass_data['y'],self.compass_data['z']]
        
        
        self.mux.change_channel(3)
        
        ax, ay, az, gx, gy, gz = self.mpu.data
        
        self.mux.change_channel(0)
        
        self.gyro = [gx, gy, gz]
        self.accel = [ax, ay, az]
        
        self.fuse.update(self.accel, self.gyro, self.mag) # Note blocking mag read
        
        self.fuse.heading += 26
        
        if self.fuse.heading < 0:
            self.fuse.heading +=360
            
    def data(self):
        return {'pitch': self.fuse.roll, 'roll': self.fuse.pitch, 'heading': self.fuse.heading, 'accel': self.accel}
    
    
sensor_fuse = sensor_fusion()


