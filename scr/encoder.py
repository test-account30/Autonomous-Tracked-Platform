from machine import Pin, I2C
from encoder_driver import *
from i2c_mux_driver import *
import utime
from micropython import const
import math

class EncoderManager:
    def __init__(self):
        self.mux = i2c_mux(0x70)
        self.mux.change_channel(2)
        self.i2c = I2C(1, scl=Pin(11), sda=Pin(10), freq=400000)
        self.encoder_a = AS5600(self.i2c, AS5600_id)
        self.encoder_b = AS5600(self.i2c, AS5600_id)
        self.a_speed = 0
        self.b_speed = 0
        self.a_speed_prev = 0
        self.b_speed_prev = 0
        self.a_val = 0
        self.b_val = 0
        self.a_val_prev = 0
        self.b_val_prev = 0
        self.time_now = utime.ticks_ms()
        self.time_prev = utime.ticks_ms()
        
        self.encoder_idler_encoder_teeth = const(32)
        self.encoder_wheel_teeth = const(22)
        self.gear_ratio = const(self.encoder_wheel_teeth/self.encoder_idler_encoder_teeth)
        
        self.idler_gear_diameter = const(41.5)
        self.rotation_distance = const(math.pi*self.idler_gear_diameter)
        self.distance_travelled = [0,0]
        self.MAX_ENCODER_VALUE = 4095
        
    def _encoderRead(self, val):
        if val:
            self.mux.change_channel(1)
            return self.encoder_a.ANGLE
        else:
            self.mux.change_channel(2)
            return self.encoder_b.ANGLE
        
    def calculate_rotations_per_second(self, current_value, previous_value, dt):
       """Calculates rotations per second, handling rollover."""
       delta = current_value - previous_value
       if abs(delta) > self.MAX_ENCODER_VALUE / 2:  # Account for rollover
           delta += self.MAX_ENCODER_VALUE * (-1 if delta < 0 else 1)
       return delta / self.MAX_ENCODER_VALUE * dt  # Rotated fraction of a full cycle
        
    def Update(self, smooth_factor = 0.6):

        try:
            self.mux.change_channel(2)
            self.a_val = self._encoderRead(0)
            self.mux.change_channel(1)
            self.b_val = self._encoderRead(1)
        except:
            pass 
        
        
        self.time_now = utime.ticks_ms()
        
        self.time_delta = ((self.time_now-self.time_prev)/1000)


        
        self.a_speed = abs(((((((self.a_val - self.a_val_prev + 2048) % 4096 - 2048) / 4095) / self.time_delta)) * self.gear_ratio)*self.rotation_distance)/1000
        self.b_speed = abs(((((((self.b_val - self.b_val_prev + 2048) % 4096 - 2048) / 4095) / self.time_delta)) * self.gear_ratio)*self.rotation_distance)/1000
    
        
        self.time_prev = self.time_now
        
        self.a_val_prev = self.a_val
        self.b_val_prev = self.b_val
        
        self.a_speed = self.a_speed * smooth_factor + (1-smooth_factor)*self.a_speed_prev
        self.b_speed = self.b_speed * smooth_factor + (1-smooth_factor)*self.b_speed_prev
        
        
        self.a_speed_prev = self.a_speed
        self.b_speed_prev = self.b_speed
        
        
    def speed(self):
        return [abs(self.a_speed), abs(self.b_speed)]
    
    def get_distance_travelled(self):
        return sum(self.distance_travelled)/2
    
    def distance_clear(self):
        self.distance_travelled.clear()
        
    
encoders = EncoderManager()
        
        