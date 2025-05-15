import time
from machine import Pin, PWM

class motor:
    def __init__(self, pwm,dir1,dir2):
        self.pwm_pin = PWM(Pin(pwm))
        self.dir1_pin = Pin(dir1, Pin.OUT)
        self.dir2_pin = Pin(dir2, Pin.OUT)
        self.pwm_pin.freq(200)
    def forward(self, speed):
        self.pwm_pin.duty_u16(speed)
        self.dir1_pin.value(1)
        self.dir2_pin.value(0)
    def reverse(self, speed):
        self.pwm_pin.duty_u16(speed)
        self.dir1_pin.value(0)
        self.dir2_pin.value(1)
    def stop(self):
        self.pwm_pin.duty_u16(0)
        self.dir1_pin.value(0)
        self.dir2_pin.value(0)
    
class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        
        self.prev_error = 0
        self.integral = 0
        self.output = 0
        
        # Additional attributes for saturation and anti-windup
        
    def compute(self, measured_value, setpoint, low, up, sum_max):
        self.output_min = low  # Change these values according to your system
        self.output_max = up   # Change these values according to your system
        self.error_sum_max = sum_max  # Maximum allowable sum for the error
        
        error = setpoint - measured_value
        P = self.Kp * error  # P term
        
        self.integral += error
        self.integral = max(min(self.integral, self.error_sum_max), -self.error_sum_max)  # Anti-windup
        
        I = self.Ki * self.integral  # I term
        
        D = self.Kd * (error - self.prev_error)  # D term
        
        # PID Output without clamping
        output = P + I + D
        
        # Apply output saturation
        if output > self.output_max:
            output = self.output_max
        elif output < self.output_min:
            output = self.output_min
        
        # Update previous error for next iteration
        self.prev_error = error
        
        # Store the output for use or analysis
        self.output = output
        
        return output
        
        
