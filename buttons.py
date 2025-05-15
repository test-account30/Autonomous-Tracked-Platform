from machine import Pin
import utime
from drive_system import *

bumper1 = Pin(12, Pin.IN)
bumper2 = Pin(13, Pin.IN)

collision1 = collision2 = 0

def debounce_handler(pin):
    global collision1, collision2
    collision1 = 1
    collision2 = 1
        
bumper1.irq(trigger=Pin.IRQ_FALLING, handler=debounce_handler)
bumper2.irq(trigger=Pin.IRQ_FALLING, handler=debounce_handler)