import utime
from ulab import numpy
from micropyGPS import *


uart = machine.UART(0, rx=1, tx=0, baudrate=9600, bits=8, parity=None, stop=1, timeout=5000)
gps = MicropyGPS()

def update_gps():
    buf = uart.readline()
    for char in buf:
        gps.update(chr(char))

update_gps()

print("Getting GPS")

while (gps.latitude == 0 or gps.longitude == 0):
    update_gps()
    utime.sleep(0.1)
    print("Establishing Link:", gps.latitude, gps.longitude)
    
print("Got IT")
            
a = []
try:     
    while True:
        update_gps()
        print(gps.latitude,gps.longitude)
        a.append([gps.latitude,gps.longitude])
        utime.sleep(1)

except KeyboardInterrupt:
    print(a)
    b=numpy.asarray(a)
    numpy.savetxt("trial1.csv", b, delimiter=",")
    

    