from machine import Pin, I2C
class i2c_mux:
    def __init__(self, address):
        self.address = address
        self.i2c = I2C(1, scl=Pin(11), sda=Pin(10), freq=400000)
        print(self.i2c.scan())
    def change_channel(self, channel):
        self.i2c.writeto(self.address, (1 << channel).to_bytes(1,'big'))




        
        
