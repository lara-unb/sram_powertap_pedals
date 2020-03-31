from binascii import hexlify
import  binascii 
import struct
from time import sleep
global msg_aux
class Strength:
    def handle_data(self, handle, value):
#        binascii.b2a_hex(value).decode('ascii')
        self.msg_aux=hexlify(value)

    def notify(self, device):
        try:
                self.msg_aux = None
                device.subscribe("00002a63-0000-1000-8000-00805f9b34fb",callback=self.handle_data)
                count=0
                while True:
                    if self.msg_aux is not None:
#                        print(' contador {}'.format(count))
                        return self.msg_aux
                    else:
                        count=count+1
                        sleep(0.005)
        finally:
               pass
