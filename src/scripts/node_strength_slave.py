#!/usr/bin/env python
import rospy
import strength as strength
from ble_scanner.msg import *
import pygatt
from std_msgs.msg import Int8
def hexStrToInt(hexstr):

        val = int(hexstr[0:2],16) + (int(hexstr[3:5],16)<<8)

        if ((val&0x8000)==0x8000): # treat signed 16bits

            val = -((val^0xffff)+1)

return val
def main():
   # init control node
   rospy.init_node('strength1', anonymous=False)
   forca_msg=Int8()
   stre=strength.Strength()
   adapter = pygatt.BGAPIBackend()
   #adapter = pygatt.GATTToolBackend()
   YOUR_DEVICE_ADDRESS =  "C2:2F:AE:9E:AB:D0"
   #YOUR_DEVICE_ADDRESS = "FF:A6:B9:57:F6:DC"
   ADDRESS_TYPE = pygatt.BLEAddressType.random
   adapter.start()
   print("connecting")
   device = adapter.connect(YOUR_DEVICE_ADDRESS, address_type=ADDRESS_TYPE)
   print("connected")
   pub={}
   pub['strength1'] = rospy.Publisher('strength1',Int8, queue_size=10)

  
   while not rospy.is_shutdown():
        msg_notify=stre.notify(device)
        msg_strength=int(msg_notify[4]+msg_notify[5], 16)
        forca_msg.data= msg_strength
        print('Received data: {} forca {} '.format(msg_notify,msg_strength))
        pub['strength1'].publish(forca_msg)
   adapter.stop()
   
   
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
