#!/usr/bin/env python
import rospy
import strength as strength
from ble_scanner.msg import *
from binascii import hexlify

import pygatt
from std_msgs.msg import Int8
from std_msgs.msg import Float64
import binascii

def hexStrToInt(hexstr):

        val = int(hexstr[0:2],16) + (int(hexstr[3:5],16)<<8)

        if ((val&0x8000)==0x8000): # treat signed 16bits

            val = -((val^0xffff)+1)

        return val

def main():
   # init control node
   rospy.init_node('strength', anonymous=False)
   forca_msg=Int8()
   rev_msg = Float64()
   stre=strength.Strength()
   adapter = pygatt.BGAPIBackend()
   #adapter = pygatt.GATTToolBackend()
   #YOUR_DEVICE_ADDRESS =  "C2:2F:AE:9E:AB:D0"
   YOUR_DEVICE_ADDRESS = "FF:A6:B9:57:F6:DC"
#   YOUR_DEVICE_ADDRESS2 =  "C2:2F:AE:9E:AB:D0"
   ADDRESS_TYPE = pygatt.BLEAddressType.random
   adapter.start()
#   while True:  
#         try:
   print("connecting")
   device = adapter.connect(YOUR_DEVICE_ADDRESS, address_type=ADDRESS_TYPE)
   print("connected")
#            break
#         except pygatt.exceptions.NotConnectedError:
#            print('Waiting1...')
#

   
#   while True:  
#        try:
#            print("connecting2")
#            device2 = adapter.connect(YOUR_DEVICE_ADDRESS2, address_type=ADDRESS_TYPE)
#            print("connected2")
#            break
#        except pygatt.exceptions.NotConnectedError:
#            print('Waiting2...')
#            
#            
#   pub={}
#   pub['strength'] = rospy.Publisher('strength/strength',Int8, queue_size=10)
#   pub['revolution'] = rospy.Publisher('strength/revolution', Float64, queue_size=10)
  
   while not rospy.is_shutdown():
        msg_notify=stre.notify(device)
#        msg_notify2=stre.notify(device2)
#        if msg_notify2 is None:
#            adapter.stop()
#            
        msg_strength=msg_notify
#        msg_strength2=int(msg_notify2[4]+msg_notify2[5], 16)
#
#        msg_rev = int(msg_notify[8] + msg_notify[9], 16)
#        msg_rev2 = int(msg_notify2[8] + msg_notify2[9], 16)

#        rev_msg.data=msg_rev
#        forca_msg.data= msg_strength
#        print(bin(int( msg_strength, 16))[2:].zfill(8)[0:16])
        print('Received teste: {} forca {} voltas {} '.format(msg_notify[0:5],msg_notify,msg_notify))

        print('Received data1: {} forca {} voltas {} '.format(hexStrToInt(msg_notify),msg_notify,msg_notify))
#        print('Received data2: {} forca {} voltas {} '.format(msg_notify2,msg_strength2,msg_rev2))

#        pub['strength'].publish(forca_msg)
#        pub['revolution'].publish(rev_msg)
   adapter.stop()
   
   
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
