#!/usr/bin/env python3

"""

Particularly, this code aims to communicate with cycling power pedals,
through Bluethooth Low Energy (BLE).

"""

# Python 2 and 3 compatibility
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
from builtins import *

import modules.pedal as pedal

# Initial config, yaml later:
config_dict = {
    'dev_names': [
        'BLED112',
        'PowerTap P1-R'
    ],
    'dev_type': {
        'BLED112': 'DNG',
        'PowerTap P1-L': 'WL',
        'PowerTap P1-R': 'WL'
    },
    'mac_addr': {
        'PowerTap P1-L': 'FF:A6:B9:57:F6:DC', 
        'PowerTap P1-R': 'C2:2F:AE:9E:AB:D0'
    },
    'addr_type': {
        'PowerTap P1-L': 'random', 
        'PowerTap P1-R': 'random'
    },
    'wireless_dng': {
        'PowerTap P1-L': 'BLED112', 
        'PowerTap P1-R': 'BLED112'
    }
}


try:
    # Get pedal config
    pedal_manager = pedal.Pedal(config_dict)

    for name in pedal_manager.sensors:
        data = pedal_manager.getCharacteristic('BatteryLevel', name)
        print('Battery: ' + str(data[0]) + '%')

        # device.subscribe('CyclingPowerMeasurement', callback=handleData)

        # # print(str(handle) + ' Received data: ' + str(struct.unpack('>B', value)[0]))
        # inst_power = struct.unpack('<h', value[-4:-2])[0]
        # print(str(handle) + ' Received data: ' + str(inst_power))

        # input('Enter any key to quit...')

finally:
    pedal_manager.terminate()
