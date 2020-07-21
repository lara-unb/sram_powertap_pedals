"""

Particularly, this code is an auxiliary module for the cycling power
pedals. It consists of classes and methods that establish the BLE
comm and give support in a deeper level.

"""

import pygatt
import struct
import serial
import logging

# Display debug info:
logging.basicConfig()
logging.getLogger('pygatt').setLevel(logging.DEBUG)

# BLE characteristics mapping:
char_dict = {
    'CyclingPower': {
        'uuid':0x1818,'source':'gss',
        'identifier':'org.bluetooth.service.cycling_power'
    },
    'BatteryService': {
        'uuid':0x180F,'source':'gss',
        'identifier':'org.bluetooth.service.battery_service'
    },
    'DeviceName': {
        'uuid':0x2A00,'source':'gss',
        'identifier':'org.bluetooth.characteristic.gap.device_name'
    },
    'Appearance': {
        'uuid':0x2A01,'source':'gss',
        'identifier':'org.bluetooth.characteristic.gap.appearance'
    },
    'PeripheralPreferredConnectionParameters': {
        'uuid':0x2A04,'source':'gss',
        'identifier':'org.bluetooth.characteristic.gap.peripheral_preferred_connectn_parameters'
    },
    'ServiceChanged': {
        'uuid':0x2A05,'source':'gss',
        'identifier':'org.bluetooth.characteristic.gatt.service_changed'
    },
    'ManufacturerNameString': {
        'uuid':0x2A29,'source':'gss',
        'identifier':'org.bluetooth.characteristic.manufacturer_name_string'
    },
    'ModelNumberString': {
        'uuid':0x2A24,'source':'gss',
        'identifier':'org.bluetooth.characteristic.model_number_string'
    },
    'SerialNumberString': {
        'uuid':0x2A25,'source':'gss',
        'identifier':'org.bluetooth.characteristic.serial_number_string'
    },
    'HardwareRevisionString': {
        'uuid':0x2A27,'source':'gss',
        'identifier':'org.bluetooth.characteristic.hardware_revision_string'
    },
    'FirmwareRevisionString': {
        'uuid':0x2A26,'source':'gss',
        'identifier':'org.bluetooth.characteristic.firmware_revision_string'
    },
    'SoftwareRevisionString': {
        'uuid':0x2A28,'source':'gss',
        'identifier':'org.bluetooth.characteristic.software_revision_string'
    },
    'PnPID': {
        'uuid':0x2A50,'source':'gss',
        'identifier':'org.bluetooth.characteristic.pnp_id'
    },
    'CyclingPowerMeasurement': {
        'uuid':0x2A63,'source':'gss',
        'identifier':'org.bluetooth.characteristic.cycling_power_measurement'
    },
    'CyclingPowerFeature': {
        'uuid':0x2A65,'source':'gss',
        'identifier':'org.bluetooth.characteristic.cycling_power_feature'
    },
    'SensorLocation': {
        'uuid':0x2A5D,'source':'gss',
        'identifier':'org.bluetooth.characteristic.sensor_location'
    },
    'CyclingPowerControlPoint': {
        'uuid':0x2A66,'source':'gss',
        'identifier':'org.bluetooth.characteristic.cycling_power_control_point'
    },
    'BatteryLevel': {
        'uuid':0x2A19,'source':'gss',
        'identifier':'org.bluetooth.characteristic.battery_level'
    }
}


class Pedal(object):
    """A class used to communicate with the power pedals.

    Attributes:
        config_dict (dict): stores the static config parameters
        self.devices (dict): stores all present device handles
        self.dongles (list): present dongle names
        self.sensors (list): present sensor names    
    """
    def __init__(self, config_dict):
        self.config_dict = config_dict
        self.devices = {}
        self.dongles = []
        self.sensors = []

        for name in config_dict['dev_names']:
            dev_type = config_dict['dev_type'][name]

            if dev_type == 'DNG':  # Wired dongle
                # BlueGiga backend instance for BGAPI compatible USB adapters:
                self.devices[name] =  pygatt.BGAPIBackend()

                # Auto-discover the adapter name and start serial comm:
                self.devices[name].start()

                self.dongles.append(name)

            elif dev_type == 'WL':  # Wireless sensors
                # Address type according to PYGATT
                if config_dict['addr_type'][name] == 'random':
                    self.config_dict['addr_type'][name] = pygatt.BLEAddressType.random
                else:
                    self.config_dict['addr_type'][name] = pygatt.BLEAddressType.public

                self.devices[name] = {}
                self.sensors.append(name)

        self.initialize()

    def initialize(self):
        """Connect to the sensors."""
        for name in self.sensors:
            dev_type = self.config_dict['dev_type'][name]

            if dev_type == 'WL':
                dev_mac_addr = self.config_dict['mac_addr'][name]
                dev_addr_type = self.config_dict['addr_type'][name]
                dongle = self.config_dict['wireless_dng'][name]
                self.devices[dongle].connect(dev_mac_addr, timeout=10,
                    address_type=dev_addr_type)

    def terminate(self):
        """Stop the device comm."""
        for name in self.dongles:
            self.devices[name].stop()

    def setCharacteristic(self, char, value, name):
        """Take the characteristic label, packs its value according to
        the correct predefined format and sends it.

        Attributes:
            char (string): BLE characteristic with no white space
            value (int): information to be sent
            name (string): pedal name from initial config
        """
        UUID = self.completeUUID(char_dict[char]['uuid'])
        handle = self.devices[name].get_handle(UUID)

        value = struct.pack('<h', value)
        self.devices[name].char_write(handle, value)

    def getCharacteristic(self, char, name):
        """Take the characteristic label, reads it as bytearray, unpacks it
        according to the correct predefined format and return its value
        as a tuple.

        Attributes:
            char (string): BLE characteristic with no white space
            name (string): pedal name from initial config
        """
        UUID = self.completeUUID(char_dict[char]['uuid'])
        array = self.devices[name].char_read(UUID)
        data = struct.unpack('<B', array)
        return data

    def completeUUID(self, short_UUID):
        """Convert the short form of UUID to the full form and return it.

        Attributes:
            short_UUID (int): UUID short number, like 0x0000
        """
        return pygatt.util.uuid16_to_uuid(short_UUID)
