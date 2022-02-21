#!/usr/bin/python3

import smbus
import time


def _bcd2bin(value):
    """Convert binary coded decimal to Binary

    :param value: the BCD value to convert to binary (required, no default)
    """
    return value - 6 * (value >> 4)


def _bin2bcd(value):
    """Convert a binary value to binary coded decimal.

    :param value: the binary value to convert to BCD. (required, no default)
    """
    return value + 6 * (value // 10)


class DS3231:
    
    # Masking value list   n/a  sec min hr day wkday mon year
    mask_datetime = b"\xFF\x7F\x7F\x3F\x3F\x07\x1F\xFF"

    def __init__(self, bus_number, device_address=0x68):
        self.bus = smbus.SMBus(bus_number)
        self.address = device_address
        
    def get_datetime(self):
        buf = bytearray(self.bus.read_i2c_block_data(self.address, 0, 7))
        return time.struct_time(
            (
                _bcd2bin(buf[6] & self.mask_datetime[7]) + 2000,
                _bcd2bin(buf[5] & self.mask_datetime[6]),
                _bcd2bin(buf[4] & self.mask_datetime[4]),
                _bcd2bin(buf[2] & self.mask_datetime[3]),
                _bcd2bin(buf[1] & self.mask_datetime[2]),
                _bcd2bin(buf[0] & self.mask_datetime[1]),
                _bcd2bin((buf[3] & self.mask_datetime[5]) - 1),
                -1,
                -1
        ))
    
    def set_datetime(self, val=time.localtime()):
        buf = bytearray(7)
        buf[0] = _bin2bcd(val.tm_sec) & 0x7F
        buf[1] = _bin2bcd(val.tm_min)
        buf[2] = _bin2bcd(val.tm_hour)
        buf[3] = _bin2bcd(val.tm_wday + 1)
        buf[4] = _bin2bcd(val.tm_mday)
        buf[5] = _bin2bcd(val.tm_mon)
        buf[6] = _bin2bcd(val.tm_year - 2000)
        self.bus.write_i2c_block_data(self.address, 0, list(buf))


