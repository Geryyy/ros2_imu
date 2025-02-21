import spidev
import numpy as np
import time
import RPi.GPIO as GPIO
from colorama import Fore, Style

GYRO = 0
ACCEL = 1
    
GLOB_CMD = 0x3E
DIAG_STAT = 0x02
X_GYRO_LOW = 0x04
X_GYRO_OUT = 0x06
Y_GYRO_LOW = 0x08
Y_GYRO_OUT = 0x0A
Z_GYRO_LOW = 0x0C
Z_GYRO_OUT = 0x0E
X_ACCEL_LOW = 0x10
X_ACCEL_OUT = 0x12
Y_ACCEL_LOW = 0x14
Y_ACCEL_OUT = 0x16
Z_ACCEL_LOW = 0x18
Z_ACCEL_OUT = 0x1A
TEMP_OUT = 0x1E
SMPL_CNTR = 0x1C
    

class ADIS16460():
    def __init__(self, dev, cs, mode, speed, dr, rst):

        # SPI Setup
        self.spi = spidev.SpiDev()
        self.spi.open(dev, cs)
        self.spi.mode = mode
        self.spi.max_speed_hz = speed
        self.spi.no_cs = False

        self.dev = dev
        self.cs = cs

        # GPIO Setup        
        self.dr = dr
        self.rst = rst
        
        # Sensor Setup
        self.scale_gyro = 1e-9
        self.scale_accel = 3.7e-8
        
        
        self.count = 0
        self.checksum_count = 0
            
        self._setup_gpio()
        self._reset_spi()
            
    def _setup_gpio(self):
        
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.dr, GPIO.IN)
        GPIO.setup(self.rst, GPIO.OUT)
    
    def _reset_spi(self):
        
        GPIO.output(self.rst, GPIO.LOW)
        time.sleep(0.001)
        GPIO.output(self.rst, GPIO.HIGH)
    
    def _convert(self, channel, OUT, LOW = [0x00, 0x00]):
        value = (OUT[0] << 24) | (OUT[1] << 16) | (LOW[0] << 8) | LOW[1]
    
        intValue = value
    
        if (value & (1 << (32 - 1))) != 0:
            value = value - (1 << 32)

        if channel == GYRO:
            value = value * self.scale_gyro
        elif channel == ACCEL:
            value = value * self.scale_accel
        else:
            return None
        
        binary = format(intValue, f'0{32}b')
        
        # if channel is ACCEL:
        #     print(f'{OUT}, {LOW}, {binary}, {value}')
        
        return value
    
    def _read_burst(self):
        
        # Send global command        
        send = [0] * 2*11
        send[0] = GLOB_CMD

        # Read 11 registers
        resp = self.spi.xfer(send)

        # Remove first two bytes -> data starts at byte 3
        resp = resp[2:]

        return resp

        
    def read(self):
        self.count = self.count + 1   
        
        if not GPIO.input(self.dr):
            print(Fore.YELLOW + "Data not Ready!" + Style.RESET_ALL)
            return None
        
        resp = self._read_burst()
        
        # Extract data
        diag_stat = resp[0:2]
        x_gyro = resp[2:4]
        y_gyro = resp[4:6]
        z_gyro = resp[6:8]
        x_accel = resp[8:10]
        y_accel = resp[10:12]
        z_accel = resp[12:14]
        
        temp = resp[14:16]
        smpl_cntr = resp[16:18]
        checksum = resp[18:20]
                      
        # Calculate checksum
        calc = diag_stat[0] + x_gyro[0] + y_gyro[0] + z_gyro[0] + x_accel[0] + y_accel[0] + z_accel[0] + temp[0] + smpl_cntr[0] + \
                diag_stat[1] + x_gyro[1] + y_gyro[1] + z_gyro[1] + x_accel[1] + y_accel[1] + z_accel[1] + temp[1] + smpl_cntr[1]
        
        checksum = (checksum[0] << 8) | checksum[1]
        
        if checksum != calc:
            self.checksum_count = self.checksum_count + 1
            print(Fore.RED + "Checksum Error! count: {}".format(self.checksum_count) + Style.RESET_ALL)
            # self._reset_spi()
            return None
        
        # Convert data
        data = np.empty((6))
        
        data[0] = self._convert(GYRO, x_gyro)
        data[1] = self._convert(GYRO, y_gyro)
        data[2] = self._convert(GYRO, z_gyro)
        data[3] = self._convert(ACCEL, x_accel)
        data[4] = self._convert(ACCEL, y_accel)
        data[5] = self._convert(ACCEL, z_accel)

        return data
            
            
if __name__ == '__main__':
    dev = 0
    spi_freq = 1000000
    spi_mode = 3
    
    # Device 0
    cs0 = 0 # spi0, ce0
    dr0 = 25
    rst0 = 12
    
    # Device 1
    cs1 = 1 # spi0, ce1
    dr1 = 26
    rst1 = 13
    
    dev0 = ADIS16460(dev, cs0, spi_mode, spi_freq, dr0, rst0)
    dev1 = ADIS16460(dev, cs1, spi_mode, spi_freq, dr1, rst1)
    cnt = 0
    while cnt < 50:
        dev0_read = dev0.read()
        if dev0_read is None:
            print(Fore.RED + "Dev0 Read failed!" + Style.RESET_ALL)
        else:
            print(Fore.GREEN + "Dev0 Read Successull!" + Style.RESET_ALL)

        dev1_read = dev1.read()
        if dev1_read is None:
            print(Fore.RED + "Dev1 Read failed!" + Style.RESET_ALL)
        else:
            print(Fore.GREEN + "Dev1 Read Successull!" + Style.RESET_ALL)


        time.sleep(0.01)
        cnt += 1

        