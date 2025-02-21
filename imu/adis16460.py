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
            self._reset_spi()
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
    
    dev0 = ADIS16460(0, 0, 3, 1000000, 25, 8)
    dev1 = ADIS16460(0, 1, 3, 1000000, 26, 7)
    # dev1 = None

    GPIO.setup(12, GPIO.OUT)
    GPIO.setup(13, GPIO.OUT)
    
    GPIO.output(12, GPIO.LOW)
    GPIO.output(13, GPIO.LOW)
    
    time.sleep(0.5)
    
    GPIO.output(12, GPIO.HIGH)
    GPIO.output(13, GPIO.HIGH)
    
    start = time.time()

    for i in range(10000):
        GPIO.wait_for_edge(25, GPIO.RISING)
        print(dev0.read()) 
        # dev0._read_burst()
        GPIO.wait_for_edge(26, GPIO.RISING)
        print(dev1.read()) 
        # dev1._read_burst()
        # if GPIO.event_detected(26):
            
    end = time.time()
    
    print(f'Failure Percentage dev0: {dev0.checksum_count / dev0.count * 100}%')
    print(f'Failure Percentage dev1: {dev1.checksum_count / dev1.count * 100}%')
    
    print(f'Frequency dev0: {dev0.count / (end - start)} Hz')
    print(f'Frequency dev1: {dev1.count / (end - start)} Hz')

        # end = time.time()

    GPIO.cleanup() 
    
    
    # exit()
    
    # GPIO.setmode(GPIO.BCM)
    # GPIO.setup(25, GPIO.IN)
    # GPIO.setup(26, GPIO.IN)
        
    # GPIO.setup(25, GPIO.IN)
    # GPIO.setup(26, GPIO.IN)
        
    # GPIO.add_event_detect(25, GPIO.RISING, callback=dev0._read_all)
    # GPIO.add_event_detect(26, GPIO.RISING, callback=dev1._read_all)

    exit()
        
    start = time.time()
    end = time.time()
    while (end - start) < 5:
        end = time.time()
        
    print(f'Time: {end - start}')
    print(f'Frequency: {dev0.count / (end - start)} Hz')
    print(dev0.count)
    print('End')
        
    if dev0:
        fig, axs = plt.subplots(2, 1)
        axs[0].plot(dev0.data[:, 0], 'r', label='X_GYRO')
        axs[0].plot(dev0.data[:, 1], 'g', label='Y_GYRO')
        axs[0].plot(dev0.data[:, 2], 'b', label='Z_GYRO')
        axs[0].grid(True)
        axs[0].legend()

        axs[1].plot(dev0.data[:, 3], 'r', label='X_ACCEL')
        axs[1].plot(dev0.data[:, 4], 'g', label='Y_ACCEL')
        axs[1].plot(dev0.data[:, 5], 'b', label='Z_ACCEL')
        axs[1].grid(True)
        axs[1].legend()
        plt.savefig('SPI0.0.png')
    
    if dev1:
        fig, axs = plt.subplots(2, 1)
        axs[0].plot(dev1.data[:, 0], 'r', label='X_GYRO')
        axs[0].plot(dev1.data[:, 1], 'g', label='Y_GYRO')
        axs[0].plot(dev1.data[:, 2], 'b', label='Z_GYRO')
        axs[0].grid(True)
        axs[0].legend()

        axs[1].plot(dev1.data[:, 3], 'r', label='X_ACCEL')
        axs[1].plot(dev1.data[:, 4], 'g', label='Y_ACCEL')
        axs[1].plot(dev1.data[:, 5], 'b', label='Z_ACCEL')
        axs[1].grid(True)
        axs[1].legend()
        plt.savefig('SPI0.1.png')
        
    GPIO.cleanup()
