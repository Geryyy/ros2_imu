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

PROD_ID = 0x56
PROD_ID_VAL = 0x404C

# sync control
MSC_CTRL = 0x32
SYNC_OUTPUT = 0xCD
SYNC_INPUT = 0xC5
    

class ADIS16460():
    def __init__(self, dev, cs, mode, speed, dr, rst, sync_master=False):

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

        time.sleep(0.5)

        product_id = self.read_register(PROD_ID)
        if product_id != PROD_ID_VAL:
            raise Exception("ADIS16460 Product Identifier wrong! Check SPI communication.")

        if sync_master:
            self.write_register(MSC_CTRL, SYNC_OUTPUT)
        else:
            self.write_register(MSC_CTRL, SYNC_INPUT)

            
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

    # def read_register(self, reg):
    #     # Send the register address with a read command (0x00 is a read command)
    #     send = [reg, 0x00, reg+1, 0x00]  # 2-byte array: [register address, dummy byte]
        
    #     # Perform the SPI transfer
    #     resp = self.spi.xfer(send)
    #     print(resp)
    #     # The received response is in the second byte (register data)
    #     return (resp[0] << 8) | resp[1]  # Combine the two bytes into a 16-bit value

    def read_register(self, reg):
        # Send the register address with a read command (0x00 is a read command)
        send = [reg, 0x00]  # 2-byte array: [register address, dummy byte]
        # Perform the SPI transfer
        resp = self.spi.xfer(send)

        receive = [0x00,0x00]
        resp = self.spi.xfer(receive)
        # The received response is in the second byte (register data)
        return (resp[0] << 8) | resp[1]  # Combine the two bytes into a 16-bit value
        

    def write_register(self, reg, value):
        # Send the register address with a write command (MSB set for write, 0x80)
        send = [reg | 0x80, (value & 0xFF), (reg + 1) | 0x80, (value >> 8) & 0xFF]
        
        # Perform the SPI transfer (4 bytes: 2 for the register and 2 for the value)
        self.spi.xfer(send)


            
import time            
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
    
    dev0 = ADIS16460(dev, cs0, spi_mode, spi_freq, dr0, rst0, sync_master=True)
    dev1 = ADIS16460(dev, cs1, spi_mode, spi_freq, dr1, rst1, sync_master=False)

    # time.sleep(0.5)

    # cnt = 0
    # while cnt < 10:
    #     reg_addr = 0x56
    #     ret_val = dev0.read_register(reg_addr)
    #     print(f"Dev0 Register value: 0x{ret_val:2x}")
    #     ret_val = dev1.read_register(reg_addr)
    #     print(f"Dev1 Register value: 0x{ret_val:2x}")
    #     cnt += 1

    #     MSC_CTRL = 0x32
    #     SYNC_OUTPUT = 0xCD
    #     SYNC_INPUT = 0xC5
    #     dev0.write_register(MSC_CTRL, SYNC_OUTPUT)
    #     dev1.write_register(MSC_CTRL, SYNC_INPUT)

    # print("reset device")
    # dev1._reset_spi()


    # cnt = 0
    # while cnt < 50:
    #     dev0_read = dev0.read()
    #     if dev0_read is None:
    #         print(Fore.RED + "Dev0 Read failed!" + Style.RESET_ALL)
    #     else:
    #         print(Fore.GREEN + "Dev0 Read Successull!" + Style.RESET_ALL)

    #     dev1_read = dev1.read()
    #     if dev1_read is None:
    #         print(Fore.RED + "Dev1 Read failed!" + Style.RESET_ALL)
    #     else:
    #         print(Fore.GREEN + "Dev1 Read Successull!" + Style.RESET_ALL)


    #     time.sleep(0.01)
    #     cnt += 1

        