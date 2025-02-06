from collections import deque
import numpy as np

class IMUBuffer():
    def __init__(self, size):
        
        self.size = size
        
        self.data = deque(maxlen=size)
        
    def __getitem__(self, key):
        return np.array([self.data[key]])
        
    def __len__(self):
        return len(self.gyro_X)
        
    def add(self, data):
        self.gyro_X.append(data[0])
        self.gyro_Y.append(data[1])
        self.gyro_Z.append(data[2])
        
        self.accel_X.append(data[3])
        self.accel_Y.append(data[4])
        self.accel_Z.append(data[5])
        
    def variance(self):
        return np.array([np.var(self.gyro_X),
                         np.var(self.gyro_Y),
                         np.var(self.gyro_Z),
                         np.var(self.accel_X),
                         np.var(self.accel_Y),
                         np.var(self.accel_Z)])

        
        
    
if __name__ == '__main__':
    size = 10
    # buffer = IMUBuffer(size)
    
    # for i in range(5):
    #     data = np.random.rand(6,2)
    #     print(data[0])
    #     buffer.add(data)
        
    
    # for i in range(len(buffer)):
    #     print(buffer[i])
    
    # print(buffer.variance())
    
    deq = deque(maxlen=size)
    
    data = np.random.rand(6,2)
    deq.append(data)
    
    print(np.array(deq))
    