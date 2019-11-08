import pandas as pd 
import matplotlib.pyplot as plt
import numpy as np 


class Gyroscope(object):
    def __init__(self,file):
        self.file = file
        self.bias = [] # x y z bias 
        self.gyro_xyz = []
    

    def read_data(self, file):
        imu_data = pd.read_csv(file)
        self.gyro_xyz = imu_data.iloc[:,6:9]
        

    def calibrate(self):
        self.read_data(self.file)
        self.compute_bias()
    
    def compute_bias(self):
        gyro_bias_x = np.mean(self.gyro_xyz.iloc[:,0])
        gyro_bias_y = np.mean(self.gyro_xyz.iloc[:,1])
        gyro_bias_z = np.mean(self.gyro_xyz.iloc[:,2])
        self.bias=  [gyro_bias_x, gyro_bias_y,gyro_bias_z]
        print(self.bias)