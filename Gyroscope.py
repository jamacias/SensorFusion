import pandas as pd 
import matplotlib.pyplot as plt
import numpy as np 


class Gyroscope(object):
    def __init__(self,file):
        self.file = file
        self.bias = [] # x y z bias 
        self.gyro_xyz = []
        self.covariance_matrix = []
    

    def read_data(self, file):
        imu_data = pd.read_csv(file)
        self.gyro_xyz = imu_data.iloc[:,6:9]
        

    def calibrate(self):
        self.read_data(self.file)
        self.compute_bias()
        self.compute_covariance()
    
    def compute_bias(self):
        
        gyro_bias_x = np.mean(self.gyro_xyz.iloc[:,0])
        gyro_bias_y = np.mean(self.gyro_xyz.iloc[:,1])
        gyro_bias_z = np.mean(self.gyro_xyz.iloc[:,2])
        plt.plot(self.gyro_xyz.iloc[:,0])
        plt.plot(self.gyro_xyz.iloc[:,1])
        plt.plot(self.gyro_xyz.iloc[:,2])
        plt.legend(["x","y","z"])
        plt.xlabel("Sample ")
        plt.ylabel("Angular speed [degree/s]")
        #plt.show()

        self.bias=  [gyro_bias_x, gyro_bias_y,gyro_bias_z]
        print("Gyroscope bias: ", self.bias)

    def compute_covariance(self):
        # self.gyro_xyz has the variables as columns. np.conv uses rows for vars by default.
        self.covariance_matrix = np.cov(self.gyro_xyz, rowvar=False)
        print("Covariance matrix: ", self.covariance_matrix)
