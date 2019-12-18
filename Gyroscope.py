import pandas as pd 
import matplotlib.pyplot as plt
import numpy as np 




class Gyroscope(object):
    def __init__(self,file):
        self.file = file

        self.bias = [] # x y z bias 
        self.gyro_xyz = []
        self.covariance_matrix = []
        self.step = 0 # variable for reading trough the all file 
    

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
        """ 
        plt.plot(self.gyro_xyz.iloc[:,0])
        plt.plot(self.gyro_xyz.iloc[:,1])
        plt.plot(self.gyro_xyz.iloc[:,2])
        plt.legend(["x","y","z"])
        plt.xlabel("Sample ")
        plt.ylabel("Angular speed [degree/s]")
        plt.show()
        """
        self.bias=  [gyro_bias_x, gyro_bias_y,gyro_bias_z]
        print("Gyroscope bias: ", self.bias)
    
    #set navigation file for the gyroscope 
    #the only relevant parameter for the navigation is the z the other are all around 0
    def set_navigation_file(self,_file):
        imu_data = pd.read_csv(_file)
        self.navigation_gyro_xyz = imu_data.iloc[:,6:9]

    def get_measurement(self):
        #measurement -> [x,y,z] degrees/s at each step return the measurement of the gyro, summing the bias and increase the step of 1 so the next
        # time the next value  of the file will be returned
        # TODO understand how to handle the variance correctly
        measurement = [self.navigation_gyro_xyz.iloc[self.step,0]+self.bias[0],self.navigation_gyro_xyz.iloc[self.step,1]+self.bias[1],self.navigation_gyro_xyz.iloc[self.step,2]+self.bias[2]]
        self.step +=1
        return measurement

    def compute_covariance(self):
        # self.gyro_xyz has the variables as columns. np.conv uses rows for vars by default.
        self.covariance_matrix = np.cov(self.gyro_xyz, rowvar=False)
        print("Covariance matrix: ", self.covariance_matrix)
