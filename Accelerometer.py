import pandas as pd 
import matplotlib.pyplot as plt
import numpy as np 

class Accelerometer(object):
    def __init__(self,file):
        self.file = file
        self.bias = [] # x y z bias 
        self.gain = [] # x y z gain 
        self.g    = 9.8
        self.acc_xyz = []
    
    def read_data(self, file):
        imu_data = pd.read_csv(file)
        self.acc_xyz = imu_data.iloc[:,1:4]

    
    def find_value_of_interest(self, values):
        v_d = []
        v_u = []
        for v in values:
            if v >0.5:
                v_u.append(v)
            elif v <-0.5:
                v_d.append(v)
        return np.mean(v_u), np.mean(v_d)

    def compute_bias_and_gain(self):
        # plot to see the data of the accelerometer
        #plt.plot(self.acc_xyz.iloc[:,0])
        #plt.show()
        x_u, x_d = self.find_value_of_interest(self.acc_xyz.iloc[:,0]) # values up and down for x 
        y_u, y_d = self.find_value_of_interest(self.acc_xyz.iloc[:,1]) # values up and down for x 
        z_u, z_d = self.find_value_of_interest(self.acc_xyz.iloc[:,2]) # values up and down for x 

        self.bias = [(x_u+x_d)/2,(y_u+y_d)/2,(z_u+z_d)/2 ]
        self.gain = [(x_u-x_d)/(2*self.g),(y_u-y_d)/(2*self.g),(z_u-z_d)/(2*self.g)] 

        print(self.bias)
        print(self.gain)

    def calibrate(self):
        self.read_data(self.file)
        self.compute_bias_and_gain()
        