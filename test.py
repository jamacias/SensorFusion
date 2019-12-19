import pandas as pd 
import matplotlib.pyplot as plt
import numpy as np 

imu_data = pd.read_csv("data/IMUNavigation.csv")
camera_data = pd.read_csv("data/CameraModuleNavigation.csv")

#print(camera_data.columns)

#print(l)
data = camera_data.loc[(camera_data.iloc[:,0] >= 1572534381.6805596)&  (camera_data.iloc[:,0]<= 1572534381.7305596) ]
print(data.iloc[:,1])
num_detection = list(set(camera_data.iloc[:, 0])) 
#print(num_detection)
