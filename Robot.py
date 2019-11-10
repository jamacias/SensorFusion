import pandas as pd 
import matplotlib.pyplot as plt
import numpy as np 

from IMU import IMU
from Camera import Camera
from Map import Map

from collections import Counter


class Robot(object):
    def __init__(self):
        self.calibration_files =["./data/IMU_static.csv","./data/IMU_dynamic.csv" ,"./data/CameraModuleCalibration.csv"]
        self.imu = IMU(self.calibration_files[:-1])
        self.camera = Camera(self.calibration_files[2])
        self.map = Map()
    
    def calibrate(self):
        self.imu.calibrate()
        self.camera.calibrate()
    
    def _localize_one_point(self, index_qr_1, index_qr_2, camera_data):
        _debug = False

        #first point
        qr_x_center_1 = camera_data.iloc[index_qr_1,2]
        qr_num_1 = camera_data.iloc[index_qr_1,1]
        qr_dist_1 =  self.camera.get_distance(camera_data.iloc[index_qr_1,5]) #camera_data.iloc[0,6]

        #second point need for localization     
        qr_x_center_2 = camera_data.iloc[index_qr_2,2]
        qr_num_2 = camera_data.iloc[index_qr_2,1]
        qr_dist_2 = self.camera.get_distance(camera_data.iloc[index_qr_2,5]) #camera_data.iloc[i,6]

        angle_1 , _ = self.camera.get_angle(qr_x_center_1, qr_num_1 )
        glob_cord_1 = self.map.get_qr_global_coordintates_cm(qr_num_1)
        angle_2, _  =  self.camera.get_angle(qr_x_center_2, qr_num_2 )
        glob_cord_2 = self.map.get_qr_global_coordintates_cm(qr_num_2)
        
        phi = (np.abs(angle_1) + np.abs(angle_2))*np.pi/180

        #distance between two points
        dist = np.sqrt( qr_dist_1**2 + qr_dist_2**2 - 2*qr_dist_1*qr_dist_2*np.cos(phi) ) # np.abs(glob_cord_2[0]-glob_cord_1[0])
        
        alpha = np.arcsin( (qr_dist_1* np.sin(phi))/dist)
        x = (np.cos(alpha)*qr_dist_2) + glob_cord_1[0]
        y = (np.sin(alpha)*qr_dist_2) - glob_cord_1[1]
        
        if _debug:
            print(camera_data)        
            #print("Qr_angle")
            #print("center x : ", camera_data.iloc[1,2] , " qr_number : ", camera_data.iloc[1,1])
        
            print("cord_1 ", glob_cord_1 )
            print("dist_1 ",qr_dist_1, ", dits_1**2 ",qr_dist_1**2 )
            print("cord_2 ", glob_cord_2)
            print("dist_2 ",qr_dist_2, ", dits_2**2 ",qr_dist_2**2 )
            print("dist between two qr", dist)
            print("sin alpha : ", (qr_dist_1* np.sin(phi))/dist)
            print("alpha :", alpha, " alpha_deg : ", alpha*180/np.pi)
            print(" np.sin(alpha)*qr_dist_2 ", np.sin(alpha)*qr_dist_2, " glob_cord_2[1] : ",  glob_cord_2[1])
    
            print("x ", x, "y", y)   
            print("phi sum ", phi, "phi sum deg ", phi*180/np.pi)

        return [np.abs(x), np.abs(y)]



    def localize(self,file):
        print("--------------LOCALIZATION")
        camera_data = pd.read_csv(file)
        # number of Qr detecter
        qr_detected = list(set(camera_data.iloc[:, 1]))
        n_qr = len(qr_detected)
        positions = []
        for k in range(len(qr_detected)-1):
            positions.append(self._localize_one_point(qr_detected[k], qr_detected[k+1], camera_data))
        positions = np.array(positions)
        print(positions)
        print(np.mean(positions , axis=0))

        return 0
