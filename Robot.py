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
    
    def _localize_one_point(self, qr_1, qr_2, camera_data):
        _debug = False

        wall_qr_1 = self.map.get_qr_global_coordintates_cm(qr_1)
        wall_qr_2 = self.map.get_qr_global_coordintates_cm(qr_2)
        if wall_qr_1[2] != wall_qr_2[2]:
            return [-1,-1]
        elif wall_qr_1[2] == 'l1' or  wall_qr_1[2] == 'l3':
            if wall_qr_1[1]>wall_qr_2[1]:
                qr_1, qr_2 = qr_2, qr_1 
        elif wall_qr_1[2] == 'l2' or  wall_qr_1[2] == 'l4':
            if wall_qr_1[0]>wall_qr_2[0]:
                qr_1, qr_2 = qr_2, qr_1 
        

        #first point data from data 
        qr_x_center_1, qr_num_1, qr_dist_1 =  self.get_value_qr_from_data(qr_1, camera_data)

        #second point needed for localization     
        qr_x_center_2, qr_num_2, qr_dist_2 =  self.get_value_qr_from_data(qr_2, camera_data)
     

        angle_1 , _ = self.camera.get_angle(qr_x_center_1, qr_num_1 )
        glob_cord_1 = self.map.get_qr_global_coordintates_cm(qr_num_1)
        angle_2, _  =  self.camera.get_angle(qr_x_center_2, qr_num_2 )
        glob_cord_2 = self.map.get_qr_global_coordintates_cm(qr_num_2)
        
        phi = (np.abs(angle_1) + np.abs(angle_2))*np.pi/180

        #distance between two points
        dist = np.sqrt( qr_dist_1**2 + qr_dist_2**2 - 2*qr_dist_1*qr_dist_2*np.cos(phi) ) 
        
        alpha = np.arcsin( (qr_dist_2* np.sin(phi))/dist)
        #TODO detection which wall is and use the proper one
        
        # for horizontal wall 
        if glob_cord_1[2] == 'l2' or  glob_cord_1[2] == 'l4':
             # for horizontal wall 
            x = (np.cos(alpha)*qr_dist_1) + glob_cord_1[0]
            y = (np.sin(alpha)*qr_dist_1) - glob_cord_1[1]
        else:
            # fine for vertical wall
            x = (np.sin(alpha)*qr_dist_1) - glob_cord_1[0] 
            y = (np.cos(alpha)*qr_dist_1) + glob_cord_1[1]     

        if _debug:
            print(camera_data)            
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


    def get_value_qr_from_data(self, qr,camera_data):
        index_qr=0
        while camera_data.iloc[index_qr,1] == qr:
            index_qr +=1
        
        qr_x_center = camera_data.iloc[index_qr,2]
        qr_num = camera_data.iloc[index_qr,1]
        qr_dist =  self.camera.get_distance(camera_data.iloc[index_qr,5])

        return qr_x_center, qr_num, qr_dist 


    def localize(self,file):
        print("--------------LOCALIZATION")
        camera_data = pd.read_csv(file)
        # number of Qr detecter
        qr_detected = list(set(camera_data.iloc[:, 1]))
        positions = []
        for k in range(len(qr_detected)-1):
            positions.append(self._localize_one_point(qr_detected[k], qr_detected[k+1], camera_data))
        positions = np.array(positions)
        print(positions)
        print(np.mean(positions , axis=0))

        return 0
