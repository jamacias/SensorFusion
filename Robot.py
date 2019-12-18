import pandas as pd 
import matplotlib.pyplot as plt
import numpy as np 

from IMU import IMU
from Camera import Camera
from Map import Map

from scipy.optimize import least_squares
from scipy.optimize import minimize


class Robot(object):
    def __init__(self):
        self.calibration_files =["./data/IMU_static.csv","./data/IMU_dynamic.csv" ,"./data/CameraModuleCalibration.csv"]
        self.imu = IMU(self.calibration_files[:-1])
        self.camera = Camera(self.calibration_files[2])
        self.map = Map()
        self.camera_data = []
        self.qr_detected = []

        # state of the robot
        # constant velocity of the robot
        self.v = 6.68 # cm/s SUPER DRAFT ESTIMATION  
        self.delta_t = 0.05 # delta t of 0,05s used in quasi constant turning model 
        

        # position and angle of the robot 
        self.x = 0,
        self.y = 0, 
        self.phi = 0
        
    
    def calibrate(self):
        self.imu.calibrate()
        self.camera.calibrate()
    
    # Given two QR IDs, return the estimated position of the robot.
    # This is purely a geometric approximation using cosine similarities.
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
        
        #print("qr_1 ", qr_1, " qr_2 " , qr_2)
        #first point data from data 
        qr_x_center_1, qr_num_1, qr_dist_1 =  self.get_value_qr_from_data(qr_1)

        #second point needed for localization     
        qr_x_center_2, qr_num_2, qr_dist_2 =  self.get_value_qr_from_data(qr_2)
     

        angle_1 , _ = self.camera.get_angle(qr_x_center_1, qr_num_1 )
        glob_cord_1 = self.map.get_qr_global_coordintates_cm(qr_num_1)
        angle_2, _  =  self.camera.get_angle(qr_x_center_2, qr_num_2 )
        glob_cord_2 = self.map.get_qr_global_coordintates_cm(qr_num_2)
        
        if angle_1*angle_2 > 0:
            phi = np.abs((np.abs(angle_1) - np.abs(angle_2)))*np.pi/180 # angle here TODO fix a bug when angle same sign
        else :
            phi = np.abs((np.abs(angle_1) + np.abs(angle_2)))*np.pi/180
        #distance between two points
        dist = np.sqrt( qr_dist_1**2 + qr_dist_2**2 - 2*qr_dist_1*qr_dist_2*np.cos(phi) ) 
         
        """ print("dist B", dist)
        if glob_cord_1 == 'l3' or glob_cord_1 == 'l1':
            dist= np.abs(glob_cord_1[1] - glob_cord_2[1])
        else :
            dist= np.abs(glob_cord_1[0] - glob_cord_2[0])
        print("dist", dist) """
        
        alpha = np.arcsin( (qr_dist_2* np.sin(phi))/dist)

        # computing the attitude of the robot using cosindering which wall is pointing
        if wall_qr_1[2] == 'l4':
             attitude = np.abs(angle_1) + alpha*180/np.pi -90
        elif  wall_qr_1[2] == 'l3':
            attitude = np.abs(angle_1) + alpha*180/np.pi +180
        elif  wall_qr_1[2] == 'l2':
            attitude = np.abs(angle_1) + alpha*180/np.pi +90
        elif  wall_qr_1[2] == 'l1':
            attitude = np.abs(angle_1) + alpha*180/np.pi 
        
        
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
        # conversion of the attitude in radiants
        return [np.abs(x), np.abs(y), attitude*(np.pi/180)]


    # Given a QR ID, return the centre, the number and the distance
    def get_value_qr_from_data(self, qr):
        index_qr=0
        while self.camera_data.iloc[index_qr,1] != qr:
            #print("camera_data.iloc[index_qr,1] ", camera_data.iloc[index_qr,1], " ,qr, ", qr)
            index_qr +=1
            
        
        qr_x_center = self.camera_data.iloc[index_qr,2]
        qr_num = self.camera_data.iloc[index_qr,1]
        qr_dist =  self.camera.get_distance(self.camera_data.iloc[index_qr,5])

        return qr_x_center, qr_num, qr_dist 

    #return the euclidean distance between the two points
    def euclidean_dist(self, coord_qr, coord_robot):
        return np.sqrt( (coord_qr[0]-coord_robot[0])**2 + (coord_qr[1]-coord_robot[1])**2  )
    
    # get the vector y of measurement
    def get_measurement(self,qr_height):
        return self.camera.get_distance(qr_height)
    

    # computing the loss function by passing the initial guess first
    def loss(self, coord_robot):
        loss = []
        for qr in self.qr_detected:
            coord_qr = self.map.get_qr_global_coordintates_cm(qr) # Get the global coordinates of QR code qr
            qr_x_center , _,  qr_dist = self.get_value_qr_from_data(qr) # Obtain the measurement of pixel distance to the centre and distance to the QR code from the sensors
            phi_measured = self.camera.get_angle(qr_x_center, qr)[0]*(np.pi/180) # Compute the Phi from measured parameters (switch to radiants)
            
            # Compute the losses:
            # Vector of d_i and phi_i stacked that is to be minimized wrt [x_r, y_r, theta_r]
            phi_expression = np.arctan2( (coord_qr[1]-coord_robot[1]),   (coord_qr[0]-coord_robot[0]) ) - coord_robot[2]
            l_distance = qr_dist  -  self.euclidean_dist(coord_qr, coord_robot)
            l_angle = phi_measured - phi_expression # phi expression contains the three minimization variables
            loss.append(l_distance)
            loss.append(l_angle)

        loss = np.array(loss[:])

        # Return the scalar (loss)
        return np.dot(loss.T, loss)

    def localize(self,file):
        print("--------------LOCALIZATION")
        self.camera_data = pd.read_csv(file)
        # Identify the QR codes that are detected and store them in a list.
        self.qr_detected = list(set(self.camera_data.iloc[:, 1])) 
        positions = []
        # Make a first estimation using geometric relations for all the detected QR codes 
        for k in range(len(self.qr_detected)-1):
            #print("qr_detected[k] ",qr_detected[k], "qr_detected[k+1] ",qr_detected[k+1])
            # Set of localization using two QR codes at a time.
            positions.append(self._localize_one_point(self.qr_detected[k], self.qr_detected[k+1], self.camera_data))
    
        # The mean of all the localizations using two QR codes will be the initial guess of the minimization algorithm
        robo_ini_guess = np.mean(positions , axis=0)
        print("initial_guess ", robo_ini_guess)
        res = minimize(self.loss, robo_ini_guess, method='BFGS',options={ 'xtol': 1e-8,'disp': True})
        #print(res)
        print("minimized version ", res.x)

        return 0

    # passing the navigation file to the various component for the time being only imu later also camera file
    def navigation_file(self, imu_navigation_file):
        self.imu.set_navigation_file(imu_navigation_file)

    # inizialization function useful for setting the initial state at the beginning
    def initialize_state(self,_x, _y,_phi):
        self.x = _x
        self.y = _y 
        self.phi = _phi

    # given the previous observation return the state given the dynamic model
    def dynamic_model(self,_x,_y,_phi):
        #considering the velocity always stable, suggested in the lab boook
        # list with the parameter of the robot at state n 
        phi_rad = _phi * np.pi/180.0
        self.x  = _x + (self.delta_t*self.v*np.cos(phi_rad)) + 0 # TODO zero is the noise not realistic at all  
        self.y  = _y + (self.delta_t*self.v*np.sin(phi_rad)) + 0
        return self.x, self.y, _phi
