import pandas as pd 
import matplotlib.pyplot as plt
import numpy as np 

class Camera(object):
    def __init__(self,file):
        self.file= file
        self.focal_lenght = 1
        self.qr_dimension = 11.5 #cm
        
    
    def read_data(self):
        # info file
        # timestamp , qr number, center_x , center_y , width_qr, height_qr , dist_cam_qr_cm, attitude_qr_cam_deg
        camera_data = pd.read_csv(self.file)
        
        dist_cam_qr_cm = camera_data.iloc[:, 6]
        height_qr = camera_data.iloc[:, 5]
        
        return dist_cam_qr_cm , height_qr , camera_data.iloc[:, 2]
        
    
    def calibrate(self):
        dist_cam_qr_cm, height_qr , x_pos= self.read_data()
        # least square error for fitting line
        A = np.vstack([1/height_qr, np.ones(len(1/height_qr))]).T
        dist_cam_qr = np.array(dist_cam_qr_cm.values)
        m, b = np.linalg.lstsq(A, dist_cam_qr, rcond=None) [0]
        self.focal_lenght = m/self.qr_dimension
        self.bias = b

        print("f_l : ", self.focal_lenght)

        print(np.arctan2(x_pos,self.focal_lenght)*180/np.pi)
        
        #plt.plot(1/height_qr, m*(1/height_qr) + b, 'r', label='Fitted line')
        #plt.legend()    
        #plt.plot(1/height_qr, dist_cam_qr_cm,'.')
        #plt.plot(camera_data.iloc[:, 3])
        #plt.show()