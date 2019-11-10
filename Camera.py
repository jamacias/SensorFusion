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
        dist_cam_qr_cm, height_qr , x_pos = self.read_data()
        #data collected manually
        height_qr = np.array([36.8,41.4,46.4,51.4,56.4,61.4,66.4,71.4,76.4,81.4,86.4,91.4,96.4,101.4,106.4,111.4,116.4,121.4,126.4,131.4,136.4])
        dist_cam_qr_cm = [176,156,137,124,112,103,96,88,83,78,73,68,65,62,60,55,53,52,50,47,46]


        # least square error for fitting line
        A = np.vstack([1/height_qr, np.ones(len(1/height_qr))]).T
        dist_cam_qr = np.array(dist_cam_qr_cm)
        m, b = np.linalg.lstsq(A, dist_cam_qr, rcond=None) [0]
        self.focal_lenght = m/self.qr_dimension
        self.bias = b

        print("f_l : ", self.focal_lenght)
        print("bias : ", self.bias)

        #plt.plot(1/height_qr, m*(1/height_qr) + b, 'r', label='Fitted line')
        #plt.legend()    
        #plt.plot(1/height_qr, dist_cam_qr_cm,'.')
        #plt.show()

    def get_angle(self, qr_x, qr_num):
        return [(np.arctan2(qr_x,self.focal_lenght)*180/np.pi) , qr_num]

    def get_distance(self,qr_h):
        return (self.qr_dimension*self.focal_lenght)/qr_h + self.bias
    
    def localize(self):
        camera_localization = pd.read_csv("./data/CameraModuleLocalization1.csv")
        print(camera_localization)