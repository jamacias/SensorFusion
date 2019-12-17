import pandas as pd 
import matplotlib.pyplot as plt
import numpy as np 
from Accelerometer import Accelerometer
from Gyroscope import Gyroscope

class IMU(object):
    def __init__(self,files):
        self.files = files
        self.gyroscope = Gyroscope(files[0]) # static file to the gyroscope 
        self.accelerometer = Accelerometer(files[1]) # dynamic file to the accelerometer
    
    def calibrate(self):
        self.gyroscope.calibrate()
        self.accelerometer.calibrate()

    def set_navigation_file(self,file):
        self.gyroscope.set_navigation_file(file)
        