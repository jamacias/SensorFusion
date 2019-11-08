import pandas as pd 
import matplotlib.pyplot as plt
import numpy as np 
from IMU import IMU


if __name__ == "__main__":
    
    files =["./data/IMU_static.csv","./data/IMU_dynamic.csv" ]
    imu = IMU(files)
    imu.calibrate()
