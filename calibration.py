import pandas as pd 
import matplotlib.pyplot as plt
import numpy as np 
from Robot import Robot





if __name__ == "__main__":
    
    # localize the robot
    robot = Robot()
    robot.calibrate()

    
    print("top wall -----------")
    robot.localize("./data/CameraModuleLocalization1.csv")
    
    print("left wall ----------")
    robot.localize("./data/CameraModuleLocalization2.csv")
   
    print("right wall ---------")
    robot.localize("./data/CameraModuleLocalization4.csv")

    print("down wall ---------")
    robot.localize("./data/CameraModuleLocalization3.csv")
 
