import pandas as pd 
import matplotlib.pyplot as plt
import numpy as np 


class Map(object):
    def __init__(self):

        self.wall_dimension = 120 #cm
        # qr codes 
        self.qr_dimension = 11.5 #cm 
        self.l1 = [6,5,4,30,29,28,36,35,34] #from down to top using image
        self.l1_gap_top_down = [4.6, 7.6]
        self.l2 = [10,11,12,16,17,18,22,23,24] #from left to right 
        self.l2_gap_left_right = [7.6, 4.8]
        self.l3 = [7,8,9,3,2,1,15,14,13] #from down to top 
        self.l3_gap_top_down = [5.8,6.4]
        self.l4 = [19,20,21,25,26,27,31,32,33] #from left to right
        self.l4_gap_left_right = [7.6,4.2] 
    
    def get_qr_global_coordintates_cm(self,qr_id):
        if qr_id in self.l1:
            return [ 0, (self.qr_dimension*(self.l1.index(qr_id)+1)+ +self.qr_dimension/2 +self.l1_gap_top_down[0]) ]
        elif qr_id in self.l2:
            return [ (self.qr_dimension*(self.l2.index(qr_id))+self.qr_dimension/2 +self.l2_gap_left_right[0]), 0]
        elif qr_id in self.l3:
            return [self.wall_dimension,(self.qr_dimension*(self.l3.index(qr_id))+self.qr_dimension/2 +self.l3_gap_top_down[0]) ]
        elif qr_id in self.l4:
            return [ (self.qr_dimension*(self.l4.index(qr_id))+self.qr_dimension/2 +self.l4_gap_left_right[0]), self.wall_dimension]
        else:
            return [0,0]
           
