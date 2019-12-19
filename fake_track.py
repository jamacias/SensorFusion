import numpy as np 
import matplotlib.pyplot as plt

ca_det = np.load("cam_detection.npy")
x = []
y = []
for p in ca_det:
    if p[0]>0 and p[1]>0:
        x.append(p[0])
        y.append(p[1])
plt.scatter(x,y,'go-', linewidth=2)
plt.show()