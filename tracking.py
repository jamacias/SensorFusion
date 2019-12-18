import pandas as pd 
import matplotlib.pyplot as plt
import numpy as np 
from Robot import Robot



# resampling function made by TA slightly modified to make it work wi all the other parameter
def resample(particle_x, particle_y, particle_phi,q):
    new_particle_x = particle_x.copy()
    new_particle_y = particle_y.copy()
    new_particle_phi = particle_phi.copy()
    
    for i in range(particle_x.shape[0]):
        u = np.random.rand()
        qtempsum=0
        for j in range(particle_x.shape[0]):
            qtempsum += q[j]
            if qtempsum >= u:
                new_particle_x[i] = particle_x[j]
                new_particle_y[i] = particle_y[j]
                new_particle_phi[i] = particle_phi[j]
                break
    return new_particle_x, new_particle_y, new_particle_phi


if __name__ == "__main__":
    
    # localize the robot
    robot = Robot()
    robot.calibrate()

    # initialize the state of the robot with the initial position that we already know
    robot.navigation_file("data/IMUNavigation.csv")
    imu_data = pd.read_csv("data/IMUNavigation.csv")
    print(imu_data.shape[0])
    
    P = 1 # initial estimation error covariance
    N = 10 # number of particle 

    R = 2 # measurement noise variance (ft^2) TODO fix with the correct value

    sqrt_Q = 1
    sqrt_R = np.sqrt(R)
    sqrt_P = np.sqrt(P)

    # initial position of the robot that we already know 
    # and we create particle around, also here P values is the same as TA implementation
    particle_x =  101 + sqrt_P*np.random.randn(N)
    particle_y = 60.5 + sqrt_P*np.random.randn(N)
    particle_phi = -90 + sqrt_P*np.random.randn(N)

    # error of each particle
    particle_z = np.zeros(N)

    for t in range(imu_data.shape[0]): # number of timestepe totally arbitrary just to try the algorithm
        vel_giro = robot.imu.gyroscope.get_measurement()[2] # get only the last one because the first two (x and y) are zero 
        phi_measured = robot.phi + ( vel_giro * robot.delta_t + sqrt_Q*np.random.randn() )  # phi + v_phi * delta_t
        
        # TODO all the variance are not correct but was more for a prove of concept of the all algorithm 
        for i in range(N):
            # for each particle goes trhough the dynamic model 
            particle_x[i], particle_y[i], particle_phi[i] = robot.dynamic_model(particle_x[i],particle_y[i],particle_phi[i]) +  sqrt_P*np.random.randn()
            # compute the error respect to the measured one 
            
            particle_z[i] = phi_measured - particle_phi[i] + sqrt_R*np.random.randn() 
        
        #computing the weighting, TA implementation 
        q = np.exp(-particle_z*particle_z/(2*R))
        qsum = np.sum(q)

        q = q/qsum

        #Resample
        
        #resampling using the TA implementation not super clear how it works exactly but it use the error z 
        particle_x, particle_y, particle_phi = resample(particle_x, particle_y, particle_phi,q)
        # compute the estimation of the va
        x_v = particle_x.mean()
        y_v = particle_y.mean()
        phi_v = particle_phi.mean()
        # update the phi_v value of the robot with the estimation of phi
        robot.phi = phi_v
        # plot the points every 100 steps
        if t%100 ==0:
            plt.scatter(x_v, y_v)
            plt.scatter(particle_x, particle_y)
            plt.pause(1)

        print("x_v: " ,x_v, "y_v: ",y_v, "phi_v:", phi_v%360 )



