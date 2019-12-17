import pandas as pd 
import matplotlib.pyplot as plt
import numpy as np 
from Robot import Robot


def h(x):
    h = x*x / 20
    return h

def resample(particle_xhat,q):
    new_particle_xhat = particle_xhat.copy()
    for i in range(particle_xhat.shape[0]):
        u = np.random.rand()
        qtempsum=0
        for j in range(particle_xhat.shape[0]):
            qtempsum += q[j]
            if qtempsum >= u:
                new_particle_xhat[i] = particle_xhat[j]
                #Use roughening to prevent sample impoverishment.
                #TODO: to be implemented
                break
    return new_particle_xhat


if __name__ == "__main__":
    
    # localize the robot
    robot = Robot()
    robot.calibrate()

    # initialize the state of the robot with the initial position that we already know
    robot.navigation_file("data/IMUNavigation.csv")
    robot.initialize_state(101,60.5,np.pi/2.0)

    P = 1 # initial estimation error covariance
    N = 100 # number of particle 

    R = 1 # measurement noise variance (ft^2) TODO fix with the correct value

    sqrt_R = np.sqrt(R)
    sqrt_P = np.sqrt(P)

    particle_x =  101 + sqrt_P*np.random.randn(N)
    particle_y = 60.5 + sqrt_P*np.random.randn(N)
    particle_phi = -90 + sqrt_P*np.random.randn(N)

    particle_z = np.zeros(N)

    for t in range(2000): # number of timestepe totally arbitrary just to try the algorithm
        phi_measured = robot.phi + robot.imu.gyroscope.get_measurement()[2] * robot.delta_t # phi + v_phi * delta_t
    
        for i in range(N):
            particle_x[i], particle_y[i], particle_phi[i] = robot.dynamic_model(particle_x[i],particle_y[i],particle_phi[i])
            particle_z[i] = phi_measured - h(particle_phi[i]) # TODO understand what h does 
        robot.phi = phi_measured
        
        #computing the weighting
        q = np.exp(-particle_z*particle_z/(2*R))
        qsum = np.sum(q)

        q = q/qsum

        #Resample
        
        #TODO implement that part 
        #particle_xhat = resample(particle_xhat,q)

        x_v = particle_x.mean()
        y_v = particle_y.mean()
        phi_v = particle_phi.mean()

        plt.scatter(x_v, y_v)
        #plt.pause(0.000001)
        #plt.clf()

        print("x_v: " ,x_v, "y_v: ",y_v, "phi_v:", phi_v )

    plt.show()


        
        
    # imu measurement


    """ for k in range(200):
        #step of the dynamic model 
        robot.dynamic_model()
        x = robot.x
        y = robot.y
        phi = robot.phi

        #get the measurement
        y = h(x) + sqrt_Q*np.random.randn()

        #Simulate the discrete-time part of the particle filter (time-update)
        for i in range(N):
            particle_xhat[i] = f(particle_xhat[i],k) + sqrt_Q*np.random.randn()
            
            particle_z[i] = y - h(particle_xhat[i])
            # particle_z[i] = y - h(particle_xhat[i,:],params)
        
        # Extended Kalman filter
        F = 0.5 + 25 * (1 - xhat_EKF**2) / (1 + xhat_EKF**2)**2
        P = F * P * F + Q
        H = xhat_EKF / 10
        K = P * H / (H * P * H + R)
        xhat_EKF = f(xhat_EKF,k)
        xhat_EKF = xhat_EKF + K * (y - h(xhat_EKF))
        P = (1 - K * H) * P
        P_history_EKF[k] = P
        # Note that we need to scale all of the q(i) probabilities in a way
        # that does not change their relative magnitudes.
        # Otherwise all of the q(i) elements will be zero because of the
        # large value of the exponential.
        

        #computing the weighting
        q = np.exp(-particle_z*particle_z/(2*R))
        qsum = np.sum(q)

        #Normalize the likelihood of each a priori estimate
        q = q/qsum

        #Resample
        particle_xhat = resample(particle_xhat,q)
        

        #The particle filter estimate is the mean of the particles.
        xhat_PF = particle_xhat.mean()
        xhat_history_PF[k] = xhat_PF
        xhat_history_EKF[k] = xhat_EKF """