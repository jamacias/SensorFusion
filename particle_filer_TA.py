#%%%
# Exercise 9
# Based on Dan Simon Example 15.1
# Particle filter example, adapted from Gordon, Salmond, and Smith paper.
#
#matplotlib auto
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
from scipy.linalg import expm
import seaborn as sns
import scipy.linalg as sla
import numba as nb
sns.set()


#%% Discrete nonlinear 
@nb.njit
def f(x,k):
    return 0.5 * x + 25 * x / (1 + x*x) + 8 * np.cos(1.2*(k-1))

# def f(x,params):
#     xdot = np.array([x[1],
#         0.5*params['rho0']*np.exp(-x[0]/params['k'])*x[1]*x[1]*x[2] - params['g'],
#         0])
#     return xdot

@nb.njit
def h(x):
    h = x*x / 20
    
    return h

@nb.njit
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

# %%
R = 1 # measurement noise variance (ft^2)
Q = 1 # process noise covariance

P = 2 # initial estimation error covariance
tf = 50 #simulation length (seconds)
N = 100
 #Number of particles

sqrt_Q = 1
sqrt_P = np.sqrt(P)
sqrt_R = np.sqrt(R)


x = 0.1 # initial state
xhat_PF = x # initial state estimate
xhat_EKF = x # initial state estimate

#

#Initialize the particle filter
particle_xhat = np.zeros(N)
particle_z = np.zeros(N)
particle_xhat = x + sqrt_P*np.random.randn(N)

#set random seed
np.random.seed(1)

#Simulation time
t = np.arange(0,tf,1)


#for history
xhat_history_PF = np.zeros(t.shape[0])
xhat_history_EKF = np.zeros(t.shape[0])
P_history_EKF = np.zeros(t.shape[0])
x_history = np.zeros(t.shape[0])
xhat_history_PF[0] = xhat_PF
xhat_history_EKF[0] = xhat_EKF
x_history[0] = x
P_history_EKF[0] = P
#%%
#Main loop
for k in range(1,t.shape[0]):
    print('.')
    #simulate the actual state at next measurement sampling
    x = f(x,k) + sqrt_Q*np.random.randn()
    
    x_history[k] = x 
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
    xhat_history_EKF[k] = xhat_EKF







# %%
#plt.rc('text', usetex=True)
plt.rc('font', family='serif')

#plotting
plt.semilogy(t, abs(x_history - xhat_history_PF),'b',label='x_pf', linewidth=0.5) 
plt.semilogy(t, abs(x_history - xhat_history_EKF), 'r',label='x_ekf', linewidth=0.5) 
plt.xlabel('k')
plt.ylabel('x')
plt.legend()

plt.figure()
plt.plot(t, x_history,'*k')
plt.plot(t, xhat_history_EKF,':r',label='x_ekf',linewidth=0.5)
plt.fill_between(t,xhat_history_EKF-2*np.sqrt(P_history_EKF),xhat_history_EKF+2*np.sqrt(P_history_EKF), color='r', alpha=0.1)
plt.plot(t, xhat_history_PF,':b',label='x_pf',linewidth=0.5)
plt.xlabel('k')
plt.ylabel('x')
plt.legend()
plt.show()

# %%
