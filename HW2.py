import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from numpy.linalg import pinv

def draw_robot():
    vx_p = 0
    vy_p = 0
    x = 0
    y = 0
    theta = 0
    x_p = x + np.cos(theta)
    y_p = y + np.sin(theta)
    dxpdx = 1
    dxpdy = 0
    dxpd0= -np.sin(theta)
    dypdx = 0
    dypdy = 1
    dypd0= np.cos(theta)
    robdiam = 0.05
    wheel_rad = 0.03

    Jac = np.zeros(shape=(2,3))
    print Jac
    Jac[0] = [dxpdx,dxpdy,dxpd0]
    Jac[1] = [dypdx,dypdy,dypd0]

    print Jac.shape
    print pen_vel(0,0)
    inv_Jac = pinv(Jac)
    print inv_Jac.shape
    q = np.dot(inv_Jac,pen_vel(0,0))
    print q

    jprime = np.zeros(shape=(3,2))
    jprime[0] = [np.sin(theta), np.sin(theta)]
    jprime[1] = [np.cos(theta), np.cos(theta)]
    jprime[2] = [robdiam, robdiam]

    inv_jprime = pinv(jprime)
    print inv_jprime.shape
    wheel_vec = (2/wheel_rad) *  np.dot(inv_jprime, q)
    print wheel_vec



def rob_angv(wheel_vec, wheel_rad, bot_diam):
    return wheel_rad * (wheel_vec[0] - wheel_vec[1]) / (2* bot_diam)

def pen_vel(x_p, y_p):
    return np.array([[x_p],[y_p]])
# def center_vel(Jac, pen_vel)

draw_robot()


class Robot:
    def init(self, x = 0, y = 0 , theta = 0, x_p = 0, y_p = 0):
        self.x = x
        self.y = y
        self.theta = theta
        self.x_p = 0
        self.y_p = 0

    def update_velocity(self, q):
        self.x += q[0]
        self.y += q[1]
        self.theta = q[2]