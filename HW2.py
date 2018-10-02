import cs1lib
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from numpy.linalg import pinv

# def draw_robot():
#     vx_p = 0
#     vy_p = 0
#     x = 0
#     y = 0
#     theta = 0
#     x_p = x + np.cos(theta)
#     y_p = y + np.sin(theta)
#     dxpdx = 1
#     dxpdy = 0
#     dxpd0= -np.sin(theta)
#     dypdx = 0
#     dypdy = 1
#     dypd0= np.cos(theta)
#     robdiam = 0.05
#     wheel_rad = 0.03

#     self.Jac = np.zeros(shape=(2,3))
#     print self.Jac
#     self.Jac[0] = [dxpdx,dxpdy,dxpd0]
#     self.Jac[1] = [dypdx,dypdy,dypd0]

#     print self.Jac.shape
#     print pen_vel(0,0)
#     inv_self.Jac = pinv(self.Jac)
#     print inv_self.Jac.shape
#     q = np.dot(inv_self.Jac,pen_vel(0,0))
#     print q

#     jprime = np.zeros(shape=(3,2))
#     jprime[0] = [np.sin(theta), np.sin(theta)]
#     jprime[1] = [np.cos(theta), np.cos(theta)]
#     jprime[2] = [robdiam, robdiam]

#     inv_jprime = pinv(jprime)

#     wheel_vec = (2/wheel_rad) *  np.dot(inv_jprime, q)




def rob_angv(wheel_vec, wheel_rad, bot_diam):
    return wheel_rad * (wheel_vec[0] - wheel_vec[1]) / (2* bot_diam)

def pen_vel(x_p, y_p):
    return np.array([[x_p],[y_p]])
# def center_vel(self.Jac, pen_vel)

draw_robot()


class Robot:
    def init(self, x=0, y=0 , theta=0, x_p=0, y_p=0, wheel_rad = 0.003, robdiam = 0.05):
        self.x = x
        self.y = y
        self.theta = theta
        self.x_p = 0
        self.y_p = 0
        self.wheel_rad = wheel_rad
        self.wheel_vel = 0
        self.robdiam = robdiam
        self.vx_p = 0
        self.vy_p = 0


        #Set up variables for the Jacobian
        dxpdx = 1
        dxpdy = 0
        dxpd0= -np.sin(self.theta)
        dypdx = 0
        dypdy = 1
        dypd0= np.cos(self.theta)
        self.Jac = np.zeros(shape=(2,3))
        self.Jac[0] = [dxpdx,dxpdy,dxpd0]
        self.Jac[1] = [dypdx,dypdy,dypd0]

    def update_position(self, q):
        self.x += q[0]
        self.y += q[1]
        self.theta = q[2]
        self.x_p += self.vx_p
        self.vy_p = 0

    def draw_robot(self):
        #draw pen
        draw_line(self.x, self.x + 5*np.cos(self.theta), self.y, self.y +5*np.cos(self.theta))
        #draw robot as pointmass
        draw_circle(self.x,self.y)
        dxpdx = 1
        dxpdy = 0
        dxpd0= -np.sin(self.theta)
        dypdx = 0
        dypdy = 1
        dypd0= np.cos(self.theta)

        self.Jac = np.zeros(shape=(2,3))

        self.Jac[0] = [dxpdx,dxpdy,dxpd0]
        self.Jac[1] = [dypdx,dypdy,dypd0]
        #inverse Jacobian using Moore-Penrose inverse
        inv_self.Jac = pinv(self.Jac)
        #Take the dot product of the inverse Jacobian and pen velocity to get
        #The velocity vector.
        q = np.dot(inv_self.Jac,pen_vel(self.x_p,self.y_p))
        #Inverse matrix to get the wheel velocities
        jprime = np.zeros(shape=(3,2))
        jprime[0] = [np.sin(theta), np.sin(theta)]
        jprime[1] = [np.cos(theta), np.cos(theta)]
        jprime[2] = [robdiam, robdiam]
        inv_jprime = pinv(jprime)

        #velocity vector for each of the wheels.
        wheel_vec = (2/wheel_rad) *  np.dot(inv_jprime, q)
        #def

        update_position(q)
        draw_line(self.x, self.x + 5*np.cos(self.theta), self.y, self.y +5*np.cos(self.theta))
        draw_circle(self.x + 5*np.cos(self.theta), self.y +5*np.cos(self.theta), 10)
        draw_circle(self.x,self.y, 10)