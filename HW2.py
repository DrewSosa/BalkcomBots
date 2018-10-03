from cs1lib import *
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from numpy.linalg import pinv

FRAMERATE = 1
WIDTH = 400
HEIGHT = 400
TIMESTEP = 1/FRAMERATE

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


class Robot:
    def __init__(self, x=0, y=0 , theta=np.pi/2, x_p=0, y_p=0, wheel_rad = 0.003, robdiam = 0.05):
        self.x = x
        self.y = y
        self.theta = theta
        self.omega = 0
        self.x_p = self.x + 10*np.cos(self.theta)
        self.y_p = self.y + 10*np.sin(self.theta)
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
        #Takes care of the angular velocity??
        self.theta = q[2]
        self.x_p += self.vx_p
        self.y_p += self.vy_p

    def ink(self):
        enable_stroke()
        draw_point(self.x_p, self.y_p)
    def draw_robot(self):
        clear()
        #draw pen
        draw_line(self.x, self.x + 5*np.cos(self.theta), self.y, self.y +5*np.cos(self.theta))
        #draw robot as pointmass
        draw_circle(self.x,self.y, 10)
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
        inv_Jac = pinv(self.Jac)
        #Take the dot product of the inverse Jacobian and pen velocity to get
        #The velocity vector.
        q = np.dot(inv_Jac,pen_vel(self.vx_p,self.vy_p))
        #Inverse matrix to get the wheel velocities
        jprime = np.zeros(shape=(3,2))
        jprime[0] = [np.sin(self.theta), np.sin(self.theta)]
        jprime[1] = [np.cos(self.theta), np.cos(self.theta)]
        jprime[2] = [self.robdiam, self.robdiam]
        inv_jprime = pinv(jprime)

        #velocity vector for each of the wheels.
        wheel_vec = (2/self.wheel_rad) *  np.dot(inv_jprime, q)

        if is_key_pressed("a"):
            self.vx_p = -0.2
        elif is_key_pressed("b"):
            self.vx_p = 0.2
        elif is_key_pressed("w"):
            self.vy_p = 0.2
        elif is_key_pressed("x"):
            self.vy_p = -0.2
        #def
        # if is_key_pressed("a") :
        #     self.v_x = -0.2
        # elif is_key_pressed("b"):
        #     self.x = 0.2
        # elif is_key_pressed("w"):
        #     self.y = 0.2
        # elif is_key_pressed("x"):
        #     self.y = -0.2
        # elif is_key_pressed("s"):
        #     self.x = 0
        #     self.y = 0

        #actually drawing the robot and pen.
        self.update_position(q)
        enable_stroke()
        self.ink()
        draw_line(self.x, self.y,  self.x_p, self.y_p)
        set_fill_color(1, 0, 0)
        draw_circle(self.x_p, self.y_p, 3)
        set_fill_color(0, 1, 0)
        draw_circle(self.x,self.y, 10)
        set_fill_color(1, 0, 0)




Robot = Robot(x=WIDTH/2, y=HEIGHT/2)
start_graphics(Robot.draw_robot, width=WIDTH, height=HEIGHT, framerate=60)