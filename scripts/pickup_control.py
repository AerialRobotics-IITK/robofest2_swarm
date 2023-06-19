import math
import time
import numpy as np
import rospy
from geometry_msgs.msg import Point


pi = 3.1415
rospy.init_node('pickup_control', anonymous=True)

class pickup_control:
    def __init__(
        self,
        queue1=None,
        queue2=None,
        kp=[4, 4, 2.7],
        kd=[5, 5, 2.15],
        ki=[0.05, 0.05, 1.1],
        eqb_thrust=1550,
    ):
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.tol = 10
        self.ball_pos = [-1.0, -1.0, -1.0]
        self.prev_time = [0.0, 0.0, 0.0]
        self.error_tol = 0.01
        self.prev_error = [0.0, 0.0, 0.0]
        self.e_i = [0.0, 0.0, 0]
        self.e_d = [0.0, 0.0, 0.0]
        self.vel = np.array([0.0, 0.0, 0.0])
        self.curr_pos = [0.0, 0.0, 0.0]
        self.curr_attitude = [0, 0, 0, 0]
        self.equilibrium_thrust = eqb_thrust
        self.b3d = np.array([0.0, 0.0, 0.0])
        self.re3 = np.array([0.0, 0.0, 0.0])
        self.prev_pose = np.array([-1000,-1000,-1000])
        self.timer_start = None


    def listener(self, data):
        self.ball_pos[0] = data.x
        self.ball_pos[1] = data.y
        self.ball_pos[2] = data.z
        

    def listen(self):
        rospy.Subscriber("/color_detector/color_detected", Point, self.listener)
        rospy.spin()


    def talker_pub(self, roll, pitch, yaw, thrust):

        pitch = 1500 + pitch * (1800 / pi)
        pitch = self.clip(pitch, 1400, 1600)
        roll = 1500 + roll * (1800 / pi)
        roll = self.clip(roll, 1400, 1600)
        thrust = self.clip(thrust, 1480, 2099)
        yaw = 1510
        
        self.talker.set_RPY_THR(int(roll), int(pitch), int(yaw), int(thrust))

    def calc_error(self, i, error):

        curr_time = time.time()
        dt = 0.0
        if self.prev_time[i] != 0.0:
            dt = curr_time - self.prev_time[i]
        de = error - self.prev_error[i]
        e_p = error
        if self.prev_time[i] != 0:
            self.e_i[i] += error * dt
        self.e_d[i] = 0
        if dt > 0:
            self.e_d[i] = de / dt
        self.prev_time[i] = curr_time
        self.prev_error[i] = error
        return (
            (self.kp[i] * e_p) + (self.kd[i] * self.e_d[i]) +
            (self.ki[i] * self.e_i[i])
        )

    def pos_change(self, targ_pos=([0, 0, 0])):
        error =  targ_pos - #current wala
        # this code takes in input the position of the drone and and the relative position of the ball on the screne 
        # calculates the error in the calculations Ball pos- targ pos
        # if z = -1 it doesnt publish
        # else it publishes it from the publisher

    def autopilot(self, targ_pos, duration):

        self.start = time.time()
        self.listener()
        start = time.time()
        while time.time() - start < duration:
            self.listener()
            if (max([abs(targ_pos[0] - self.curr_pos[0]),abs(targ_pos[1] - self.curr_pos[1]),])) < self.tol:
                print(f"Drone{self.drone_num} Position Reached!!")
            self.pos_change(targ_pos)


iris = pickup_control()
iris.listen()