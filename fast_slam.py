"""
    This is the main file that starts the simulation.
    It contains a "World" object specifying the world settings, and a set of particles.
    Every time the robot moves, it generates random observations usded to update the particle sets
    to estimate the robot path as well as the landmarks locations.
"""
import sys
import random
import math
from copy import deepcopy
from world import World
from particle import Particle
from ekf import EKF
import numpy as np

class FastSlam(object):
    """
    Main class that implements the FastSLAM1.0 algorithm
    """
    def __init__(self, x, y, orien):
        self.world = World()
        self.ekf = EKF(x, y, orien)
        # self.particles = [Particle(x, y, random.random()* 2.*math.pi) for i in range(particle_size)]
        self.robot = Particle(x, y, orien, is_robot=True)
        self.gamma = 0

    def run_simulation(self):
        while True:
            for event in self.world.pygame.event.get():
                self.world.test_end(event)
            self.world.clear()
            key_pressed = self.world.pygame.key.get_pressed()
            if self.world.move_forward(key_pressed):
                self.move_forward(2)
            else : 
                self.slow_down()
            if self.world.turn_left(key_pressed):
                self.turn_left(5)
                self.gamma = 5*(math.pi/180)
            elif self.world.turn_right(key_pressed):
                self.turn_right(5)
                self.gamma = -5*(math.pi/180)
            else:
                self.gamma = 0
            
            # print("---------------------------")
            # print("Momentum: ", self.robot.momentum)
            # print("x, y : ", self.robot.pos_x, self.robot.pos_y)
            # print("yaw : ", self.robot.orientation)

            measured_x = self.robot.pos_x + np.random.normal(loc=0.0, scale=0.05)
            measured_y = self.robot.pos_y + np.random.normal(loc=0.0, scale=0.05)
            measured_yaw = self.robot.orientation + np.random.normal(loc=0.0, scale=0.002)
            measured_vel = self.robot.momentum + np.random.normal(loc=0.0, scale=0.02)
            measured_gamma = self.gamma# + np.random.normal(loc=0.0, scale=0.05)

            measured_yaw = self.process_yaw(measured_yaw)

            state_vec = [measured_x, measured_y, measured_yaw, measured_vel, measured_gamma]
            filtered_state = self.ekf.update(state_vec)

            print("Error x : ", abs(filtered_state[0] - self.robot.pos_x))
            print("Error y : ", abs(filtered_state[1] - self.robot.pos_y))
            # print("x, y : ", filtered_state[0], filtered_state[1])
            # print("yaw: ", filtered_state[2])
            print("------------------------")

            self.world.render(self.robot, filtered_state)

    def process_yaw(self, yaw):
        pi = 3.14159
        if yaw > pi and yaw < pi*2:
            return yaw - 2*pi
        elif yaw > 2*pi:
            return yaw - 2*pi
        return yaw

    def move_forward(self, step):
        self.robot.forward(step)
    
    def slow_down(self):
        self.robot.slow_down()

    def turn_left(self, angle):
        self.robot.turn_left(angle)

    def turn_right(self, angle):
        self.robot.turn_right(angle)

if __name__=="__main__":
    random.seed(5)
    simulator = FastSlam(80, 140, 0)
    simulator.run_simulation()
