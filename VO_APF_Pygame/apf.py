from ast import Num
from dis import dis
# from msilib.schema import Class
import sys, random, pygame
from traceback import print_tb
from turtle import pos
from time import sleep
from pygame.locals import *
import numpy as np
import math
from bresenham import bresenham

#REF for Pygame Random Movement : https://stackoverflow.com/questions/30015787/random-movement-pygame
#REF for APF: https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathPlanning/PotentialFieldPlanning/potential_field_planning.py

pygame.init()

WIDTH = 640 #game window width
HEIGHT = 480 #game window height
FPS = 25 #game's speeds
RED_VALUE = 255
Pixsize = 8
Pixsize_robot = 10
screen = pygame.display.set_mode((WIDTH, HEIGHT)) #set the game window

pygame.display.set_caption("Artificial Potencial Field")

class cell:
    def __init__(self):
        self.x = random.randrange(10, WIDTH-10) #x position
        self.y = random.randrange(10, HEIGHT-10) #y position
        self.speed = random.randrange(2,5) #cell speed
        self.move = [None, None] #realtive x and y coordinates to move to
        self.direction = None #movement direction
        self.velocity = None
        self.size = 8
        self.position = None
        
    def createposition(self):
        self.position =  [self.x,self.y]
        return self.position

    def draw(self):
        pygame.draw.rect(screen, (255,255,255), (self.x,self.y,Pixsize,Pixsize),0) #draw the cell
        
        
    def createVO(self):
        
        velocity = self.getvelocity()
        
        self.velocity = velocity
        position = [self.x,self.y]
        
        dot = np.dot(velocity,position)
        
        return dot


    def wander(self):
        directions = {"S":((-1,2),(1,self.speed)),"SW":((-self.speed,-1),(1,self.speed)),"W":((-self.speed,-1),(-1,2)),"NW":((-self.speed,-1),(-self.speed,-1)),"N":((-1,2),(-self.speed,-1)),"NE":((1,self.speed),(-self.speed,-1)),"E":((1,self.speed),(-1,2)),"SE":((1,self.speed),(1,self.speed))} #((min x, max x)(min y, max y))
        directionsName = ("S","SW","W","NW","N","NE","E","SE") #possible directions
        if random.randrange(0,5) == 2: #move about once every 5 frames
            if self.direction == None: #if no direction is set, set a random one
                self.direction = random.choice(directionsName)
            else:
                a = directionsName.index(self.direction) #get the index of direction in directions list
                b = random.randrange(a-1,a+2) #set the direction to be the same, or one next to the current direction
                if b > len(directionsName)-1: #if direction index is outside the list, move back to the start
                    b = 0
                self.direction = directionsName[b]
            self.move[0] = random.randrange(directions[self.direction][0][0],directions[self.direction][0][1]) #change relative x to a random number between min x and max x
            self.move[1] = random.randrange(directions[self.direction][1][0],directions[self.direction][1][1]) #change relative y to a random number between min y and max y
        if self.x < 5 or self.x > WIDTH - 5 or self.y < 5 or self.y > HEIGHT - 5: #if cell is near the border of the screen, change direction
            if self.x < 5:
                self.direction = "E"
            elif self.x > WIDTH - 5:
                self.direction = "W"
            elif self.y < 5:
                self.direction = "S"
            elif self.y > HEIGHT - 5:
                self.direction = "N"
            self.move[0] = random.randrange(directions[self.direction][0][0],directions[self.direction][0][1]) #change relative x to a random number between min x and max x
            self.move[1] = random.randrange(directions[self.direction][1][0],directions[self.direction][1][1]) #change relative x to a random number between min x and max x
        if self.move[0] != None: #add the relative coordinates to the cells coordinates
            self.x += self.move[0]
            self.y += self.move[1]
            
    def getvelocity(self):
        direction = self.direction
        speed = self.speed
        speed_sqrt = math.sqrt(speed)
        velocity = [0,0]
        
        if direction == "N":
            velocity = [0,speed]
        if direction == "W":
            velocity = [-speed,0]
        if direction == "S":
            velocity = [0,-speed]
        if direction == "E":
            velocity = [speed,0]
        if direction == "NW":
            velocity = [-speed_sqrt,speed_sqrt]
        if direction == "NE":
            velocity = [speed_sqrt,speed_sqrt]
        if direction == "SE":
            velocity = [speed_sqrt,-speed_sqrt]
        if direction == "SW":
            velocity = [-speed_sqrt,-speed_sqrt]      
        
        
        return velocity
    
    def getxy(self):
        return [self.x,self.y]
            
class Robot:
    def __init__(self):
        self.x = 45 #x position
        self.y = 50 #y position
        self.speed = random.randrange(2,5) #cell speed
        self.move = [None, None] #realtive x and y coordinates to move to
        self.direction = None #movement direction
        self.size = 10
        self.agent_path= None
        self.velocity = None
        self.v_max = [2,3]
        
    def createposition(self):
        self.position =  [self.x,self.y]
        return self.position

        
        
    def getvelocity(self):
        direction = self.direction
        speed = self.speed
        speed_sqrt = math.sqrt(speed)
        velocity = [0,0]
        
        if direction == "N":
            velocity = [0,speed]
        if direction == "W":
            velocity = [-speed,0]
        if direction == "S":
            velocity = [0,-speed]
        if direction == "E":
            velocity = [speed,0]
        if direction == "NW":
            velocity = [-speed_sqrt,speed_sqrt]
        if direction == "NE":
            velocity = [speed_sqrt,speed_sqrt]
        if direction == "SE":
            velocity = [speed_sqrt,-speed_sqrt]
        if direction == "SW":
            velocity = [-speed_sqrt,-speed_sqrt]     
            
            
        self.velocity = velocity 
        
        
        return velocity
    
    def getxy(self):
        return [self.x,self.y]
    
    def checkposdist(self,obstacle):
        robot_pos = self.createposition()
        obstalce_pos = obstacle.createposition()
        
        p1 = np.array(robot_pos)
        p2 = np.array(obstalce_pos)
        
        dist = np.linalg.norm(p1-p2)
        
        return dist
    

    def draw(self,x,y):
        pygame.draw.rect(screen, (255,0,0), (x,y,Pixsize_robot,Pixsize_robot),0) #draw the cell
        
    def robot_step(self,goal):
        
        self.getvelocity()
        
        x_current  = self.x
        y_current = self.y
        
        x_desired = goal[0]
        y_desired = goal[1]
        
        
        traj = list(bresenham(x_current, y_current, x_desired, y_desired))
        
        if len(traj)>2:
            x = traj[1][0]
            y = traj[1][1]
            
        else:
            x = x_desired
            y = y_desired
            
        self.velocity = [x-self.x,y-self.y]
            
        self.x = x
        self.y = y

        
        return x,y
    



#Artificial Potencial Field 
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Apr 27 00:12:34 2022

@author: chinmaya
"""

from collections import deque
import matplotlib.pyplot as plt

# Parameters
KP = 5.0  # attractive potential gain
ETA = 100.0  # repulsive potential gain
AREA_WIDTH = 30.0  # potential area width [m]
# the number of previous positions used to check oscillations
OSCILLATIONS_DETECTION_LENGTH = 3

def calc_attractive_potential(x, y, gx, gy):
    return 0.5 * KP * np.hypot(x - gx, y - gy)


def calc_repulsive_potential(x, y, ox, oy, rr):
    # search nearest obstacle
    minid = -1
    dmin = float("inf")
    for i, _ in enumerate(ox):
        d = np.hypot(x - ox[i], y - oy[i])
        if dmin >= d:
            dmin = d
            minid = i

    # calc repulsive potential
    dq = np.hypot(x - ox[minid], y - oy[minid])

    if dq <= rr:
        if dq <= 0.1:
            dq = 0.1

        return 0.5 * ETA * (1.0 / dq - 1.0 / rr) ** 2
    else:
        return 0.0
    
def calc_potential_field(gx, gy, ox, oy, reso, rr, sx, sy):
    minx = min(min(ox), sx, gx) - AREA_WIDTH / 2.0
    miny = min(min(oy), sy, gy) - AREA_WIDTH / 2.0
    maxx = max(max(ox), sx, gx) + AREA_WIDTH / 2.0
    maxy = max(max(oy), sy, gy) + AREA_WIDTH / 2.0
    xw = int(round((maxx - minx) / reso))
    yw = int(round((maxy - miny) / reso))

    # calc each potential
    pmap = [[0.0 for i in range(yw)] for i in range(xw)]

    for ix in range(xw):
        x = ix * reso + minx

        for iy in range(yw):
            y = iy * reso + miny
            ug = calc_attractive_potential(x, y, gx, gy)
            uo = calc_repulsive_potential(x, y, ox, oy, rr)
            uf = ug + uo
            pmap[ix][iy] = uf

    return pmap, minx, miny

def get_motion_model():
    # dx, dy
    motion = [[1, 0],
              [0, 1],
              [-1, 0],
              [0, -1],
              [-1, -1],
              [-1, 1],
              [1, -1],
              [1, 1]]

    return motion

def oscillations_detection(previous_ids, ix, iy):
    previous_ids.append((ix, iy))

    if (len(previous_ids) > OSCILLATIONS_DETECTION_LENGTH):
        previous_ids.popleft()

    # check if contains any duplicates by copying into a set
    previous_ids_set = set()
    for index in previous_ids:
        if index in previous_ids_set:
            return True
        else:
            previous_ids_set.add(index)
    return False


def potential_field_planning(sx, sy, gx, gy, ox, oy, reso, rr):

    # calc potential field
    pmap, minx, miny = calc_potential_field(gx, gy, ox, oy, reso, rr, sx, sy)

    # search path
    d = np.hypot(sx - gx, sy - gy)
    ix = round((sx - minx) / reso)
    iy = round((sy - miny) / reso)
    rx, ry = [sx], [sy]
    motion = get_motion_model()
    previous_ids = deque()

    while d >= reso:
        minp = float("inf")
        minix, miniy = -1, -1
        for i, _ in enumerate(motion):
            inx = int(ix + motion[i][0])
            iny = int(iy + motion[i][1])
            if inx >= len(pmap) or iny >= len(pmap[0]) or inx < 0 or iny < 0:
                p = float("inf")  # outside area
                # print("outside potential!")
            else:
                p = pmap[inx][iny]
            if minp > p:
                minp = p
                minix = inx
                miniy = iny
        ix = minix
        iy = miniy
        xp = ix * reso + minx
        yp = iy * reso + miny
        d = np.hypot(gx - xp, gy - yp)
        rx.append(xp)
        ry.append(yp)

        if (oscillations_detection(previous_ids, ix, iy)):
            break


    return rx[0], ry[0]


def main():
    print("potential_field_planning start")





if __name__ == '__main__':
    print(__file__ + " start!!")
    main()
    print(__file__ + " Done!!")
   

cells = []
for i in range(5): #generate n cells
    Cell = cell()
    cells.append(Cell)
    
robot = Robot()

ox = []
oy= []
def mainloop():
    while True:
        ox = []
        oy= []
        for event in pygame.event.get():
            if event.type== QUIT: #if pressing the X, quit the progra
                pygame.quit() #stop pygame
                sys.exit() #stop the program
        screen.fill((0,0,0)) #clear the screen;
        ox.clear()
        oy.clear()
        for i in cells: #update all cells
            i.wander()
            i.draw()
            i.createVO()
            ox.append(i.createposition()[0])
            oy.append(i.createposition()[1])
            
            
            x,y = robot.robot_step([470,450])
            sx = x  # start x position [m]
            sy = y  # start y positon [m]
            gx = 470  # goal x position [m]
            gy = 450  # goal y position [m]
            grid_size = 1.45  # potential grid size [m]
            robot_radius = 2.0  # robot radius [m]
            # ox = [15.0, 5.0, 20.0, 25.0]  # obstacle x position list [m]
            # oy = [25.0, 15.0, 26.0, 25.0]  # obstacle y position list [m]
            
            x,y = potential_field_planning(sx, sy, gx, gy, ox, oy, grid_size, robot_radius)
            x = x
            y = y
        # # print(ox,oy)
        # # constraint = robot.vo(i,[0,0])
        # # delta_v = robot.interrupt(i,constraint)
        # # x,y = robot.changedirection(delta_v)
        # # sleep(0.05)
        # sx = x  # start x position [m]
        # sy = y  # start y positon [m]
        # gx = 470  # goal x position [m]
        # gy = 450  # goal y position [m]
        # grid_size = 1.0  # potential grid size [m]
        # robot_radius = 2.0  # robot radius [m]
        # ox = [15.0, 5.0, 20.0, 25.0]  # obstacle x position list [m]
        # oy = [25.0, 15.0, 26.0, 25.0]  # obstacle y position list [m]
        # x,y = potential_field_planning(sx, sy, gx, gy, ox, oy, grid_size, robot_radius)
        # print(x)
        # for i in range(len(x)):
        #     robot.draw(x[i], y[i])
        robot.draw(x,y)
            
                

    
        
        
    
        pygame.display.update() #update display
        pygame.time.Clock().tick(FPS) #limit FPS

mainloop()