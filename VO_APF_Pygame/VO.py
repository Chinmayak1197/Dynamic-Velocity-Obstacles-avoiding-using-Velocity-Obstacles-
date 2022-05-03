from ast import Num
from dis import dis
from msilib.schema import Class
import sys, random, pygame
from traceback import print_tb
from turtle import pos
from time import sleep
from pygame.locals import *
import numpy as np
import math
from bresenham import bresenham

#REF for Pygame Random Movement : https://stackoverflow.com/questions/30015787/random-movement-pygame
# REF for VO: https://github.com/RahulSajnani/Robotics-Planning-and-Navigation


pygame.init()

WIDTH = 640 #game window width
HEIGHT = 480 #game window height
FPS = 60 #game's speeds
RED_VALUE = 255
Pixsize = 8
Pixsize_robot = 10
screen = pygame.display.set_mode((WIDTH, HEIGHT)) #set the game window


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
    
    def vo(self,obstacle,v_change):
        self.createposition()
        obstacle.createposition()
        radius = self.size + obstacle.size
        r = [obstacle.position[0] - self.position[0],obstacle.position[1] - self.position[1]]
        rel_v = [self.velocity[0] - obstacle.velocity[0],self.velocity[1] - obstacle.velocity[1]]
        
        dist = self.checkposdist(obstacle)


        v_new = [rel_v[0] + v_change[0],rel_v[1] + v_change[1]]

        
        norm = np.linalg.norm(v_new)
        if norm > 0:
            v_new = v_new / norm

        constraint = (np.dot(r, r) - np.dot(r, v_new)**2 >= radius**2) and (dist<=75)
        return constraint
    
    def rotate(self, vector, theta):
        '''
        Rotate vector by theta in 2D
        '''

        rotation_matrix = np.array([[np.cos(theta), -np.sin(theta)],
                                    [np.sin(theta),  np.cos(theta)]])

        return rotation_matrix @ vector
    
    
    def interrupt(self,obstacle,constraint):
        if constraint==False:
            return [0,0]
        else:
            while True:
                
                delta_init = [0,4]
                velocity = np.random.uniform(0, self.v_max)
                angle = np.random.uniform(-np.pi, np.pi)
                delta_v = self.rotate((delta_init * velocity), angle)
                constraint = self.vo(obstacle,delta_v)
                
                if constraint==False:
                    return delta_v
                
    def changedirection(self,delta_v):
        
        x = delta_v[0]*1+self.x
        y = delta_v[1]*1+self.y
        
        
        self.x = int(x)
        self.y = int(y)
        
        
        
        
        return(x,y)
        
        
                
                
    
        
 
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
    
    

cells = []
for i in range(5): #generate n cells
    Cell = cell()
    cells.append(Cell)
    
robot = Robot()


def mainloop():
    while True:
        for event in pygame.event.get():
            if event.type== QUIT: #if pressing the X, quit the progra
                pygame.quit() #stop pygame
                sys.exit() #stop the program
        screen.fill((0,0,0)) #clear the screen;
        for i in cells: #update all cells
            i.wander()
            i.draw()
            i.createVO()
            
            x,y = robot.robot_step([470,450])
            constraint = robot.vo(i,[0,0])
            delta_v = robot.interrupt(i,constraint)
            x,y = robot.changedirection(delta_v)
            sleep(0.05)
        
        robot.draw(x,y)
            
                
        
        
    
        
        
    
        pygame.display.update() #update display
        pygame.time.Clock().tick(FPS) #limit FPS

mainloop()