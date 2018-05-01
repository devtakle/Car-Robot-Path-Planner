# -*- coding: utf-8 -*-
from itertools import product
import sys, random, math, pygame
import numpy as np
from pygame.locals import *
from math import *
from tree import Tree
from robot import Robot, Rectangle
"""PyGame window"""
pygame.init()
XDIM = 250
YDIM = 250
WINSIZE = [XDIM, YDIM]
screen = pygame.display.set_mode(WINSIZE)
black = (  0,   0,   0)
white = (255, 255, 255)
red   = (255,   0,   0)
blue  = (0,   0,   255)
screen.fill(white)
pygame.display.update()
"""PARAMETERS"""
speed = 12
length = 15
width = 10
initial_state = np.array([200,25,math.radians(60)])
final_state = np.array([50,200,math.radians(45)])
dt = 0.3
EPOCH  = 5000
"""OBSTACLES"""
obs1 = pygame.draw.rect(screen, black, [0, 100, 100, 50])
obs2 = pygame.draw.rect(screen, black, [170, 150, 50, 50])
obs3 = pygame.draw.rect(screen, black, [70, 50, 10, 25])
obs = [obs1,obs2,obs3]

pygame.display.update()
clock = pygame.time.Clock()

"""Control System Constraints"""
def derivative(state, fi):
    xDot = speed*cos(state[2])*cos(fi)
    yDot = speed*sin(state[2])*cos(fi)
    thetaDot = (0.8)*sin(fi)  #speed/length * sin(fi)
    return np.array([xDot,yDot,thetaDot])

"""Euclidean Distance between states"""
def euclid_dist(p1,p2):
    #+(p1[2]-p2[2])*(p1[2]-p2[2])
    return sqrt((p1[0]-p2[0])*(p1[0]-p2[0])+(p1[1]-p2[1])*(p1[1]-p2[1]))

"""Runge Kutta 4th Order Method to get approximate next state"""
def runge_kutta(point, fi):
    k1 = derivative(point,fi)
    k2 = derivative((k1/2)+point,fi)
    k3 = derivative((k2/2)+point,fi)
    k4 = derivative(k3+point,fi)
    increment =  ( k1 + 2*k2+ + 2*k3 + k4 )*(dt/6)
    result = point + increment
    return result

"""Draw the car given the state"""
def draw_car(state,fill = 0):
    car_init = Robot(state[0],state[1],state[2],15,10)
    car_rec_init = car_init.getRectangle()
    pygame.draw.polygon(screen,blue,[car_rec_init.get_point(1), car_rec_init.get_point(0), car_rec_init.get_point(2), car_rec_init.get_point(3)],fill)
    pygame.display.update()

"""Extend from input state to nearest state in the tree"""
def extend(state, tree):
    nearest_state = tree.get_closest(state)
    #robot = Robot(nearest_state[0],nearest_state[1],nearest_state[2],length,width)
    opt_state,opt_dist = opt_input(nearest_state, state)
    robot = Robot(opt_state[0], opt_state[1], opt_state[2], 8, 6)
    rect_robot = robot.getRectangle()
    check = False
    for ob in obs:
        ob_rect = Rectangle(np.array([np.asarray(ob.bottomleft),np.asarray(ob.topleft),np.asarray(ob.topright),np.asarray(ob.bottomright)]))
        if(rect_robot.collides(ob_rect)):
            check = True
    if(check == False):
        pygame.draw.line(screen,black,(nearest_state[0],nearest_state[1]),(opt_state[0],opt_state[1]),1)
        pygame.display.update()
        tree.insert(opt_state, nearest_state)

        return opt_state

"""Try a range of values of fi to get the optimum next state"""
def opt_input(from_state, to_state):
    opt_dist = 999999
    opt_state = []
    opt_angle = 0
    for s_angle in range (-45,45,10):
        next_state = runge_kutta(from_state,s_angle)
        temp_dist = euclid_dist(next_state,to_state)
        if (temp_dist < opt_dist):
            opt_dist = temp_dist
            opt_state = next_state
            opt_angle = s_angle
    return opt_state,opt_dist

"""draw a path backtracking through the tree"""
def draw_path(goal_state,tree):
    parent = tree.get_parent(goal_state)
    i = 1
    while(len(parent) > 0):
        pygame.draw.line(screen,red,(goal_state[0],goal_state[1]),(parent[0],parent[1]),2)
        if(i%25 == 0):
            draw_car(goal_state,1)
        i+=1
        pygame.display.update()
        goal_state = parent
        parent = tree.get_parent(goal_state)


def planner():
    tree = Tree()
    draw_car(initial_state)

    tree.insert(initial_state, np.array([]))
#    pygame.draw.line(screen,blue,(point[0],point[1]),(final_state[0],final_state[1]),2)
    pygame.display.update()
    for i in range (EPOCH):
         rand = np.array([random.uniform(15,235), random.uniform(15,235), random.uniform(math.radians(0),math.radians(360))])
         opt_state = extend(rand,tree)

         if(opt_state is not None and euclid_dist(opt_state,final_state) < 5):
             print "reached at ",i,"th iteration"
             goal_state = opt_state
             pygame.display.update()
             draw_path(goal_state,tree)
             break
    draw_car(final_state)
    """save result to an image"""
    string = np.array_str(initial_state)+ ' to ' +np.array_str(final_state)+'.png'
    pygame.image.save(screen, string)
    print "end"
def main():
    planner()

if __name__ == '__main__':
    main()
    running = True
    while running:
        clock.tick(60)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

    pygame.quit()
