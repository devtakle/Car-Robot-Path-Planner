# -*- coding: utf-8 -*-
from itertools import product
import sys, random, math, pygame
import numpy as np
from pygame.locals import *
from math import *
import matplotlib.pyplot as plt
class Tree():
    tree_map = {}
#    def _init_(self):
#        self.tree_map = {}
#        #self.tree_map[tuple(point)]
#        print len(self.tree_map.keys())
        
    def insert(self,current, parent):
        self.tree_map[tuple(current)] = tuple(parent)
   
    def euclid_dist(self,p1, p2):
        return sqrt((p1[0]-p2[0])*(p1[0]-p2[0])+(p1[1]-p2[1])*(p1[1]-p2[1])+(p1[2]-p2[2])*(p1[2]-p2[2]))            
    
    def get_closest(self, point):
        closest_point =  []
        opt_distance = 99999
        point_arr = np.asarray(point)
        for key in self.tree_map.keys():
            key_arr = np.asarray(key)
            distance = self.euclid_dist(key_arr, point_arr)
            if(distance < opt_distance):
                closest_point = key_arr
                opt_distance = distance
        
        return closest_point
    def find_path(self,point,path):
        if(len(point)== 0):
            return path
        else:
            parent = np.asarray(self.tree_map[tuple(point)])
            path.append(parent)
            return self.find_path(parent,path)
        
    def get_parent(self,point):
        return np.asarray(self.tree_map[tuple(point)])
