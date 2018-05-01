# -*- coding: utf-8 -*-
import numpy as np
from itertools import product
from math import *
class Robot():
    def __init__(self,x,y,theta,length,width):
        self.x = x;
        self.y = y;
        self.theta = theta;
        self.length = length;
        self.width = width;
        
    
        
    def combinations(self,nums):
        return list(product(*((x, -x) for x in nums)))
    
    def getRectangle(self):
        list = self.combinations([1,1])
        result=[]
        for mul1,mul2 in list:
            Ox = (self.length/2) *mul1
            Oy = (self.width/2) *mul2     
            Rx = self.x + (Ox  * cos(self.theta)) - (Oy * sin(self.theta))
            Ry = self.y + (Ox  * sin(self.theta)) + (Oy * cos(self.theta))
            result.append([Rx,Ry])
        return Rectangle(np.array(result))

class Rectangle():
    def __init__(self,points):
        self.points = points
    """Single Axis Detection Theorem used to check 
       if a rectangle collides with another"""    
    def collides(self,rect):
        axis = np.array([self.get_point(3) - self.get_point(2)
        ,self.get_point(1) - self.get_point(3),rect.get_point(1) - self.get_point(2),
                       rect.get_point(2) - self.get_point(3)])
        all_points = np.array([self.get_point(2),self.get_point(3),self.get_point(1)
        ,self.get_point(0),rect.get_point(0),rect.get_point(1),rect.get_point(2),rect.get_point(3)])
        for i in range(len(axis)):
            min_r1 = 99999
            min_r2 = 99999
            max_r1 = -99999
            max_r2 = -99999
            for j in range(len(all_points)):
                proj = self.projected(all_points[j],axis[i])
                position = np.dot(proj,axis[i])
                if(j < 4):
                  
                    if(position < min_r1):
                        min_r1 = position
                    

                    if(position >  max_r1):
                        max_r1 = position
                    
                else:
                    
                    if(position < min_r2):
                        min_r2 = position
                    
                    if(position > max_r2):
                        max_r2 = position
                
            if(min_r2 > max_r1 or max_r2 < min_r1):
                return False
                
        return True
    
        
    def get_point(self,i):
        return np.asarray(self.points[i])
    
    def projected(self,p1,p2):
        return p2*(np.dot(p1,p2)/np.dot(p2,p2))
  