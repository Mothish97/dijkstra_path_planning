# -*- coding: utf-8 -*-
"""
Created on Sun Mar  5 18:39:16 2023

@author: mothi
"""


import numpy as np
import heapq
import time
import cv2
import pygame
from collections import defaultdict
from tqdm import tqdm


"""
----------------------------------------------------------------
 Creating obstacles in the map
---------------------------------------------------------------- 
"""

def get_slope_const(p1, p2):
    try:
        slope = (p2[1]-p1[1])/(p2[0] - p1[0])
        constant = (p2[1] - slope * p2[0])
        return slope, constant
    except:
        return p1[1], p1[1]
    
def getObstacleCoord(mapWidth,mapHeight):
    coords=[]
    for i in range(mapWidth + 1): 
        for j in range(mapHeight + 1): 
            coords.append((i,j)) 
            
    obstacles= []
    clearance = 5 
    
    Hexa_pt1 = (235.05, 162.5)
    Hexa_pt2 = (300, 200)
    Hexa_pt3 = (364.95, 162.5) 
    Hexa_pt4 = (364.95, 87.5)
    Hexa_pt5 = (300, 50)
    Hexa_pt6 = (235.05, 87.5)  
    
    slope_1_2,const_1_2 = get_slope_const(Hexa_pt1,Hexa_pt2)
    slope_2_3,const_2_3 = get_slope_const(Hexa_pt2, Hexa_pt3) 
    slope_6_5, const_6_5 = get_slope_const(Hexa_pt6,Hexa_pt5)
    slope_5_4, const_5_4 = get_slope_const(Hexa_pt5, Hexa_pt4) 
    
    tri_pt1 = (460,225)
    tri_pt2 = (510,125)
    tri_pt3 = (460,25)
    
    tri_slope_1_2,tri_const_1_2 = get_slope_const(tri_pt1,tri_pt2)
    tri_slope_2_3,tri_const_2_3 = get_slope_const(tri_pt2,tri_pt3)
     
    for pts in coords:
        x, y = pts[0], pts[1] 
        #Rectangle 1
        if x>100 -clearance and x<150 +clearance and y >150-clearance and y <250:
            obstacles.append((x,y))
        #Rectangle 2
        if x>100-clearance and x<150+clearance and y >=0 and y <100+clearance:
            obstacles.append((x,y))
        # Hexagon Obstacle
        if x > 235.05 -  clearance and x < 364.95 + clearance:
            if (y - slope_1_2*x < const_1_2  + clearance) and  (y - slope_2_3*x < const_2_3 + clearance) and  (y - slope_6_5*x > const_6_5 - clearance) and  (y - slope_5_4*x > const_5_4  - clearance)  :
                obstacles.append((x,y))
        #Triangle
        if x>460-clearance and x<510 + clearance:
            if  (y - tri_slope_1_2*x < tri_const_1_2 + clearance) and (y - tri_slope_2_3*x > tri_const_2_3 - clearance)  :
                obstacles.append((x,y))
                
    return obstacles



"""
----------------------------------------------------------------
Action Set
---------------------------------------------------------------- 
"""

def up(coords,map_width,map_height):  

    if  coords[1]<map_height-1:
        coords= (coords[0]  , coords[1]+1) 
        return(coords,True)
    else:
        return(coords,False)
    
def upRight(coords,map_width,map_height): #DONE

    if coords[0]<map_width-1 and coords[1]<map_height-1:
        coords= (coords[0]+1  , coords[1]+1) 
        return(coords,True)
    else:
        return(coords,False)
    
def right(coords,map_width,map_height):  

    if coords[0]<map_width-1:
        coords= (coords[0]+1  , coords[1]) 
        return(coords,True)
    else:
        return(coords,False)
    

def downRight(coords,map_width,map_height):  #DONE

    if coords[0]<map_width-1 and coords[1]>0:
        coords= (coords[0]+1  , coords[1]-1) 
        return(coords,True)
    else:
        return(coords,False)
    
    
def down(coords,map_width,map_height):   

    if coords[1]>0:
        coords= (coords[0]  , coords[1]-1) 
        return(coords,True)
    else:
        return(coords,False)
    
def downLeft(coords,map_width,map_height):  #DONE

    if coords[0]>0 and coords[1]>0:
        coords= (coords[0]-1  , coords[1]-1) 
        return(coords,True)
    else:
        return(coords,False)

def left(coords,map_width,map_height):   

    if coords[0]>0:
        coords= (coords[0]-1  , coords[1]) 
        return(coords,True)
    else:
        return(coords,False)   
    
def upLeft(coords,map_width,map_height):  

    if coords[0]>0 and coords[1]<map_height-1:
        coords= (coords[0]-1  , coords[1]+1) 
        return(coords,True)
    else:
        return(coords,False)
    



