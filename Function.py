#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Mar 25 16:50:56 2023

@author: robot
"""

"""
This is some functions which will be used in the implementation of RRT; A* algorithms

"""

import math
import random
from shapely.geometry import Point,Polygon,LineString

# A function that will calculate a distance between two nodes
def calc_distance(p_1, p_2):
    """
    Calculates distance between two points.

    ARGUMENTS
        p1      Point 1 coordinates; tuple
        p2      Point 2 coordinates; tuple

    OUTPUT
        Straight-line distance from p1->p2; scalar
    """
    return math.sqrt((p_2[0]-p_1[0])**2 + (p_2[1]-p_1[1])**2)


# A function that will calcule a distance intersection 
def point_to_line(p_1, p_2, p_3):
    """

    ARGUMENTS
        p_1     Point 1 coordinates; tuple
        p_2     Point 2 coordinates; tuple
        p_3     Point 3 coordinates; tuple

    OUTPUT
        r_u         p_1 -> p_2 distance intersection ratio; scalar
        tan_len     Distance from p_1 -> p_2 line to p_3; scalar
    """
    # distance from P1 -> P2
    dist = math.sqrt((p_2[0]-p_1[0])**2 + (p_2[1]-p_1[1])**2)

    # determine intersection ratio u
    # for three points A, B with a line between them and a third point C, the tangent to the line AB
    # passing through C intersects the line AB a distance along its length equal to u*|AB|
    r_u = ((p_3[0] - p_1[0])*(p_2[0] - p_1[0]) + (p_3[1] - p_1[1])*(p_2[1] - p_1[1]))/(dist**2)

    # intersection point
    p_i = (p_1[0] + r_u*(p_2[0] - p_1[0]), p_1[1] + r_u*(p_2[1] - p_1[1]))

    # distance from P3 to intersection point
    tan_len = calc_distance(p_i, p_3)

    return r_u, tan_len

def obstacle_check(point,obstacle):
    """
    Checks whether a point is inside any of the obstacles.

    ARGUMENTS
        point           Point to check; tuple

    OUTPUT
        collision       Collision condition; boolean (true if collision exists)
    """
    
    for obs in obstacle:
        if obs[0]=="Circle":        # obstacle is a circle
            
            if calc_distance(point, obs[1][0]) <= obs[1][1]:
                return True
            else:
                pass
        else:                       # obstacle is a polygon
            
            center=(obs[1].centroid.x,obs[1].centroid.y) 
            minx,miny,maxx,maxy=obs[1].bounds
            radius=0.5*calc_distance((minx,miny),(maxx,maxy))
            if calc_distance(point, center) <= radius:
                return True
            else:
                pass
            
            """
            pnt = Point(point[0],point[1])
            if pnt.within(obs[1]) == True:
                return True
            else:
                pass
            """
            
    return False



def check_obstacle(point,obstacle):
    """
    Checks whether a point is inside any of the obstacles.

    ARGUMENTS
        point           Point to check; tuple

    OUTPUT
        collision       Collision condition; boolean (true if collision exists)
    """
    
    for obs in obstacle:
        if obs[0]=="Circle":        # obstacle is a circle
            
            if calc_distance(point, obs[1][0]) <= obs[1][1]:
                return True
            else:
                pass
        else:                       # obstacle is a polygon
            
            #center=(obs[1].centroid.x,obs[1].centroid.y) 
            #minx,miny,maxx,maxy=obs[1].bounds
            #radius=0.5*calc_distance((minx,miny),(maxx,maxy))
            pnt = Point(point)
           # line = LineString([node1,node2])
            if pnt.within(obs[1]) :
                return True
            else:
                pass
            
    return False

def check_path(obstacle,end,point):
     """
     Checks for a clear straight-line path from a node to the end point.

     ARGUMENTS
         point       Point to check; tuple

     OUTPUT
         Collision condition; boolean (true if collision(s) present)
     """
     # empty list to hold collision conditions between path and individual obstacles
     path_collisions = []
     
     # find parent node
     #parent = self.nodes_list[-2][1]
     #print(parent)
     
     # check for collision with each obstacle
     for obs in obstacle:
         too_close, between = False, False
         #intersept = False
         
         # return tangent distance and intersection ratio between obstacle center and path to end
         if obs[0]=="Circle":                 # The obstacle is a circle
             center=obs[1][0]                 # The center of the circle
             radius=obs[1][1]                 # The radius of the circle
             r_u, d_obs = point_to_line(point, end, center)
             # determine if line segment and tangent through obstacle center intersect within segment bounds
             if 0 <= r_u <= 1:
                 between = True
             
             if d_obs <= radius:
                 too_close = True
             
         else:                                # The obstacle is a polygon
             #center=(obs[1].centroid.x,obs[1].centroid.y)    # The center of the polygon
             line = LineString([point,end])
            # line0 = LineString([point,parent])
             if line.intersects(obs[1]):
                 between = True
                 too_close = True
               
         # path is blocked if intersection is both:
         #   a) within segment bounds
         #   b) closer to obstacle center than obstacle radius length
         if between and too_close:
             path_collisions.append(True)
         else:
             path_collisions.append(False)

     return any(path_collisions)
 

def check_pathe(obstacle,end,point):
    """
    Checks for a clear straight-line path from a node to the start point.

    ARGUMENTS
        point       Point to check; tuple

    OUTPUT
        Collision condition; boolean (true if collision(s) present)
    """
    # empty list to hold collision conditions between path and individual obstacles
    path_collisions = []

    # check for collision with each obstacle
    for obs in obstacle:
        too_close, between = False, False

        # return tangent distance and intersection ratio between obstacle center and path to end
        if obs[0]=="Circle":   # The obstacle is a circle
            center=obs[1][0]   # The center of the circle
        else:                  # The obstacle is a polygon
            center=(obs[1].centroid.x,obs[1].centroid.y)    # The center of the polygon
        
        r_u, d_obs = point_to_line(end, point, center)

        # determine if line segment and tangent through obstacle center intersect within segment bounds
        if 0 <= r_u <= 1:
            between = True

        # determine if intersection distance is smaller than obstacle radius
        if obs[0]=="Circle":
            radius=obs[1][1]   # The radius of the circle
           # print(radius)
        else:
            minx,miny,maxx,maxy=obs[1].bounds
            radius=0.5*calc_distance((minx,miny),(maxx,maxy))
            
        if d_obs <= radius:
            too_close = True

        # path is blocked if intersection is both:
        #   a) within segment bounds
        #   b) closer to obstacle center than obstacle radius length
        if between and too_close:
            path_collisions.append(True)
        else:
            path_collisions.append(False)

    return any(path_collisions)


def node_gen(obstacle,parent):
    """
    Freeflight model
    """
    point_ok = False
    #node_name = "q{}".format(len(self.nodes_list))
    side_len = 5
    side_coef = 1
    
    while not point_ok:
        # generate random coordinates
        p_coords = (side_len*random.random(), side_len*random.random())

        # find parent node
        #parent = self.nodes_list[self.find_closest(p_coords)]

        # print(parent)

        # x- and y-distances to random point from parent node
        d_x = p_coords[0] - parent[0]
        d_y = p_coords[1] - parent[1]

        # magnitude of vector to closest node
        vec_mag = math.sqrt((d_x**2) + (d_y**2))

        # get new node coordinates by adding unit vector components to parent coordinates
        node = (parent[0] + d_x*side_coef/vec_mag,
               parent[1] + d_y*side_coef/vec_mag)

        # if newly created node
        if check_obstacle(node,obstacle):
            pass
        else:
            point_ok = True

    return node
