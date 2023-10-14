#!/usr/bin/env python3

"""
This is the first version of RRT algorithm implemented in python. In this version, we have already defined obstacles

"""

import random
import math
from shapely.geometry import Point, Polygon, LineString
from graph import Toulouse,get_node
from Function import calc_distance,point_to_line


class RRT(object):
    """
    k
    """
    
    def __init__(self,obstacle,start,end):
        """
        obstacle is in this format: [["Polygon",poly],["Circle",[center,rayon]]]
        """
        self.side_len = 10

        self.num_obs = random.randint(15, 25)                       # number of obstacles
        self.obstacle_props = obstacle                              # List of obstacles of the scenario

        self.start, self.end = start,end                            # start and end points
        
        self.nodes_list = [['q0', self.start, 'None']]               
        #self.gen_start()
                   
                                                                    # [Node name, node coordinates, parent node]
        self.segments_list = []                                     # list of node-to-node path segments

        self.gen_tree()                                             # list of nodes in tree

        self.path_nodes, self.path_segment = [], []               # nodes/segments along the path
                                                                    # from start to finish
        
        
        self.find_path() 
        
        #self.plot=self.plot()
        
       
            
            
    def obstacle_check(self, point):
        """
        Checks whether a point is inside any of the obstacles.

        ARGUMENTS
            point           Point to check; tuple

        OUTPUT
            collision       Collision condition; boolean (true if collision exists)
        """
        
        for obs in self.obstacle_props:
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
    
    def gen_start(self):
        
        while self.obstacle_check(self.start):
            #print("POINT CONTENU DANS L'OBSTACLE\n")
            code = []
            [code.append(elt) for elt in Toulouse().codes]
            j=random.choice(code)
            self.start = (get_node()[j].x,get_node()[j].y)  #Noeud de départ
            
        
        self.nodes_list = [['q0', self.start, 'None']] 
        #print("Nouveau start:",self.start)
        
        
    def find_closest(self, point):
        """
        Finds the closest existing tree node to a given point.

        ARGUMENTS
            point           Point to find closest node to

        OUTPUT
            ind             List index of closest node in tree
        """
        d_list = [calc_distance(point, node[1]) for node in self.nodes_list]

        return min(range(len(d_list)), key=d_list.__getitem__)

    def gen_node(self):
        """
        Freeflight model
        """
        point_ok = False
        node_name = "q{}".format(len(self.nodes_list))
        
        coord = [(-8,-2),(-8,25),(20,25),(20,-2)]
        poly = Polygon(coord)
        
        
        while not point_ok:
            # generate random coordinates
            p_coords = (self.side_len*random.random(), self.side_len*random.random())

            # find parent node
            parent = self.nodes_list[self.find_closest(p_coords)]
            
            #parent = self.nodes_list[-1]
            # print(parent)

            # x- and y-distances to random point from parent node
            d_x = p_coords[0] - parent[1][0]
            d_y = p_coords[1] - parent[1][1]

            # magnitude of vector to closest node
            vec_mag = math.sqrt((d_x**2) + (d_y**2))

            # get new node coordinates by adding unit vector components to parent coordinates
            node = (parent[1][0] + d_x/vec_mag,
                    parent[1][1] + d_y/vec_mag)
            
            pnt = Point(node)
            # if newly created node
            if self.obstacle_check(node) :
                #print("Interieur d'un obstacle\n")
                pass
            
            elif pnt.within(poly) == False :
                #print("Sorti du champ\n")
                pass
            
            else:
                point_ok = True

        self.nodes_list.append([node_name, node, parent[0]])
        self.segments_list.append([parent[1], node])

    def path_check(self, point):
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
        for obs in self.obstacle_props:
            too_close, between = False, False
            #intersept = False
            
            # return tangent distance and intersection ratio between obstacle center and path to end
            if obs[0]=="Circle":                 # The obstacle is a circle
                center=obs[1][0]                 # The center of the circle
                radius=obs[1][1]                 # The radius of the circle
                r_u, d_obs = point_to_line(point, self.end, center)
                # determine if line segment and tangent through obstacle center intersect within segment bounds
                if 0 <= r_u <= 1:
                    between = True
                
                if d_obs <= radius:
                    too_close = True
                
            else:                                # The obstacle is a polygon
                #center=(obs[1].centroid.x,obs[1].centroid.y)    # The center of the polygon
                line = LineString([point,self.end])
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

    def gen_end_seg(self):
        """
        Generates final path segment and adds to list of segments.
        """

        self.segments_list.append([self.nodes_list[-1][1], self.end])

    def gen_tree(self):
        """
        k
        """
        done = False

        while not done:
            self.gen_node()

            if not self.path_check(self.nodes_list[-1][1]):
                done = True

        self.gen_end_seg()

        self.nodes_list.append(["q{}".format(len(self.nodes_list)), self.end, self.nodes_list[-1][0]])

    def find_path(self):
        """
        Works backward through the list of points to find the path from start to finish.
        """
        current = self.nodes_list[-1]               # set end as current node
        self.path_nodes.append(current[1])          # append end coordinates to list of path nodes

        for _, j in reversed(list(enumerate(self.nodes_list))):
            if current[2] == j[0]:
                self.path_nodes.insert(0, j[1])
                self.path_segment.insert(0, (j[1], current[1]))
                current = j
            
