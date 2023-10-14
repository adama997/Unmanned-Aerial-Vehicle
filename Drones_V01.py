#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Apr 16 13:49:00 2023

@author: robot
"""

import time

from graph import *
from  Astar_V02 import *
from RRT_V01 import RRT
from Panel_V01 import *


class Drones(object):
    
    def __init__(self,speed,algorithm,graph,obstacle = None, vehicleList= None,panelList = None):
        self.speed=speed
        self.algorithm = algorithm
        self.graph = graph
        self.obstacle = obstacle
        self.panelList = panelList
        self.vehicleList = vehicleList
        self.path_node = {}
        self.path_segment = {}
        self.compteur = 0
        self.arrived = False
        self.state_list = []
        
        self.trajectoire = {}  #trajectoire
        for vehicle in self.vehicleList:
            self.trajectoire[vehicle.ID] = []
        #self.trajectoire[self.ID] = []
        self.drone_path = {}  #liste des segments
        
        self.init_path()
        self.path_finding()
        
        
    def init_path(self):
        """
        if self.algorithm == 'astar':
            start_code = get_code(self.start)
            end_code = get_code(self.end)
            astar = Astar(start_code,end_code,self.graph)
            self.path_node[self.ID] = astar.path
            self.path_segment[self.ID] = astar.path_segment
            self.start_time = time.time()
            self.trajectoire[self.ID].append(((get_node()[self.path_segment[self.ID][0][0]].x,get_node()[self.path_segment[self.ID][0][0]].y),(get_node()[self.path_segment[self.ID][0][0]].x,get_node()[self.path_segment[self.ID][0][0]].y)))
            print("\nAstar:",self.path_node[self.ID])
            #self.arrived = True
         """  
            
        if self.algorithm == 'rrt':
            for vehicle in self.vehicleList:
                start = (vehicle.position[0],vehicle.position[1])
                end = (vehicle.goal[0],vehicle.goal[1])
                rrt = RRT(self.obstacle,start,end)
           # rrt = RRT(self.obstacle,self.start,self.end)
                self.path_node[vehicle.ID] = rrt.path_nodes      # l'ensemble des noeuds
                self.path_segment[vehicle.ID] = rrt.path_segment  # l'ensemble des segments
                self.trajectoire[vehicle.ID].append(self.path_segment[vehicle.ID][0])   #l'ensemble des chemins
                vehicle.state = 0
                self.state_list.append(vehicle.state)
            
            print("\nFini de trouver les chemins RRT")
            self.start_time = time.time()
            
            #print("\nRRT:",self.path_node[self.ID])
            #self.arrived = True
        
        else:
            #V = Flow_velocity(self.vehicleList,self.panelList,self.obstacle)
            V = Flow_velocity(self.vehicleList,self.panelList,self.obstacle)
            for vehicle in self.vehicleList:
                self.path_node[vehicle.ID] = V.path_node[vehicle.ID]
                self.path_segment[vehicle.ID] = V.path_segment[vehicle.ID]
                self.trajectoire[vehicle.ID].append(self.path_segment[vehicle.ID][0])
                vehicle.state = 0
                self.state_list.append(0)    # Initialise la liste des états à 0 
                print("\ndrone")
           # print("\nPanel:",self.path_node[self.ID])
            #self.arrived = True
            print("\nNoeuds des panneaux:",self.path_node)
            print("\nFini de trouver les chemins PANEL")
            self.start_time = time.time()
            
    
    def path_finding(self):
        """
        if self.algorithm == 'astar':
            while not self.arrived:
                origin = self.path_segment[self.ID][0][0]
                destination = self.path_segment[self.ID][0][1]
                a = (get_node()[destination].y - get_node()[origin].y)/(get_node()[destination].x - get_node()[origin].x)  # coefficient directeur
                b = get_node()[origin].y - a*get_node()[origin].x
                distance_x = get_node()[destination].x - get_node()[origin].x    # distance sur l'axe X
                
                self.current_time = time.time()
                dist_x = get_node()[origin].x + self.speed*(self.current_time - self.start_time)   # pas suivant x
                dist_y = a*dist_x + b                                           # pas suivant y
                while dist_x < distance_x:
                    self.trajectoire[self.ID].append((self.trajectoire[self.ID][-1][1],(dist_x,dist_y)))
                    dist_x = dist_x + self.speed*(self.current_time - self.start_time)
                    dist_y = a*dist_x + b 
                    
                self.trajectoire[self.ID].append((self.trajectoire[self.ID][-1][1],(get_node()[destination].x,get_node()[destination].y)))
                #origin = self.trajectoire[self.ID][-1][0]
                #destination = self.trajectoire[self.ID][-1][1]
                if destination == get_code(self.end):
                    self.arrived = True
                    print("\nDrone arrive!")
                else:
                    end_code = get_code(self.end)
                    astar = Astar(destination,end_code,self.graph)
                    self.path_segment[self.ID] = astar.path_segment
                    #self.trajectoire[self.ID].append(self.path_segment[self.ID][0])
             """  
        
        if self.algorithm == 'rrt':
            for vehicle in self.vehicleList:
           # while sum(self.state_list) != len(self.vehicleList) :
            #    for vehicle in self.vehicleList:
                self.start_time = time.time()
                
                while not self.arrived :
                    #print(self.path_segment[vehicle.ID][self.compteur])
                    origin = self.path_segment[vehicle.ID][self.compteur][0]    # 1er noeud du 1er segment
                    destination = self.path_segment[vehicle.ID][self.compteur][1]       #2nd noeud du 1er segment
                    a = (destination[1] - origin[1])/(destination[0] - origin[0])  # coefficient directeur
                    b = origin[1] - a*origin[0]
                    distance_x = abs(destination[0] - origin[0])    # distance sur l'axe X
                    
                    #time.sleep(0.001)
                    self.current_time = time.time()
                    dist_x = origin[0] + self.speed*(self.current_time - self.start_time)   # pas suivant x
                    dist_y = a*dist_x + b                                           # pas suivant y
                    while dist_x < distance_x:
                        self.trajectoire[vehicle.ID].append((self.trajectoire[vehicle.ID][-1][1],(dist_x,dist_y)))
                        dist_x = dist_x + self.speed*(self.current_time - self.start_time)
                        dist_y = a*dist_x + b 
                    
                    self.trajectoire[vehicle.ID].append((self.trajectoire[vehicle.ID][-1][1],(destination[0],destination[1])))   #le dernier point calculé au noeud final du segment
                
                    if destination == (vehicle.goal[0],vehicle.goal[1]):     # test si noeud final du segment est au egale au noeud de destination
                        self.arrived = True
                        vehicle.state = 1
                        self.state_list[vehicle.ID - 1] = 1
                        print("\nDrone arrive!")
                    
                    else:
                        self.compteur +=1
                    
                    time.sleep(distance_x/self.speed)
                    
                self.arrived = False
                self.compteur = 0

        
        else:
            print("\nJe suis ici")
            for vehicle in self.vehicleList:
            #while sum(self.state_list) != len(self.vehicleList) :
                #for vehicle in self.vehicleList:
                self.start_time = time.time()
                
                while not self.arrived :
                    
                    origin = self.path_segment[vehicle.ID][self.compteur][0]    # 1er noeud du 1er segment
                    destination = self.path_segment[vehicle.ID][self.compteur][1]       #2nd noeud du 1er segment
                    a = (destination[1] - origin[1])/(destination[0] - origin[0])  # coefficient directeur
                    b = origin[1] - a*origin[0]
                    distance_x = abs(destination[0] - origin[0])    # distance sur l'axe X
                    
                    #time.sleep(0.001)
                    self.current_time = time.time()
                    dist_x = origin[0] + self.speed*(self.current_time - self.start_time)   # pas suivant x
                    dist_y = a*dist_x + b                                           # pas suivant y
                    while dist_x < distance_x:
                        self.trajectoire[vehicle.ID].append((self.trajectoire[vehicle.ID][-1][1],(dist_x,dist_y)))
                        dist_x = dist_x + self.speed*(self.current_time - self.start_time)
                        dist_y = a*dist_x + b 
                    
                    self.trajectoire[vehicle.ID].append((self.trajectoire[vehicle.ID][-1][1],(destination[0],destination[1])))   #le dernier point calculé au noeud final du segment
                
                    if destination == (vehicle.goal[0],vehicle.goal[1]):     # test si noeud final du segment est au egale au noeud de destination
                        self.arrived = True
                        vehicle.state = 1
                        self.state_list[vehicle.ID - 1] = 1
                        print("\nDrone arrive!")
                    
                    else:
                        self.compteur +=1
                    
                    time.sleep(distance_x/self.speed)
                    
                self.arrived = False
                self.compteur = 0
        
    
                        