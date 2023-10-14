#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Apr 16 14:38:35 2023

@author: robot
"""

from shapely.geometry import Polygon, Point
import time 
from datetime import datetime
import csv
import random
from graph import get_node,get_code,Toulouse,Obstacles,Z_Toulouse
from Panel_V01 import Vehicle
from Drones_V01 import Drones
from Plot import Plot
from Function import calc_distance,obstacle_check
from math import ceil


class Scenario: 

    def __init__(self,Graphe,N,speed,algo,obstacle=None,panelList=None,file=None):
        self.start_time = time.time()
        
        self.graphe=Graphe
        self.nb_drones=N
        self.speed=speed
        self.drones=[]
        
        self.vehicleList = []
        self.rrt_vehicle_list = []
        self.rrt_Trajectoires = None
        self.panel_vehicle_list = []
        self.panel_Trajectoires = None
        
        self.conflits=0                                #compte les conflits
        self.algo = algo
        #self.algorithm = {}
        self.obstacle = obstacle 
        self.panelList = panelList
        self.TRAJECTOIRE = {}
        self.DISTANCE = {}
        
        for i in range(self.nb_drones+1):
            if i>0:
                self.DISTANCE[i] = 0
                
                
        self.nodes = {}
        self.file = file
        self.csv_file = 'Scenario_{}_{}.csv'.format(self.nb_drones,self.speed) # CSV  File which contains some drones characteristics
        self.file_distance = '{}.csv'.format(self.algo)
        self.date = str(datetime.now())
        self.trace_file = 'TRACE_FILE_{}_{}.txt'.format(self.algo,self.date)
        
        self.D_MIN = 10 /1000                                                  # Minimum distance for drones separation
        self.TEMPS_SIMULATION = 0  
        self.TEMPS = {} 
        self.TEMPS_MOY = 0                                          # Temps de simulation
        self.COLLISION = 0
        self.INTER_DRONE_DISTANCE = 0
        self.VITESSE_MOY = self.speed
        self.VITESSE_MIN = 0
        self.VITESSE_MAX = 0
        self.ATTENTE = 0
        self.RETARD = 0
        self.DEVIATION = 0
        
        self.init_vehicle()
        self.evolution()
        
        self.end_time = time.time()
        self.TEMPS_SIMULATION = ceil((self.end_time - self.start_time))    # Simulation time in s
        
        self.parameters()                                                       # Find the number of collisions
        
        self.IOFile('w')                                                       # Ecrire un fichier csv 
        
        Plot(self.graphe, self.obstacle,self.rrt_Trajectoires,self.panel_Trajectoires)  # Plot the results
        
        #print("\n Temps de simulation:",end_time-start_time)
    
    def IOFile(self,io):
        if io == 'r':
            with open (self.csv_file,newline='') as csvfile:
                reader = csv.DictReader(csvfile)
                for row in reader:
                    ID = int(row['ID'])
                    start_code = int(row['START CODE'])
                    end_code = int(row['END CODE'])
                    #algo = row['ALGORITHM']
                    start, end = (get_node()[start_code].x,get_node()[start_code].y), (get_node()[end_code].x,get_node()[end_code].y)
                    vehicle = Vehicle(ID)
                    vehicle.Set_Goal([end[0],end[1],0], 5, 0)
                    vehicle.Set_Position([start[0],start[1],0])
                    self.vehicleList.append(vehicle)
                    #self.algorithm[ID] = algo
        else:
            with open (self.csv_file,'w',newline='') as csvfile:               # Write in csv file
                fieldnames = ['ID','START CODE','END CODE']
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                writer.writeheader()
                for vehicle in self.vehicleList:
                    ID = vehicle.ID
                    start_code = get_code(self.TRAJECTOIRE[ID][0][0])
                    print("\nSTART : ",start_code)
                    end_code = get_code(self.TRAJECTOIRE[ID][-1][1])
                    
                    writer.writerow({'ID':ID,
                                     'START CODE' : start_code,
                                     'END CODE' : end_code})
            
            
            
            with open (self.file_distance,'w',newline='') as csvfile:               # Write in csv file
                fieldnames = ['ID','START CODE','END CODE','DISTANCE (m)','TEMPS (min)']
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                writer.writeheader()
                for ID in self.TRAJECTOIRE.keys() :
                
                    start_code = get_code(self.TRAJECTOIRE[ID][0][0])
                    end_code = get_code(self.TRAJECTOIRE[ID][-1][1])
                    distance = self.DISTANCE[ID]
                    time = self.TEMPS[ID]
                    
                    writer.writerow({'ID':ID,
                                     'START CODE' : start_code,
                                     'END CODE' : end_code,
                                     'DISTANCE (m)' : distance,
                                     'TEMPS (min)' : time})
                    
                    
            with open (self.trace_file,'w',newline='') as textfile:             # Write in trace file
                textfile.write("\t FICHIER DE TRACE\n\n")
                textfile.write("\nNombre de Drones : {}\n".format(self.nb_drones))
                textfile.write("\nVitesse Moyenne : {} m/s\n".format(self.VITESSE_MOY))
                textfile.write("\nVitesse Minimale : {} m/s\n".format(self.VITESSE_MIN))
                textfile.write("\nVitesse Maximale : {} m/s\n".format(self.VITESSE_MAX))
                textfile.write("\nRetard Maximal : {} s\n".format(self.RETARD))
                textfile.write("\nDéviation Moyenne : {}\n".format(self.DEVIATION))
                #textfile.write("Déviation Maximale : {}\n".format(self.speed))
                textfile.write("\nNombre d'attentes stationnaires : {}\n".format(self.ATTENTE))
                textfile.write("\nNombre de Conflits : {}\n".format(self.COLLISION))
                textfile.write("\nDistance Minimale entre deux drones : {} m\n".format(self.INTER_DRONE_DISTANCE))
                textfile.write("\nTemps de Simulation : {} s\n".format(self.TEMPS_SIMULATION))
                textfile.write("\nTemps moyen de simulation des drones : {} min\n".format(self.TEMPS_MOY))
    
    
    def init_vehicle(self):
        
        if self.file !=None:
            print("Lecture du ficher\n")
            self.IOFile('r')
        
        else:
                    
            for i in range(self.nb_drones+1):
                if i>0:
                    
                    code = []
                    [code.append(elt) for elt in self.graphe.codes]
                    j=random.choice(code)
                    k=j
                    while k==j:
                        k=random.choice(code)            #pour être sûr d'avoir une destination différente du point de départ
                    
                    #randm_algo = np.random.randint(0,2)
                    
                    #print(j,k)
                    start = (get_node()[j].x,get_node()[j].y)  #Noeud de départ
                    end = (get_node()[k].x,get_node()[k].y)    #Noeud d'arrivé
                    
                    while obstacle_check(start,self.obstacle):
                        #print("POINT CONTENU DANS L'OBSTACLE\n")
                        code = []
                        [code.append(elt) for elt in Toulouse().codes]
                        j=random.choice(code)
                        start = (get_node()[j].x,get_node()[j].y)  #Noeud de départ
                    
                    vehicle = Vehicle(i)
                    vehicle.Set_Goal([end[0],end[1],0], 5, 0)
                    vehicle.Set_Position([start[0],start[1],0])
                    self.vehicleList.append(vehicle)
                    #self.DISTANCE[i] = 0
                    #self.algorithm[i] = self.algo
        
        if self.algo == 'rrt':
            self.rrt_vehicle_list = self.vehicleList
        elif self.algo == 'panel':
            self.panel_vehicle_list = self.vehicleList
        else:
            pass
        
    
    def parameters(self):
        path_list = []
        distance_list = []
        time_list = []
        
        for ID in self.TRAJECTOIRE.keys():
            path_list.append(self.TRAJECTOIRE[ID])
        
            # Distance of each drones
            for segment in self.TRAJECTOIRE[ID]:
                self.DISTANCE[ID] += ceil((calc_distance(segment[0],segment[1]))*1000)
            
            self.TEMPS[ID] = ceil(self.DISTANCE[ID]/(self.speed*60))           #Temps de simulation en minutes
            time_list.append(self.TEMPS[ID])
            #v = self.DISTANCE[ID]/self.TEMPS_SIMULATION
            #speed_list.append(v)
        self.TEMPS_MOY = ceil((sum(time_list)/self.nb_drones))       # mean time
        
        for path in path_list:
            
            #v = (calc_distance(path[0][0], path[-1][1])*1000)/(self.TEMPS_SIMULATION*60)
            #speed_list.append(v) 
            conflit = False
            path_list.remove(path)                   # Delete the segment
            
            for seg in path:
                
                (X,Y) = seg[0]                          # center of our polygon
                center = (X,Y)
                radius = self.D_MIN
                polygon = Polygon([(X+radius,Y-radius),(X+radius,Y+radius),(X-radius,Y+radius),(X-radius,Y-radius)])
            
                for other_path in path_list:
                    for segment in other_path:
                        node = Point(segment[0])
                        distance_list.append(calc_distance(center,segment[0]))  # add to distance_list the distance between center and current node
                        if node.within(polygon):
                            self.COLLISION +=1               # Detection of a probable collision
                            conflit = True
                            break
                if conflit:
                    break
        
        # find the minimal distance between to drones
        self.INTER_DRONE_DISTANCE = ceil(min(distance_list)*1000)
        
        # find maximal and minimal speed
        self.VITESSE_MIN = self.speed                              #ceil(min(speed_list))
        self.VITESSE_MAX = self.speed                              #ceil(max(speed_list))
    
    
    
    def __repr__(self):
        c="Scenario : {0.nb_drones} drones\n".format(self)
        for i in self.drones:
            c+="Drone allant de {0.start} à {0.end} actuellement au point({0.x},{0.y})\n".format(i)
        return c
        
    def evolution(self):
        #print(Scenario1)
        #status=[0 for i in range (self.nb_drones)] #status[i]=1 si le drone i est arrivé
        #if sum(status) < self.nb_drones:
           # print(sum(status))
        print("\n Prêt à calculer les chemins")
        
        if self.rrt_vehicle_list != []:
            drones_rrt = Drones(self.speed, 'rrt',self.graphe,self.obstacle,self.rrt_vehicle_list)
            self.rrt_Trajectoires = drones_rrt.trajectoire
            self.TRAJECTOIRE = self.rrt_Trajectoires
            print("\n Trajectoire RRT:", self.rrt_Trajectoires)
        
        if self.panel_vehicle_list != []:
            drones_panel = Drones(self.speed, 'panel',self.graphe,self.obstacle,self.panel_vehicle_list,self.panelList)
            self.panel_Trajectoires = drones_panel.trajectoire
            self.TRAJECTOIRE = self.panel_Trajectoires
            print("\n Trajectoire PANEL:", self.panel_Trajectoires)
        
        #print("\n Prêt à afficher!!")          
        
        print("La simulation est terminée")


# Exemple de scénario

obstacle,panel = Obstacles(Z_Toulouse)
file = '/home/robot/Documents/AVI/PIR_DRONES/Scenario_10_10.csv'
scenario = Scenario(Toulouse(), 10, 10,'rrt',obstacle,panel,file)

