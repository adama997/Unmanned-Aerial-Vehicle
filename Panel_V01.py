#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Apr  3 15:56:16 2023

@author: robot
"""

"""

Implementation of the first version of panel method

"""

import numpy as np
import math
import pyclipper
from itertools import compress



#===================================================== CLASS PANEL =========================================================


class Panels(object):
    
    def __init__(self,vertices):             # A panel method is defined by one obstacle
        #self.name = name
        self.vertices = np.array(vertices)        # Vertices of obstacles which are defined as coordinates point
        
        self.nop = 0
        self.pcp = None
        self.pb = None
        self.K_inv = self.Coefficient_matrix()
        self.gamma = {}
        #self.Vortex_strength()
       # self.velocity = None 
    
    # First step : Coefficient matrix computation
    def Coefficient_matrix(self):
        """

        Raises
        ------
        ValueError
            Error if size is too large!

        Returns
        -------
        TYPE : Matrix
            Return a coefficient matrix (panel method) which is produced from the shape of the obstacles

        """
        panels = np.array([])
        rad = 0.15
        #safetyfac = 1.1
        scale = 1e6
        pco = pyclipper.PyclipperOffset()
        pco.AddPath( (self.vertices[:,:2] * scale).astype(int).tolist() , pyclipper.JT_MITER, pyclipper.ET_CLOSEDPOLYGON)
        inflated  =  np.array ( pco.Execute( rad*scale )[0] ) / scale
        height = self.vertices[0,1]
        points = np.hstack(( inflated , np.ones((inflated.shape[0],1)) *height ))
        Xavg = np.mean(points[:,0:1])
        Yavg = np.mean(points[:,1:2])
        angles = np.arctan2( ( Yavg*np.ones(len(points[:,1])) - points[:,1] ) , ( Xavg*np.ones(len(points[:,0])) - points[:,0] ) )
        sorted_angles = sorted(zip(angles, points), reverse = True)
        points_sorted = np.vstack([x for y, x in sorted_angles])
        self.vertices = points_sorted
        size=0.01 # Panelize
        for index,vertice in enumerate(self.vertices): # Divides obstacle edges into smaller line segments, called panels.
            xyz1 = self.vertices[index]                                 # Coordinates of the first vertice
            xyz2 = self.vertices[ (index+1) % self.vertices.shape[0] ]  # Coordinates of the next vertice
            s    = ( (xyz1[0]-xyz2[0])**2 +(xyz1[1]-xyz2[1])**2)**0.5   # Edge Length
            n    = math.ceil(s/size)                                    # Number of panels given desired panel size, rounded up
            if n == 1:
                raise ValueError('Size too large. Please give a smaller size value.')
            if panels.size == 0:
                panels = np.linspace(xyz1,xyz2,n)[1:]
            else:
                panels = np.vstack((panels,np.linspace(xyz1,xyz2,n)[1:])) # Divide the edge into "n" equal segments:
        
         # Compute coeff matrix for Vortex method
        self.nop = panels.shape[0]    # Number of Panels
        self.pcp = np.zeros((self.nop,2))  # Controlpoints: at 3/4 of panel
        vp = np.zeros((self.nop,2))  # Vortex point: at 1/4 of panel
        #pl = np.zeros((self.nop,1))  # Panel Length
        self.pb  = np.zeros((self.nop,1))  # Panel Orientation; measured from horizontal axis, ccw (+)tive, in radians
        XYZ2 = panels                      # Coordinates of end point of panel
        XYZ1 = np.roll(panels,1,axis=0)    # Coordinates of the next end point of panel
        self.pcp  = XYZ2 + (XYZ1-XYZ2)*0.75 # Controlpoints point at 3/4 of panel. #self.pcp  = 0.5*( XYZ1 + XYZ2 )[:,:2]
        vp = XYZ2 + (XYZ1-XYZ2)*0.25 # Vortex point at 1/4 of panel.
        self.pb = np.arctan2( ( XYZ2[:,1] - XYZ1[:,1] ) , ( XYZ2[:,0] - XYZ1[:,0] ) )  + np.pi/2
        K = np.zeros((self.nop,self.nop))
        for m in range(self.nop ):
             for n in range(self.nop ):
                 K[m,n] = ( 1 / (2*np.pi)
                     * ( (self.pcp[m][1]-vp[n][1] ) * np.cos(self.pb[m] ) - ( self.pcp[m][0] - vp[n][0] ) * np.sin(self.pb[m] ) )
                     / ( (self.pcp[m][0]-vp[n][0] )**2 + (self.pcp[m][1] - vp[n][1] )**2 ) )
        return np.linalg.inv(K) # Inverse of coefficient matrix: (Needed for solution of panel method eqn.)
    
    
    # 2nd step : Vortex strength computation
    def Vortex_strength(self,vehicle,othervehicles):
        """
        Parameters
        ----------
        vehicle : Dictionary
            A dictionary which contains drones identified by their ID
        othervehicles : Dictionary
            The other drones

        Returns
        -------
        TYPE : vortex strength
            unknown vortex strength which take into account drones and obstacles's panels

        """
        vel_sink   = np.zeros((self.nop,2))
        vel_source = np.zeros((self.nop,2))
        vel_source_imag = np.zeros((self.nop,2))
        RHS        = np.zeros((self.nop,1))
        vel_sink[:,0] = (-vehicle.sink_strength*(self.pcp[:,0]-vehicle.goal[0]))/(2*np.pi*((self.pcp[:,0]-vehicle.goal[0])**2+(self.pcp[:,1]-vehicle.goal[1])**2))
        vel_sink[:,1] = (-vehicle.sink_strength*(self.pcp[:,1]-vehicle.goal[1]))/(2*np.pi*((self.pcp[:,0]-vehicle.goal[0])**2+(self.pcp[:,1]-vehicle.goal[1])**2))
        vel_source_imag[:,0] = (vehicle.imag_source_strength*(self.pcp[:,0]-vehicle.position[0]))/(2*np.pi*((self.pcp[:,0]-vehicle.position[0])**2+(self.pcp[:,1]-vehicle.position[1])**2))
        vel_source_imag[:,1] = (vehicle.imag_source_strength*(self.pcp[:,1]-vehicle.position[1]))/(2*np.pi*((self.pcp[:,0]-vehicle.position[0])**2+(self.pcp[:,1]-vehicle.position[1])**2))
        
        for i,othervehicle in enumerate(othervehicles) :
            vel_source[:,0] += (othervehicle.source_strength*(self.pcp[:,0]-othervehicle.position[0]))/(2*np.pi*((self.pcp[:,0]-othervehicle.position[0])**2+(self.pcp[:,1]-othervehicle.position[1])**2))
            vel_source[:,1] += (othervehicle.source_strength*(self.pcp[:,1]-othervehicle.position[1]))/(2*np.pi*((self.pcp[:,0]-othervehicle.position[0])**2+(self.pcp[:,1]-othervehicle.position[1])**2))

        RHS[:,0]  = -vehicle.V_inf[0]  * np.cos(self.pb[:])  \
                  -vehicle.V_inf[1]  * np.sin(self.pb[:])  \
                  -vel_sink[:,0]     * np.cos(self.pb[:])  \
                  -vel_sink[:,1]     * np.sin(self.pb[:])  \
                  -vel_source[:,0]   * np.cos(self.pb[:])  \
                  -vel_source[:,1]   * np.sin(self.pb[:])  \
                  -vel_source_imag[:,0]  * np.cos(self.pb[:])  \
                  -vel_source_imag[:,1]  * np.sin(self.pb[:])  +vehicle.safety
        self.gamma[vehicle.ID] = np.matmul(self.K_inv,RHS)
     


# ================================================== CLASS VEHICLE ==============================================


class Vehicle(object):
#  def __init__(self,ID,sock,source_strength = 0, imag_source_strength = 0.4):
  def __init__(self,ID):
    self.position_enu = np.zeros(3)
    self.velocity_enu = np.zeros(3)
    self.heading = 0
    
    #self.start = start
    #self.end = end
    
    self.altitude        = 0
    self.sink_strength   = 10
    self.V_inf           = np.zeros(3)
    self.safety          = 0

    self.t               = 0
    self.position        = np.zeros(3)
    self.velocity        = np.zeros(3)
    self.goal            = np.zeros(3)
    self.source_strength = 0
    self.imag_source_strength = 0.4
    self.gamma           = 0
    self.altitude_mask   = None
    self.ID              = ID
    self.path            = self.position
    self.state           = 0
    self.distance_to_destination = None
    self.velocitygain    = 1/50 # 1/300 or less for vortex method, 1/50 for hybrid
    self.velocity_desired = np.zeros(3)
    self.velocity_corrected = np.zeros(3)
    self.vel_err = np.zeros(3)

  def Set_Position(self,pos):
    self.position = np.array(pos)
    self.path     = np.array(pos)
    # print('GOOOAAALLL : ', self.goal)
    if np.all(self.goal) != None:
      self.distance_to_destination = np.linalg.norm(np.array(self.goal)-np.array(self.position))
      #print("dist_to_goal_set_pos:",self.distance_to_destination)
      if np.all(self.distance_to_destination < 2):
        self.state = 1
      #print(np.all(self.distance_to_destination))
      #print("Etat_set_pos:",self.state)

  def Set_Velocity(self,vel):
    self.velocity = vel

  def Set_Desired_Velocity(self,vel, method='direct'):
    self.velocity_desired = vel
    self.correct_vel(method=method)


  def correct_vel(self, method='None'):
    if method == 'projection':
      wind = self.velocity - self.velocity_desired
      self.vel_err = self.vel_err - (wind - np.dot(wind, self.velocity_desired/np.linalg.norm(self.velocity_desired) ) * 
              np.linalg.norm(self.velocity_desired) ) *(1./240.)
    elif method == 'direct':
      # err = self.velocity_desired - self.velocity
      self.vel_err = (self.velocity_desired - self.velocity)*(1./40.)
      # self.vel_err = (self.velocity_desired - self.velocity)
      # print(f' Vel err : {self.vel_err[0]:.3f}  {self.vel_err[1]:.3f}  {self.vel_err[2]:.3f}')
    else:
      self.vel_err = np.zeros(3)
            
    self.velocity_corrected = self.velocity_desired + self.vel_err
    self.velocity_corrected[2] = 0.


  def Set_Goal(self,goal,goal_strength,safety):
    self.goal          = goal
    self.sink_strength = goal_strength
    self.safety = safety

  def Set_Next_Goal(self,goal, goal_strength=500):
    self.state         = 0
    self.goal          = goal
    # self.sink_strength = goal_strength NOT USED FOR NOW

  def Go_to_Goal(self,altitude,AoA,t_start,Vinf):
    self.altitude = altitude                                       # Cruise altitude
    self.V_inf    = np.array([Vinf*np.cos(AoA), Vinf*np.sin(AoA)]) # Freestream velocity. AoA is measured from horizontal axis, cw (+)tive
    self.t = t_start

  def update(self,position,velocity,heading):
    self.position_enu = position
    self.velocity_enu = velocity
    angle = heading - np.pi / 2
    self.heading = -np.arctan2(np.sin(angle), np.cos(angle))

    self.Set_Position(position)
    self.Set_Velocity(velocity)




#================================================= CLASS FLOW VELOCITY COMPUTATION ==============================


class Flow_velocity(object):
    def __init__(self,vehicleList,panelList,obstacles):
        self.vehicle_list = vehicleList
        self.panel_list = panelList
        self.obstacles = obstacles
        self.pas = 0.5
        self.time = 0.5
        #self.velocity = self.flow_velocity(self.vehicle_list,self.panel_list)
        self.path_node = {}
        self.path_segment = {}
        for vehicle in self.vehicle_list:
            self.path_node[vehicle.ID] = []
            self.path_segment[vehicle.ID] = []
        
        self.set_path_node()
        
    # Flow velocity computation
    def flow_velocity(self,vehicles,buildings):
        """
        

        Parameters
        ----------
        vehicles : Dictionary
            A dictionary which contains drones identified by their ID
            
        buildings : TYPE
            DESCRIPTION.

        Returns
        -------
        flow_vels : Matrix
            Matrix which represent a flow velocity of each point

        """
        
        for f,vehicle in enumerate(vehicles):
            othervehicleslist = vehicles[:f] + vehicles[f+1:]
            vehicle.altitude_mask = np.zeros(( len(buildings) )) #, dtype=int) 
            for index,panelledbuilding in enumerate(buildings):
                if (panelledbuilding.vertices[:,2] > vehicle.altitude).any():
                    vehicle.altitude_mask[index] = 1
            related_buildings = list(compress(buildings,vehicle.altitude_mask))

            for building in related_buildings:
                #building.gamma_calc(vehicle,othervehicleslist)
                building.Vortex_strength(vehicle,othervehicleslist)
        
        flow_vels = np.zeros([len(vehicles),2])
        V_gamma   = np.zeros([len(vehicles),2]) # Velocity induced by vortices
        V_sink    = np.zeros([len(vehicles),2]) # Velocity induced by sink element
        V_source  = np.zeros([len(vehicles),2]) # Velocity induced by source elements
        V_sum     = np.zeros([len(vehicles),2]) # V_gamma + V_sink + V_source          
        V_normal  = np.zeros([len(vehicles),2]) # Normalized velocity
        V_flow    = np.zeros([len(vehicles),2]) # Normalized velocity inversly proportional to magnitude
        V_norm    = np.zeros([len(vehicles),1]) # L2 norm of velocity vector
        
        for f,vehicle in enumerate(vehicles):
            othervehicleslist = vehicles[:f] + vehicles[f+1:]

            # Velocity induced by 2D point sink, eqn. 10.2 & 10.3 in Katz & Plotkin:
            V_sink[f,0] = (-vehicle.sink_strength*(vehicle.position[0]-vehicle.goal[0]))/(2*np.pi*((vehicle.position[0]-vehicle.goal[0])**2+(vehicle.position[1]-vehicle.goal[1])**2))
            V_sink[f,1] = (-vehicle.sink_strength*(vehicle.position[1]-vehicle.goal[1]))/(2*np.pi*((vehicle.position[0]-vehicle.goal[0])**2+(vehicle.position[1]-vehicle.goal[1])**2))
            # Velocity induced by 3-D point sink. Katz&Plotkin Eqn. 3.25
            #W_sink[f,0] = (-vehicle.sink_strength*(vehicle.position[2]-vehicle.goal[2]))/(4*np.pi*(((vehicle.position[0]-vehicle.goal[0])**2+(vehicle.position[1]-vehicle.goal[1])**2+(vehicle.position[2]-vehicle.goal[2])**2)**1.5))

        # Velocity induced by 2D point source, eqn. 10.2 & 10.3 in Katz & Plotkin:
            
            for othervehicle in othervehicleslist:
                pass
                #V_source[f,0] = (othervehicle.source_strength*(vehicle.position[0]-othervehicle.position[0]))/(2*np.pi*((vehicle.position[0]-othervehicle.position[0])**2+(vehicle.position[1]-othervehicle.position[1])**2))
                #V_source[f,1] = (othervehicle.source_strength*(vehicle.position[1]-othervehicle.position[1]))/(2*np.pi*((vehicle.position[0]-othervehicle.position[0])**2+(vehicle.position[1]-othervehicle.position[1])**2))
                #W_source[f,0] += (othervehicle.source_strength*(vehicle.position[2]-othervehicle.position[2]))/(4*np.pi*((vehicle.position[0]-othervehicle.position[0])**2+(vehicle.position[1]-othervehicle.position[1])**2+(vehicle.position[2]-othervehicle.position[2])**2)**(3/2))
            
            for building in buildings:
                u = np.zeros((building.nop,1))
                v = np.zeros((building.nop,1))
                if vehicle.ID in building.gamma.keys():
                        global a, b, c, d, e # Velocity induced by vortices on each panel: 
                        u = (building.gamma[vehicle.ID][:].T/(2*np.pi))  *((vehicle.position[1]-building.pcp[:,1]) /((vehicle.position[0]-building.pcp[:,0])**2+(vehicle.position[1]-building.pcp[:,1])**2)) ####
                        v = (-building.gamma[vehicle.ID][:].T/(2*np.pi))  *((vehicle.position[0]-building.pcp[:,0]) /((vehicle.position[0]-building.pcp[:,0])**2+(vehicle.position[1]-building.pcp[:,1])**2))
                        V_gamma[f,0] = V_gamma[f,0] + np.sum(u)                         
                        V_gamma[f,1] = V_gamma[f,1] + np.sum(v)
                        
                        a = building.gamma[vehicle.ID]
                        b = building.pcp
                        c = u
                        d = v
                        e = V_gamma

            # Total velocity induced by all elements on map:
            V_sum[f,0] = V_gamma[f,0] + V_sink[f,0] + vehicle.V_inf[0] + V_source[f,0]
            V_sum[f,1] = V_gamma[f,1] + V_sink[f,1] + vehicle.V_inf[1] + V_source[f,1]

            # L2 norm of flow velocity:
            V_norm[f] = (V_sum[f,0]**2 + V_sum[f,1]**2)**0.5
            # Normalized flow velocity:
            V_normal[f,0] = V_sum[f,0]/V_norm[f]
            V_normal[f,1] = V_sum[f,1]/V_norm[f]

            # Flow velocity inversely proportional to velocity magnitude:
            V_flow[f,0] = V_normal[f,0]/V_norm[f]
            V_flow[f,1] = V_normal[f,1]/V_norm[f]
        
            flow_vels[f,:] = [V_flow[f,0],V_flow[f,1]]
        
        return flow_vels
    
    
    # Define nodes
    def set_path_node(self):
            """
            state_list = []
            for vehicle in self.vehicle_list:
                
                node = (vehicle.position[0],vehicle.position[1])
                self.path_node[vehicle.ID].append(node)
                state_list.append(vehicle.state)
            
            # define a polygon in which the goal point is centered
            #coord = [(vehicle.goal[0]-self.pas,vehicle.goal[1]-self.pas),(vehicle.goal[0]+self.pas,vehicle.goal[1]-self.pas),(vehicle.goal[0]+self.pas,vehicle.goal[1]+self.pas),(vehicle.goal[0]-self.pas,vehicle.goal[1]+self.pas)]
            #polyGoal = Polygon(coord)
            #end = (vehicle.goal[0],vehicle.goal[1])
            
            #pnt = Point(node)
            #print('\nEtat2:',vehicle.state)
            #print('distane_to_goal:',vehicle.distance_to_destination)
            #print(vehicle.position)
            #print(vehicle.goal)
            #print(np.linalg.norm(np.array(vehicle.goal)-np.array(vehicle.position)))
            while sum(state_list) != len(self.vehicle_list): 
                
                #print('\ninterieur de la boucle')
                #flow_vel = self.flow_velocity(self.vehicle_list,self.panel_list)
                
                for vehicle in self.vehicle_list:
                    
                    if vehicle.state != 1:
                        flow_vel = self.flow_velocity(self.vehicle_list,self.panel_list)
                        vspeed=(flow_vel[vehicle.ID-1]/np.linalg.norm(flow_vel[vehicle.ID-1]))   # Norm of velocity
                        #print(vspeed)
                #vspeed = flow_vel[vehicle.ID-1]
                
                # update vehicle's position
                #vehicle.position[0] = vehicle.position[0] + vspeed[0]*self.time
                #vehicle.position[1] = vehicle.position[1] + vspeed[1]*self.time
                
                        position_x = vehicle.position[0] + vspeed[0]*self.time
                        position_y = vehicle.position[1] + vspeed[1]*self.time
                
                        vehicle.Set_Position([position_x,position_y,0])    # Update of the vehicle's position
                
                # Add to node list
                        node = (position_x,position_y)
                        self.path_node[vehicle.ID].append(node)
                        
                    else:
                        state_list[vehicle.ID - 1] = vehicle.state
                    print("\nON PASSE A UN AUTRE VEHICULE")
                # somme des etats des vehicules
                
            for vehicle in self.vehicle_list:
                 node = (vehicle.goal[0],vehicle.goal[1])
                 self.path_node[vehicle.ID].append(node)
             #self.path_node[.append((vehicle.goal[0],vehicle.goal[1]))
            """
            # ------------------------------ ESSAI ----------------------------
            
            for vehicle in self.vehicle_list:
                node = (vehicle.position[0],vehicle.position[1])
                self.path_node[vehicle.ID].append(node)
                
                while vehicle.state != 1:
                    flow_vel = self.flow_velocity(self.vehicle_list,self.panel_list)[vehicle.ID - 1]
                    vspeed=(flow_vel/np.linalg.norm(flow_vel)) 
                    position_x = vehicle.position[0] + vspeed[0]*self.time
                    position_y = vehicle.position[1] + vspeed[1]*self.time
                    vehicle.Set_Position([position_x,position_y,0])
                    node = (position_x,position_y)
                    self.path_node[vehicle.ID].append(node)
                    
                node = (vehicle.goal[0],vehicle.goal[1])
                self.path_node[vehicle.ID].append(node)
                
           
            
            #-------------------------------- SEGDMENT---------------------------
            for vehicle in self.vehicle_list:
                for i,node in enumerate(self.path_node[vehicle.ID]):
                    if i<len(self.path_node[vehicle.ID])-1:
                        self.path_segment[vehicle.ID].append((node,self.path_node[vehicle.ID][i+1]))
        
        
