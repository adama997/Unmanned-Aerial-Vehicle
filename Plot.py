#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Apr 14 22:22:44 2023

@author: robot
"""

# Plot Drones
import matplotlib.pyplot as plt


class Plot(object):
    
    def __init__(self,graph,obstacles,rrt_segment=None,panel_segment=None):
        self.graph = graph
        self.obstacle = obstacles
        #self.path = nodes
        self.rrt_path_segment = rrt_segment
        self.panel_path_segment = panel_segment
        #self.algorithm = algorithm
        
        self.plot = self.plot()
        
    
    def plot(self):
        
        FIG, AX = plt.subplots(nrows=1, ncols=1, sharex=True, sharey=True, figsize=(15, 15))
        
        plt.title(" CAS MULTI-DRONES")
        plt.xlabel("Axe X")
        plt.ylabel("Axe Y")
        plt.axis([-6,15,-1,20])
        plt.grid(True)
        
        # Plot the graph
        for edge in self.graph.edges:
            a=edge.orig
            b=edge.dest
            X_graph=[a.x,b.x]
            Y_graph=[a.y,b.y]
            plt.plot(X_graph,Y_graph,color='blue')
            
        for node1 in self.graph.nodes:
            for node2 in self.graph.nodes[node1]:
                X=[node1.x,node2.x]
                Y=[node1.y,node2.y]
                #plt.plot(X,Y,label=str(self.nodes[n1][n2]))#affiche le poids de l'arête (n1,n2)
                plt.text(sum(X)/2-0.3,sum(Y)/2-0.2,str(self.graph.nodes[node1][node2]),fontsize=5.5)
        
        for elt in self.graph.nodes.keys():
            x,y = elt.x,elt.y
            plt.scatter(x,y,s=50,color='blue')
            
        
        # Plot Obstacles
        for obs in self.obstacle:
            
            if obs[0] == 'Circle':
                center = obs[1][0]
                radius = obs[1][1]
                circle = plt.Circle(center,radius,color='yellow')
                AX.add_patch(circle)
            else:
                polygon = obs[1]
                xp,yp = polygon.exterior.xy
                plt.plot(xp,yp,color='yellow')
        
        
        #Plot the path
        
        if self.rrt_path_segment != None:
            for ID in self.rrt_path_segment.keys():
                start = self.rrt_path_segment[ID][0][0] 
                end = self.rrt_path_segment[ID][-1][1]
                plt.scatter(start[0],start[1],s=90,color='purple',label = 'RRT')
                plt.text(start[0],start[1],'start_{}'.format(ID),fontsize=15,color = 'purple')
                plt.scatter(end[0],end[1],s=90,color='purple')
                plt.text(end[0],end[1],'end_{}'.format(ID),fontsize=15,color = 'purple')
        
                for (i,j) in self.rrt_path_segment[ID]:
                    X = [i[0],j[0]]
                    Y = [i[1],j[1]]
                    plt.plot(X,Y,color='purple',linestyle='dashed',linewidth=1)
                    plt.pause(0.1)
                    
                plt.savefig("RRT_FIG.png")
        
        if self.panel_path_segment != None:
            for ID in self.panel_path_segment.keys():
                start = self.panel_path_segment[ID][0][0] 
                end = self.panel_path_segment[ID][-1][1]
                plt.scatter(start[0],start[1],s=90,color='green',label = 'Panel')
                plt.text(start[0],start[1]+0.5,'start_{}'.format(ID),fontsize=15,color='green')
                plt.scatter(end[0],end[1],s=90,color='green')
                plt.text(end[0],end[1]+0.5,'end_{}'.format(ID),fontsize=15,color='green')
            
                for (i,j) in self.panel_path_segment[ID]:
                    X = [i[0],j[0]]
                    Y = [i[1],j[1]]
                    plt.plot(X,Y,color='green',linestyle='dashed',linewidth=1)
                    plt.pause(0.1)
                    
                plt.savefig("PANEL_FIG.png")
        
        plt.legend(loc = 'upper right')
        
        plt.show()
        
        
        """
        for ID in self.path_segment.keys():
            
                    
                    if self.algorithm[ID] == 'astar':
                        start = self.path_segment[ID][0][0] 
                        end = self.path_segment[ID][-1][1]
                        plt.scatter(start[0],start[1],s=90,color='red',label = 'A*')
                        plt.text(start[0],start[1]+0.2,'start_{}'.format(ID),fontsize=15,color='red')
                        plt.scatter(end[0],end[1],s=90,color='red')
                        plt.text(end[0],end[1]+0.2,'end_{}'.format(ID),fontsize=15,color='red')
                        
                        for (i,j) in self.path_segment[ID]:
                             X = [i[0],j[0]]
                             Y = [i[1],j[1]]
                             plt.plot(X,Y,color='red',linestyle='dashed',linewidth=5)
                             plt.pause(0.05)
                        #plt.legend(loc = 'upper right')
                        
                        start = get_node()[self.path_segment[ID][0][0]] 
                        end = get_node()[self.path_segment[ID][-1][1]]
                        plt.scatter(start.x,start.y,s=60,color='red')
                        plt.text(start.x,start.y,'start_{}'.format(ID),fontsize=15)
                        plt.scatter(end.x,end.y,s=60,color='red')
                        plt.text(end.x,end.y,'end_{}'.format(ID),fontsize=15)
                        
                        for (i,j) in self.path_segment[ID]:
                           X = [get_node()[i].x,get_node()[j].x]
                           Y = [get_node()[i].y,get_node()[j].y]
                           plt.plot(X,Y,color='red')
                           plt.pause(0.05)
                           
                           
                    elif self.algorithm[ID] == 'rrt':
                        start = self.path_segment[ID][0][0] 
                        end = self.path_segment[ID][-1][1]
                        plt.scatter(start[0],start[1],s=90,color='purple',label = 'RRT')
                        plt.text(start[0],start[1],'start_{}'.format(ID),fontsize=15,color = 'purple')
                        plt.scatter(end[0],end[1],s=90,color='purple')
                        plt.text(end[0],end[1],'end_{}'.format(ID),fontsize=15,color = 'purple')
                    
                        for (i,j) in self.path_segment[ID]:
                            X = [i[0],j[0]]
                            Y = [i[1],j[1]]
                            plt.plot(X,Y,color='purple',linestyle='dashed',linewidth=5)
                            plt.pause(0.05)
                        #plt.legend(loc = 'upper right')
                        
                            x,y = elt[0],elt[1]
                            plt.scatter(x,y,s=50,color='red')
                            if m > 0:
                                    path_seg = mpl.collections.LineCollection(self.path_segment[ID][m-1:m], colors='red')
                                    AX.add_collection(path_seg)
                                    plt.pause(0.05)
                            
                    else:
                        start = self.path_segment[ID][0][0] 
                        end = self.path_segment[ID][-1][1]
                        plt.scatter(start[0],start[1],s=90,color='green',label = 'Panels Method')
                        plt.text(start[0],start[1]+0.5,'start_{}'.format(ID),fontsize=15,color='green')
                        plt.scatter(end[0],end[1],s=90,color='green')
                        plt.text(end[0],end[1]+0.5,'end_{}'.format(ID),fontsize=15,color='green')
                        
                        for (i,j) in self.path_segment[ID]:
                            X = [i[0],j[0]]
                            Y = [i[1],j[1]]
                            plt.plot(X,Y,color='green',linestyle='dashed',linewidth=5)
                            plt.pause(0.05)
                        #plt.legend(loc = 'upper right')
                        
                        for i,node in enumerate(self.path[ID]):
                            x,y = node[0],node[1]
                            plt.scatter(x,y,s=20,color='green')
                            
                            if i>0:
                                path_seg = mpl.collections.LineCollection(self.path_segment[ID][i-1:i], colors='green')
                                AX.add_collection(path_seg)
                                plt.pause(0.05)
                        
            """
        