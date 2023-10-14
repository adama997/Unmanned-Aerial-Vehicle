#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Mar 24 19:45:30 2023

@author: robot
"""

"""

Implementation of the 2nd version of A* Algorithm

"""

import csv
from graph import Toulouse, dico_Toulouse


class Astar(object):
    
    def __init__(self,start,end,graph): #start = id du point de départ #end=id du point d'arrivée
        self.start = dico_Toulouse[start]
        self.end = dico_Toulouse[end]
        self.graph = graph.nodes
        self.path_segment = []
        self.distance,self.previous = self.astar(self.graph,self.start,self.end)
        self.path = self.Path(self.previous,self.start,self.end)
       
        
    # Graph implementation
    def Graph(self, filename):
        """

        Parameters
        ----------
        filename : FILE
            File which contains vertices of the graph and the neighbors of these vertices

        Returns
        -------
        graph : DICTIONARY
            Graph wich will be used for the A* algorithm

        """
    
    # Heuristic function
    def heuristic(self,node,end,graph):
        """
        Parameters
        ----------
        node : NODE , VERTICE
            It's a vertice of the graph'
        end : NODE
            The END point 
        graph : DICTIONARY
            The graph that will be used

        Returns
        -------
        remaining_edges : INT
            A approximative distance between the node and the end point, it is called sometimes h-score

        """
        remaining_edges = 0
        for neighbor in graph[node]:
            if neighbor != end:
                remaining_edges += 1
        return remaining_edges
    
    # Astar function
    def astar(self,graph,start,end):
        """
        Parameters
        ----------
        graph : DICTIONARY
    
        start : NODE
            The start point 
        end : NODE 
            The end point

        Returns
        -------
        distances : DICTIONARY
            A dictionary wich each node is associated a weight which is a distance between the current node and the start node 
        previous : LIST
            The shortest path in reversed order

        """
        distances = {node: float('inf') for node in graph}
        distances[start] = 0
        unvisited = set(graph.keys())
        previous = {node: None for node in graph}
        current = start
        while unvisited:
            for neighbor in graph[current]:
                new_dist = distances[current] + graph[current][neighbor]
                if new_dist < distances[neighbor]:
                    distances[neighbor] = new_dist
                    previous[neighbor] = current
            unvisited.remove(current)
            if current == end:
                break
            candidates = {node: distances[node] + self.heuristic(node, end, graph) for node in unvisited}
            current = min(candidates, key=candidates.get)
        return distances, previous
    
    # Shortest path function implementation
    def Path(self,previous,start,end):
        """

        Parameters
        ----------
        previous : LIST
            The shortest path in reversed order
            
        start : NODE
            The start point 
        end : NODE
            The end point 

        Returns
        -------
        path : LIST
            The shortest path between start node and end node

        """
        node = end
        path = [node.code]
        while node != start:
            parent = node
           # current = self.graphe[node]
            node = previous[node]
            
            """
            done = False
            # = node
            while not done:
                if not check_path(self.obstacle,parent,node):
                    #path.append(parent)
                    #self.path_segment.insert(0,(node,parent))
                   # print(path)
                    path.append(parent)
                    done = True
                    
                parent = node_gen(self.obstacle,parent)
            
            #path.append(parent)
            path.append(node)
            if previous[node] == self.start:
                path.append(self.start)
                node = self.start
            
            
            #self.path_segment.insert(0,(node,parent))
            #print("segment\n",self.path_segment)
            
        #path.reverse()
         
        
        k = 0
        while k < len(path)-1:
            self.path_segment.insert(k,(path[k],path[k+1]))
            k +=1
        """
        
            path.append(node.code)
            self.path_segment.insert(0,(node.code,parent.code))
            
        path.reverse()
        
        return path

