# Unmanned-Aerial-Vehicle
This repository contains some detect and avoid algorithms implemented for UAVs.

The use of UAVs increase fastly and toward this evolution, "sense&avoid" or "detect&avoid" methods are performed. Note that there generally 4 aspects of "detect&avoid".
-> Sensing techniques
-> Decision-making
-> Path planning 
-> Path following

In this project, we are focused on some paths planning methods such as some detect&avoid algorithms for path planning. 
Three (03) main algorithms are been implemented in this project.
-> RRT ("RRT_V01.py" : Rapidly-exploring Random Tree)
-> A* ("Astar_V02.py" : evolution of Djisktra with heuristic)
-> Panel method ("Panel_V01.py" : method which use fluid mechanic properties for path planning)

Alongside these algorithms which are implemented, there are in this project a visualization function ("Plot.py" : for plotting), a scenario function ("Scenario.py" : for describing operations scenario), a drone modeling function ("Drones_V01.py" : models the drone or UAV) and an oriented graph("graph.py" : for traffic simulation based on the architecture of Toulouse city - France) and "Function.py" for some use cases functions for python files described above.
