import numpy as np
import math as ma
import pyproj
from shapely.geometry import Polygon, Point
from tkinter import *
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('TkAgg')
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

from Panel_V01 import Panels

R_TERRE = 6371 *10**3 # on va travailler en m

#define a projection with proj4 notation
isn2004=pyproj.CRS("+proj=lcc +lat_1=64.25 +lat_2=65.75 +lat_0=65 +lon_0=-19 +x_0=1700000 +y_0=300000 +no_defs +a=6378137 +rf=298.257222101 +to_meter=1") 
#(lat,long) with wgs84 datum used by google earth
wgs84=pyproj.CRS("EPSG:4326")

#Largeur des corridors
LARGEUR_CORRIDORS=500

class Error(Exception):
    pass

################################ Node ################################
class Node:
    def __init__(self, lat0, long0, code0, nom):
        self.adj = {} # {voisin : poids=distance}
        self.lat=float(lat0)
        self.long=float(long0)
        #self.x,self.y=lat,long
        x_ref,y_ref = 3414030.8119050264, -1861082.14248893
        self.xp,self.yp = pyproj.transform(wgs84,isn2004,self.lat,self.long)
        self.x = (self.xp - x_ref)/1000
        self.y = (self.yp - y_ref)/1000
        self.code=int(code0)
        self.name=nom
        self.coords=(self.x,self.y)
        self.coins=[] # pour construire les liaisons entre les corridors

    def distance(self,other):
            return ma.sqrt((self.x-other.x)**2+(self.y-other.y)**2) # en m
    
    '''
    def add(self, v, w): # ajoute un voisin et son poids
        self.adj[v] = w
        
    def remove(self, v): #enlève un voisin
        self.adj.pop(v)        
    
    def is_adj(self, v): #indique si le sommet v est voisin
        return v in self.adj
    
    def list_weight(self, v): #donne le poids à v
        return self.adj[v]
    
    def neighbours(self): #affiche le dictonnaire des voisins
        return self.adj
    '''
    
    def __repr__(self):
        return '<Node: {0.name} {0.x} {0.y}>'.format(self)
    
def load_nodes(filename):
    '''
    load_nodes prend en argument le fichier csv des sommets retenus de Toulouse.
    Cette fonction renvoie une liste des sommets et un dictionnaire qui à un couple de coordonnées (latitude,longitude) associe le sommet correspondant.
    '''
    res=[] #liste des sommets
    dic_coords={} #dictionnaire {coords : sommet}
    dic_code = {} #dictionnaire {code : sommet}
    with open(filename,'r') as f:
        line = f.readline() #lit la 1ère ligne
        line = f.readline() #lit la deuxième ligne
        line = line.strip().split(',')[0:4]
        while len(line)==4:
            p=Node(line[1],line[0],line[2],line[3])
            res.append(p)
            line = f.readline() # readline() pour lire la ligne suivante
            line = line.strip().split(',')[0:4]
            dic_coords[p.coords]=p
            dic_code[p.code]=p           
    return res,dic_code

def search_node(node_list,x,y):
    for n in node_list:
        if n.x==x and n.y==y:
            return n
    return None

#N_Toulouse,dico_Toulouse=load_nodes("CSV\PIR_Drone_simplifie.csv")


################################ Zone ################################
class Zone:
    def __init__(self,type0,p0,r=None,e=None,coins=[]):
        self.type=type0 # "circle" pour cercle / "poly" pour polygone / "corridor"
        self.pp=[Point(pyproj.transform(wgs84,isn2004,float(i[0]),float(i[1]))) for i in p0] # Liste des points de références 
        x_ref,y_ref = 3414030.8119050264, -1861082.14248893
        self.p = [Point(((i.x-x_ref)/1000,(i.y-y_ref)/1000)) for i in self.pp]
        self.radius=r
        self.thickness=e
        self.coins=coins
        if r!=None:
            self.radius=float(r)
            self.poly=None
        else:
            if self.type=="poly":
                self.poly=Polygon(self.p)
            else:
                if self.type=="corridor":
                    self.couloir()
                '''
                else : #c'est une liaison
                    self.coins=coins
                    if len(self.coins)>2:
                        self.poly=Polygon(self.coins)
                '''
    
    def couloir(self): # construit le corridor à partir de l'arête
        Coins=[]
        pt1=self.p[0]
        pt2=self.p[1]
        node1=search_node(N_Toulouse,pt1.x,pt1.y)
        node2=search_node(N_Toulouse,pt2.x,pt2.y)
        #Construction de la droite représentant l'arête
        a=(pt2.y-pt1.y)/(pt2.x-pt1.x)
        b=(pt1.y-a*pt1.x)
        #Construction de la droite orthogonale passant par pt1
        c=-1/a
        d=pt1.y-c*pt1.x
        #Construction de la droite orthogonale passant par pt2
        e=-1/a
        f=pt2.y-e*pt2.x
        #Coins issus de pt1
        delta=(2*c*d-2*pt1.x-2*c*pt1.y)**2-4*(1+c**2)*(pt1.x**2+pt1.y**2-2*d*pt1.y+d**2-self.thickness**2/4)
        if delta>=0:
            xsol_1=(2*c*pt1.y+2*pt1.x-2*c*d-ma.sqrt(delta))/(2*(1+c**2))
            xsol_2=(2*c*pt1.y+2*pt1.x-2*c*d+ma.sqrt(delta))/(2*(1+c**2))
        else:
            xsol_1=(2*c*pt1.y+2*pt1.x-2*c*d-ma.sqrt(-delta))/(2*(1+c**2))
            xsol_2=(2*c*pt1.y+2*pt1.x-2*c*d+ma.sqrt(-delta))/(2*(1+c**2))
        Coins.append(Point(xsol_1,c*xsol_1+d))
        Coins.append(Point(xsol_2,c*xsol_2+d))
        node1.coins.append(Point(xsol_1,c*xsol_1+d)) #pour constuire les liaisons entre les corridors
        node1.coins.append(Point(xsol_2,c*xsol_2+d))
        
        #Coins issus de pt2
        delta=(2*e*f-2*pt2.x-2*e*pt2.y)**2-4*(1+e**2)*(pt2.x**2+pt2.y**2-2*f*pt2.y+f**2-self.thickness**2/4)
        if delta>=0:
            xsol_3=(2*e*pt2.y+2*pt2.x-2*e*f+ma.sqrt(delta))/(2*(1+e**2))
            xsol_4=(2*e*pt2.y+2*pt2.x-2*e*f-ma.sqrt(delta))/(2*(1+e**2))
        else:
            xsol_3=(2*e*pt2.y+2*pt2.x-2*e*f+ma.sqrt(-delta))/(2*(1+e**2))
            xsol_4=(2*e*pt2.y+2*pt2.x-2*e*f-ma.sqrt(-delta))/(2*(1+e**2))
        Coins.append(Point(xsol_3,e*xsol_3+f))
        Coins.append(Point(xsol_4,e*xsol_4+f))
        node2.coins.append(Point(xsol_3,e*xsol_3+f))
        node2.coins.append(Point(xsol_4,e*xsol_4+f))
        self.p=Coins
        self.poly=Polygon(self.p)
        
        
    def in_zone(self, coord): #renvoie si le point de coordonnées coord (x,y) est dans le polygone
        cible=Point(coord)
        if self.type=="circle" or self.type=="liaison":
            return ma.sqrt((self.p[0].x-cible.x)**2+(self.p[0].y-cible.y)**2)<=self.radius
        else:
            return self.poly.contains(cible)
        
    def __repr__(self):
        return '<Type : {0.type}, Points de références : {0.p}, Rayon : {0.radius} >'.format(self)
        
    
        
def load_zones(filename,type):
    '''
    load_zones prend en argument le fichier csv représentant la zone.
    Cette fonction crée la zone.
    '''
    Z=[]
    p0=[]
    with open(filename,'r') as f:
        if type=="circle":
            line = f.readline() # readline() pour lire la ligne suivante
            line=line.strip()
            line=line.split(',')
            while len(line)==5:
                p0=[]
                p0.append((line[1],line[0])) #on fait la conversion en x et y au moment de la création de la zone
                Z.append(Zone(type,p0,line[4])) #il faut rajouter un élément dans le fichier csv correspondant au rayon
                line = f.readline() # readline() pour lire la ligne suivante
                line=line.strip()
                line=line.split(',')
            return Z
        else:
            line = f.readline() # readline() pour lire la ligne suivante
            line=line.strip()
            line=line.split(',')
            #print(line)
            while len(line)==5:
                p0.append((line[1],line[0])) #on fait la conversion en x et y au moment de la création de la zone       
                line = f.readline() # readline() pour lire la ligne suivante
                line = line.strip()
                line=line.split(',')          
            return Zone(type,p0)
        
        
        
def load_corridors(filename, epaisseur):
    '''
    load_corridors prend en argument le fichier csv représentant les extrémités des arêtes.
    Cette fonction crée les corridors.
    '''
    C=[]
    V={}
    with open(filename,'r') as f:
            line = f.readline() # readline() pour lire la ligne suivante
            line=line.strip()
            line=line.split(',')
            #print(line)
            while len(line)==3:
                p0=[]
                code_pt1=float(line[0])
                code_pt2=float(line[1])
                i=0
                while len(p0)<2 and i<len(N_Toulouse):
                    n=N_Toulouse[i]
                    if n.code==code_pt1:
                        p0.append((n.lat,n.long))
                    else:
                        if n.code==code_pt2:
                            p0.append((n.lat,n.long))
                    i+=1
                #print(p0)
                c0=Zone("corridor",p0,r=None,e=epaisseur)
                #print(c0.p)
                C.append(c0)      
                line = f.readline() # readline() pour lire la ligne suivante
                line = line.strip()
                line=line.split(',')     
            return C

#Corridors de Toulouse      
#C_Toulouse=load_corridors("CSV\Corridors.csv",LARGEUR_CORRIDORS)

#Zones de Toulouse
"""
Z_Toulouse=[i for i in load_zones("CSV\zones_touristiques.csv","circle")]
Z_Toulouse.append(load_zones("CSV\Zone_P_Blagnac.csv","poly"))
Z_Toulouse.append(load_zones("CSV\Zone_P_Francazal.csv","poly"))
Z_Toulouse.append(load_zones("CSV\Zone_P_Lasbordes.csv","poly"))
Z_Toulouse.append(load_zones("CSV\Zone_P_Teleo.csv","poly"))
Z_Toulouse.append(load_zones("CSV\Zone_P_Centre_ville.csv","poly"))
Z_etude_dessin=load_zones("CSV\Zone_Etude_dessin.csv","poly") 
Z_etude=load_zones("CSV\Zone_Etude.csv","circle") # Il faut y être dedans

#Liaisons entre les corridors de Toulouse
L_Toulouse=[] #liste des liaisons entre les corridors
for n in N_Toulouse:
    if len(n.coins)>2:
        #print(n.name)
        #print(n.coins)
        L_Toulouse.append(Zone("Liaison", [(n.lat,n.long)], r=LARGEUR_CORRIDORS/2, coins=n.coins))


"""
################################ Graphe ################################
class Edge:
    def __init__(self,u,v):
        self.orig=u #ce sont des noeuds
        self.dest=v
        self.distance=ma.floor(u.distance(v))
        self.weight=self.distance
        self.nb_passages=0
        self.trafic=0
        self.trafic_max=0
        self.last_drone=None
        self.largeur=LARGEUR_CORRIDORS
        
    def modif_weight(self):
        if self.last_drone!=None:
            d_last_drone=ma.sqrt((self.last_drone.x-self.orig.x)**2+(self.last_drone.y-self.orig.y)**2)
            self.weight=self.distance+(self.distance-d_last_drone)**2/(self.distance*self.largeur)
        else:
            self.weight=self.distance
    
    def __repr__(self):
        return '<Edge: {0.orig.code} {0.dest.code}>'.format(self)



class Graph:
    def __init__(self):
        self.nodes = {} # {sommet : {voisins : poids}}
        self.edges=[] #[ liste des objets arrêtes}
        self.codes=[] #[liste des codes des sommets du graphs]
        
    def add_node(self, u): #ajoute un noeud au graphe (initialement isolé des autres)
        self.nodes[u] = {}
        self.codes.append(u.code)
        
    def is_node_in(self, u): #indique si u est un noeud du graphe
        return u in self.nodes
    
    def add_edge(self, u, v):
        '''Ajoute l'arrête de u à v avec le poids correspondant'''
        e1=Edge(u,v)
        e2=Edge(v,u)
        self.nodes[u][v]= e1.distance
        self.nodes[v][u]= e2.distance
        self.edges.append(e1)
        self.edges.append(e2)
    
    def search_edge(self,u,v): # renvoie l'arête ayant pour origine u et extrémités v
        for e in self.edges:
            if e.orig==u and e.dest==v:
                return e
        return None

    def remove_edge(self, u, v):
        '''Enlève l'arrête de u à v'''
        self.nodes[u].pop(v)
        self.nodes[v].pop(u)
        self.edges.pop(self.search_edge(u,v))

    def list_weight(self, u, v):
        return self.nodes[u]

    def neighbours(self, u):
        '''Renvoie la liste d'adjacence de u ({voisins : poids})'''
        return self.nodes[u]
    
    def trace_graphe(self,path_segment=None):
        for n1 in self.nodes:
            plt.plot(n1.x,n1.y,label=str(n1.code))
            plt.text(n1.x+50,n1.y+250,str(n1.code),fontsize=7,color='red')
            for n2 in self.nodes[n1]:
                X=[n1.x,n2.x]
                Y=[n1.y,n2.y]
                plt.plot(X,Y,label=str(self.nodes[n1][n2]))#affiche le poids de l'arête (n1,n2)
                plt.text(sum(X)/2-400,sum(Y)/2-200,str(self.nodes[n1][n2]),fontsize=5.5)
                if path_segment:
                    for (i,j) in path_segment:
                        i=dico_Toulouse[i]
                        j=dico_Toulouse[j]
                        X_path=[i.x,j.x]
                        Y_path=[i.y,j.y]
                        plt.plot(X_path,Y_path,color='red',linestyle='dashed',linewidth=3)
        plt.show()
    
    def __repr__(self):
        return "<Graph: {0.nodes}>".format(self)
    
    '''
    def shortest_path(self, orig, dest, critere, list_type):                        # list_type = ['plane', 'train', 'car', 'autoroute'
        #Donne le plus court chemin pour aller de orig à dest en fonction du critère (time, dist, co2) et du moyen de transport list_type ('car, 'plane' train)
        infini = 2**1000
        sommets_connus = {orig : [0, [orig]]}                                       # {sommet : [longueur, plus court chemin]  
        sommets_inconnus = {k : [infini, ()] for k in self.nodes if k!=orig}        # {sommet : [longeur, precedent]}
        for neighbour in self.neighbours(orig):
            if self.list_weight(orig, neighbour)[-1] in list_type:
                sommets_inconnus[neighbour] = [self.weight(orig, neighbour, critere), orig]
    
        while (dest not in sommets_connus) and sommets_inconnus and (any(sommets_inconnus[k][0] < infini for k in sommets_inconnus)):
            s_short = min(sommets_inconnus, key = sommets_inconnus.get)
            longueur_s, precedent_s = sommets_inconnus[s_short]
            for neighbour in self.neighbours(s_short):
                if neighbour in sommets_inconnus:
                    if self.list_weight(s_short, neighbour)[-1] in list_type:
                        d = longueur_s + self.weight(s_short, neighbour, critere)
                        if d < sommets_inconnus[neighbour][0]:
                            sommets_inconnus[neighbour] = [d, s_short]
            sommets_connus[s_short] = [longueur_s, sommets_connus[precedent_s][1] + [s_short]]
            del sommets_inconnus[s_short]

        if dest in sommets_connus:
            return sommets_connus[dest]
        else:
            raise Error("Il n'existe pas de chemin entre ces deux points")
    '''


def Toulouse():
    
    
    Toulouse=Graph()
    for i in N_Toulouse:
        Toulouse.add_node(i)
    with open("/home/robot/Documents/AVI/PIR_DRONES/CSV/Corridors.csv",'r') as f:
            line = f.readline() # readline() pour lire la ligne suivante
            line=line.strip()
            line=line.split(',')
            while len(line)==3:
                code_pt1=float(line[0])
                code_pt2=float(line[1])
                node1=dico_Toulouse[code_pt1]
                node2=dico_Toulouse[code_pt2]
                Toulouse.add_edge(node1,node2)
                line = f.readline() # readline() pour lire la ligne suivante
                line = line.strip()
                line=line.split(',')
    return Toulouse

#print(Toulouse.codes)
#print(Toulouse)
#Toulouse.trace_graphe()
#for e in Toulouse.edges:
#    print(e)

def get_code(point):
    graph = Toulouse()
    for node in graph.nodes.keys():
        if point[0] == node.x and point[1] == node.y:
            return node.code
    


    

    
    
    
def Obstacles(zone_list):

    # Display Obstacles
    
    obstacle = []                   # List of obstacles
    Obstacle_collection = []        # List of obstacles defined as collections
    panelList = []
    
    alpha = 0.5
    for zone in zone_list:
        if zone.type=='circle':
            X = zone.p[0].x
            Y = zone.p[0].y
            center = (X,Y)
            radius = zone.radius/1000
            obstacle.append(["Circle",[center,radius]])        # Add this obstacle in the list
            #circle = plt.Circle(center,radius,color='red')
            #AX.add_patch(circle)
           #Obstacle_collection.append(plt.Circle(center,radius))
           # plot.scatter(X,Y,s=zone.radius*3.8,color='red')
           
            #panel = [(X-radius,Y-radius),(X+radius,Y-radius),(X+radius,Y+radius),(X-radius,Y+radius)]
            
            panel = [(X+radius-alpha,Y-radius),(X+radius+alpha,Y+radius),(X-radius+alpha,Y+radius),(X-radius-alpha,Y-radius)]
            panelList.append(panel)     # Add pane to panel list
            
           
        else :
            X_list=[]
            Y_list=[]
            for p in zone.p:
                X = p.x
                Y = p.y
                #plot.scatter(X,Y,s=50,color='orange')
                X_list.append(X)
                Y_list.append(Y)
            X_list.append(X_list[0])
            Y_list.append(Y_list[0])
            
            #plt.plot(X_list,Y_list,color='orange')
            polygon = Polygon([(X_list[0],Y_list[0]),(X_list[1],Y_list[1]),(X_list[2],Y_list[2]),(X_list[3],Y_list[3])])
            obstacle.append(["Polygon",polygon])
           # xp,yp = polygon.exterior.xy
            #plt.plot(xp,yp,color='yellow')
            #Obstacle_collection.append(plt.plot(xp,yp))
            
            panel = [(X_list[0],Y_list[0]),(X_list[1],Y_list[1]),(X_list[2],Y_list[2]),(X_list[3],Y_list[3])]
            panelList.append(panel)       # Add panel to list
        
    Panel_List = []
    [Panel_List.append(Panels(panel)) for panel in panelList]
    
    return obstacle,Panel_List


def get_node():
    n_toulouse, dico_toulouse = load_nodes("/home/robot/Documents/AVI/PIR_DRONES/CSV/PIR_Drone_simplifie.csv")
    return dico_toulouse

  
    
N_Toulouse, dico_Toulouse = load_nodes("/home/robot/Documents/AVI/PIR_DRONES/CSV/PIR_Drone_simplifie.csv")

Z_Toulouse=[i for i in load_zones("/home/robot/Documents/AVI/PIR_DRONES/CSV/zones_touristiques.csv","circle")]
Z_Toulouse.append(load_zones("/home/robot/Documents/AVI/PIR_DRONES/CSV/Zone_P_Blagnac.csv","poly"))
Z_Toulouse.append(load_zones("/home/robot/Documents/AVI/PIR_DRONES/CSV/Zone_P_Francazal.csv","poly"))
Z_Toulouse.append(load_zones("/home/robot/Documents/AVI/PIR_DRONES/CSV/Zone_P_Lasbordes.csv","poly"))
Z_Toulouse.append(load_zones("/home/robot/Documents/AVI/PIR_DRONES/CSV/Zone_P_Teleo.csv","poly"))
Z_Toulouse.append(load_zones("/home/robot/Documents/AVI/PIR_DRONES/CSV/Zone_P_Centre_ville.csv","poly"))

#C_Toulouse=load_corridors("/home/robot/Documents/AVI/PIR_DRONES/CSV/Corridors.csv",LARGEUR_CORRIDORS)
#start = (get_node()[1].x,get_node()[1].y)  #Noeud de départ
#end = (get_node()[13].x,get_node()[13].y)
#print(dico_Toulouse[22],'\n')
#print(len(N_Toulouse),start,end,'\n')
#start = (get_node(13).x,get_node(13).y)
#print("\n",start)
"""
graph = Toulouse()
for node in graph.nodes.keys():
    print("\n",node.code)

print(get_node()[16].x,get_node()[35].y)
"""
