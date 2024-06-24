import random
from math import pow,sqrt
from TSP_Data import RAYON
# !/usr/bin/python3
# -*- coding: utf-8 -*-

class Etat:
    def __init__(self, dim_etat, data):
        self.vecteur = []
        self.dim_etat = dim_etat
        self.data = data
        self.Poids = None
        self.Valeur = None
        self.Poids_max = 2000
        self.distance_tsp = None
        self.radius = 5  # for plot the points
        self.scale = self.radius/RAYON  # coefficient for the conversion
        self.sep_distance = None
        self.aircraft_rank = 4


    def init_aleatoire(self):
        """Initialisation aléatoire de l'état."""
        # Fonction triviale
        # for i in range(self.dim_etat):
        #    self.vecteur.append(random.randint(0,1))

        # Sac à dos
        # for i in range(self.dim_etat):
        #    self.vecteur.append(random.randint(0,self.dim_etat-1))

        # TSP
        for i in range(self.dim_etat):
           self.vecteur.append(i)
        for i in range(50):
           self.generer_voisin()

        # Séquencement
        #for i in range(self.dim_etat):
        #    self.vecteur.append(i)

    def afficher(self):
        """Affichage."""
        # Fonction triviale
        # str_vecteur = ''.join(map(str,self.vecteur))
        # return "L'état est = \n{}".format(str_vecteur)

        # Sac à dos
        # str_vecteur = ''.join(map(str, self.vecteur))
        # return "L'état est = \n{}\nPoids = {}\nValeur = {}\n".format(str_vecteur, self.Poids, self.Valeur)

        # TSP
        str_vecteur = '  '.join(map(str, self.vecteur))
        return "L'état est = \n{}\nDistance TSP = {}\n".format(str_vecteur, self.distance_tsp)

        # Séquencement avions
        # str_vecteur = '  '.join(map(str, self.vecteur))
        # categorie = '  '.join(map(str,[self.data.tab_avions[avion].categorie for avion in self.vecteur]))
        # return "L'état est = \n{}\nLa Catégorie est = \n{}\n".format(str_vecteur,categorie)


    def generer_voisin(self):
        """Générer un état voisin."""
        # Fonction triviale
        # pos = random.randint(0,self.dim_etat-1)
        # self.vecteur[pos] = (self.vecteur[pos] + 1)%2

        # Sa à dos
        # pos1 = random.randint(0,self.dim_etat - 1)
        # pos2 = random.randint(0,self.dim_etat - 1)
        # var_temp = self.vecteur[pos1]
        # self.vecteur[pos1] = self.vecteur[pos2]
        # self.vecteur[pos2] = var_temp

        # TSP
        pos1 = random.randint(0, self.dim_etat - 1)
        pos2 = random.randint(0, self.dim_etat - 1)
        var_temp = self.vecteur[pos1]
        self.vecteur[pos1] = self.vecteur[pos2]
        self.vecteur[pos2] = var_temp

        # Séquencement avions
        # pos1 = random.randint(0, self.dim_etat-1)
        # k = random.randint(0, 2*self.aircraft_rank) - self.aircraft_rank
        # pos2 = max(min(self.vecteur[pos1]+k, self.dim_etat-1), 0)
        # if (self.vecteur[pos2] - self.aircraft_rank <= pos1) and (pos1 <= self.vecteur[pos2] + self.aircraft_rank):
        #     var_temp = self.vecteur[pos1]
        #     self.vecteur[pos1] = self.vecteur[pos2]
        #     self.vecteur[pos2] = var_temp


    def calcul_critere(self):
        """Évaluation des objectifs."""
        # Fonction triviale
        # sum = 0
        # for i in range(self.dim_etat):
        #    sum += self.vecteur[i]
        # return sum

        # Sac à dos
        # self.Poids,self.Valeur = 0,0
        # for i in range(self.dim_etat):
        #    self.Poids += self.data.tab_objets[i].poids*self.vecteur[i]
        #    self.Valeur += self.vecteur[i]*self.data.tab_objets[i].valeur
        # y = self.Valeur
        # delta = self.Poids_max - self.Poids
        # if delta < 0:
        #    y = y - 10*delta

        # return y

        # TSP
        self.distance_tsp = 0
        Xn_X0 = sqrt(pow((self.data.tab_villes[self.vecteur[self.dim_etat-1]].x - self.data.tab_villes[self.vecteur[0]].x),0) + pow((self.data.tab_villes[self.vecteur[self.dim_etat-1]].y - self.data.tab_villes[self.vecteur[0]].y),2))
        for i in range(self.dim_etat-1):
            self.distance_tsp += sqrt(pow((self.data.tab_villes[self.vecteur[i+1]].x - self.data.tab_villes[self.vecteur[i]].x),2) + pow((self.data.tab_villes[self.vecteur[i+1]].y - self.data.tab_villes[self.vecteur[i]].y),2)) + Xn_X0

        return self.distance_tsp

        # Séquencement avions
        # self.sep_distance = 0
        # for i in range(self.dim_etat-1):
        #     self.sep_distance += self.data.separation[self.data.tab_avions[self.vecteur[i]].categorie][self.data.tab_avions[self.vecteur[i+1]].categorie]
        #
        # return self.sep_distance


    def copy(self, copy_from):
        self.vecteur = copy_from.vecteur[:]
        self.dim_etat = copy_from.dim_etat
        self.data = copy_from.data
        """à completer si necesaire"""

    def coordinate(self):
        res = {
            city:(self.data.tab_villes[city].x*self.scale,self.data.tab_villes[city].y*self.scale)
            for city in self.vecteur
        }
        return res