
# !/usr/bin/python3
# -*- coding: utf-8 -*-

import random
import math

# Paramètres de génération.
RAYON = 100.0


class Ville:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __str__(self):
        return "(" + str(self.x) + ", " + str(self.y) + ")"

    def distance(self, other):
        return math.sqrt((self.x - other.x) ** 2 + (self.y - other.y) ** 2)


class Data:
    # TSP problème

    def generer_villes_cercle(self, n):
        """Génération de n villes sur le cercle (TSP)."""
        self.tab_villes = []
        for i in range(n):
            theta = random.uniform(0, 2 * math.pi)
            x = RAYON * math.cos(theta)
            y = RAYON * math.sin(theta)
            self.tab_villes.append(Ville(x, y))


    def afficher_villes(self):
        """Affichage des villes."""
        print("*** Villes ***")
        for i, ville in enumerate(self.tab_villes):
            print("Ville", i, ":", str(ville))


    def __init__(self, dim):
        self.generer_villes_cercle(dim)
        self.afficher_villes()