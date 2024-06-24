"""
This is an implementation of a simulated anealing algorithm. It's one of a most popular metaheuristics used in
global optimization.
"""

import random
from ivy.std_api import *
import  time
#!/usr/bin/python3
# -*- coding: utf-8 -*-

import Bag_Data as bag
import TSP_Data as tsp
import Sequencing_Data as seq

import State as etat
from math import exp

NB_TRANSITIONS = 2000
ALPHA = .995
IS_MINIMISATION = True

# Dimension du problème.
DIMENSION = 5


class Recuit:
    def __init__(self, data):
        self.data = data
        self.time = None
        self.temperature = ''
        self.criteria_value = ''
        self.city = ''
        self.tsp_radius = None

    def _accept(self, yi, yj, temperature):
        """Principe d'acceptation en maximisation et/ou minimisation."""
        result = False
        if IS_MINIMISATION:
            if yj < yi:
                result = True
            else:
                diff = yi - yj
                proba = exp(diff/temperature)
                tirage = random.random()
                if tirage < proba:
                    result = True
        else:
            if yj >= yi:
                result = True
            else:
                diff = yj - yi
                proba = exp(diff/temperature)
                tirage = random.random()
                if tirage < proba:
                    result = True

        return result


    def heat_up_loop(self):
        """Détermine la température initiale."""

        temperature = 0.01
        taux_acceptation = 0.0

        xi = etat.Etat(DIMENSION, self.data)
        xj = etat.Etat(DIMENSION, self.data)

        while taux_acceptation < 0.8:
            accept_count = 0

            for i in range(NB_TRANSITIONS):
                # Génération d'un point de l'espace d'état.
                xi.init_aleatoire()
                yi = xi.calcul_critere()

                # Génération d'un point voisin.
                xj.copy(xi)
                xj.generer_voisin()
                yj = xj.calcul_critere()

                if self._accept(yi, yj, temperature):
                    accept_count += 1

            taux_acceptation = accept_count / NB_TRANSITIONS
            temperature *= 1.5

        return temperature

    def cooling_loop(self, temperature_initiale):
        """Processus de refroidissement."""

        temperature = temperature_initiale

        xi = etat.Etat(DIMENSION, self.data)
        xi.init_aleatoire()
        yi = xi.calcul_critere()

        xj = etat.Etat(DIMENSION, self.data)
        self.tsp_radius = xi.radius  # radius got from state.py

        while temperature > 0.0001 * temperature_initiale:
            for i in range(NB_TRANSITIONS):
                xj.copy(xi)
                xj.generer_voisin()
                yj = xj.calcul_critere()

                if self._accept(yi, yj, temperature):
                    _buffer = xi
                    xi = xj
                    xj = _buffer
                    yi = yj

            temperature *= ALPHA

            print("Température : ", temperature, ", valeur du critère : ", yi)
            print(xi.afficher())
            _city = xi.coordinate()
            print(_city)
            self.temperature = self.temperature + " " + str(temperature)
            self.criteria_value = self.criteria_value + " " + str(yi)
            self.city = self.city + "@" + str(_city)
            # info = str("Time="+str(current_time)+" Temperature="+str(temperature)+" Value="+str(yi)+" City="+str(_city))
            # IvySendMsg(info)
            # self.send_msg("Time="+str(current_time)+" Temperature="+str(temperature)+" Value="+str(yi)+" City="+str(_city))
            # IvySendMsg("Température="+str(temperature)+" Value="+str(yi)+" City="+str(ville))

    def send_msg(self,msg):
        """
        Send a message through Ivy bus
        :param msg: the message we want to send
        :return: message sent
        """
        IvyInit("Anealing - Metaheuristic",msg)  # Initialize the bus
        IvyStart()


if __name__ == "__main__":

    data = None
    # data = bag.Data(DIMENSION)
    data = tsp.Data(DIMENSION)
    # data = seq.Data(DIMENSION)

    recuit = Recuit(data)

    print("******************Heat-Up******************\n")
    temperature_initiale = recuit.heat_up_loop()

    print("Température initiale = ", temperature_initiale)

    print("******************Cooling*********************\n")
    recuit.cooling_loop(temperature_initiale)

    t = time.localtime()
    recuit.time = time.strftime("%S", t)  # initialize the time

    IvyInit('Anealing - Metaheuristic',"Time=" + recuit.time + " Temperature=" + recuit.temperature + " Value=" + recuit.criteria_value + " City=" + recuit.city + " Radius=" + str(recuit.tsp_radius))
    IvyStart()






