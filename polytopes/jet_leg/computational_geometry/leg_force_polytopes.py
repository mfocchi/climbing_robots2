"""
Created on Tue Dec 17 10:54:31 2019

@author: Romeo Orsolino
"""
import numpy as np
from jet_leg.computational_geometry.polytopes import Polytope

class LegForcePolytopes:
    def __init__(self, numberOfLegs):
        # self.forcePolytopeLF = Polytope()
        # self.forcePolytopeRF = Polytope()
        # self.forcePolytopeLH = Polytope()
        # self.forcePolytopeRH = Polytope()
    
        self.numberOfLegs = numberOfLegs
        self.forcePolytope = []
        for leg in range(numberOfLegs):
            self.forcePolytope.append(Polytope())

    def getVertices(self):
        # v1 = self.forcePolytope[0].getVertices()
        # v2 = self.forcePolytope[1].getVertices()
        # v3 = self.forcePolytope[2].getVertices()
        # v4 = self.forcePolytope[3].getVertices()
        # return [v1, v2, v3, v4]
        v = []
        for leg in range(self.numberOfLegs):
            v.append(self.forcePolytope[leg].getVertices())
        return v


    def getHalfspaces(self):
        # hs1 = self.forcePolytope[0].getHalfspaces()
        # hs2 = self.forcePolytope[1].getHalfspaces()
        # hs3 = self.forcePolytope[2].getHalfspaces()
        # hs4 = self.forcePolytope[3].getHalfspaces()
        # return [hs1, hs2, hs3, hs4]
        hs = []
        for leg in range(self.numberOfLegs):
            hs.append(self.forcePolytope[leg].getHalfspaces())
        return hs

