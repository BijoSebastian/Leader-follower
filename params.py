# -*- coding: utf-8 -*-
"""
Created on Sat Feb 10 12:50:29 2018
Parameters for human following robot

@author: Bijo Sebastian 
"""

# RRT prameters
windowSize = 25#100.0 # The operating window for each D.O.F
numNodes = 700 #Maximum number of nodes
stepSize = 1.0 # For the tree growth step
epsilon = 1.0#To pad the obsatcles

# Storage of co-ods of maze segments:
mxVec = []
myVec = []
nxVec = []
nyVec = []

# Storage for tree: 
pxVec = []
pyVec = []
pCost = []  
pparents = []

#For replanning
old_start_id = None
old_goal_id = None
old_start = []
old_goal = []