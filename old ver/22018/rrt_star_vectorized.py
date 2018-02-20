# -*- coding: utf-8 -*-
"""
Created on Sat Feb 10 12:50:29 2018
The complete vectorized RRT*code
Takes in start goal and obstacles
Return the path (waypoints to goal) and a flag (showing success or failure)
@author: Bijo Sebastian 
"""

import params
import random
import numpy as np
import matplotlib.pyplot as mp

def plotter(reachedGoal):
    #To do all the plotting
    #Takes none
    #returns next way point to goal
    #Plot all nodes
   
    for ids in range(len(params.pxVec)):
        if params.pparents[ids] != None:            
            temp_ls = params.pparents[ids]        
            mp.plot([params.pxVec[ids], params.pxVec[temp_ls]], [params.pyVec[ids], params.pyVec[temp_ls]], 'b--')
    
    if reachedGoal != 0:
        #Plot sol                
        nn = reachedGoal
        waypoints = []
        waypoints.append(nn) #The full path
        while params.pparents[nn] != None:                        
            mp.plot(params.pxVec[nn], params.pyVec[nn], 'ro')
            mp.plot([params.pxVec[nn], params.pxVec[int(params.pparents[nn])]], [params.pyVec[nn], params.pyVec[int(params.pparents[nn])]], 'r--')
            nn = params.pparents[nn] 
            waypoints.append(nn)
    mp.draw()
    mp.pause(0.1)           
    return(waypoints)
        
def node_append(x, y, parent, cost):
    #To append a node to the tree: 
    #Takes in the x, y, parent and cost
    #Returns none
    
    params.pxVec = np.append(params.pxVec, x)
    params.pyVec = np.append(params.pyVec, y)
    params.pparents = np.append(params.pparents, parent)
    params.pCost = np.append(params.pCost, cost)
     
    #plot
    mp.plot(x, y, 'bo', ms = 0.1)  
        
    return

def obes_append(xs, ys, xe, ye):
    #To append a node to the tree: 
    #Takes in the x, y aparent and cost
    #Returns none
    
    params.mxVec = np.append(params.mxVec, xs)
    params.myVec = np.append(params.myVec, ys)
    params.nxVec = np.append(params.nxVec, xe)
    params.nyVec = np.append(params.nyVec, ye)  
    
    #plot
    mp.plot([xs, xe], [ys, ye], '--y')
    
    return

def  fat_obstacles(obes):
    #To pad the obstcles
    #Takes in the obes(start_x, start_y, end_x, end_y)
    #Returns none
    
    #To restart the array to account for only current obsatcles
    #This will have to be achnged to sture  continous map
    params.mxVec = []
    params.myVec = []
    params.nxVec = []
    params.nyVec = []
    for i in range(len(obes)):
        mp.plot([obes[i,0], obes[i,2]], [obes[i,1], obes[i,3]], '-k')
        orient = -np.arctan2((obes[i,3] - obes[i,1]), (obes[i,2] - obes[i,0]))
        R = np.matrix([[np.cos(orient), -np.sin(orient)], [np.sin(orient), np.cos(orient)]])        
        temp = np.matrix([[obes[i,0], obes[i,2]], [obes[i,1], obes[i,3]]]) - np.matrix([[obes[i,0]], [obes[i,1]]]) 

        temp = R*temp
        up = temp + np.matrix([[-params.epsilon, params.epsilon], [params.epsilon, params.epsilon]])
        down = temp + np.matrix([[-params.epsilon, params.epsilon], [-params.epsilon, -params.epsilon]])   
        up = R.transpose()*up
        down = R.transpose()*down
        up += np.matrix([[obes[i,0]], [obes[i,1]]]) 
        down += np.matrix([[obes[i,0]], [obes[i,1]]])
        s1 = np.matrix([[up[0,0], down[0,0]],[up[1,0], down[1,0]]])
        s2 = np.matrix([[up[0,1], down[0,1]],[up[1,1], down[1,1]]])     
        
        obes_append(up[0,0], up[1,0], up[0,1], up[1,1])
        obes_append(down[0,0], down[1,0], down[0,1], down[1,1])
        obes_append(s1[0,0], s1[1,0], s1[0,1], s1[1,1])
        obes_append(s2[0,0], s2[1,0], s2[0,1], s2[1,1])

    return  

def checkIntersect(idxParent, temp):   
    # To check collisions from parent to temp with all obstacles existing
    # Takes in id of parent and temp node(x,y)  
    # returns True or False
    
    # Convert all the vectors and scalars to matrices of the appropriate shapes by using the boradcasting operation:
    shapingMat = np.zeros(len(params.mxVec))
    
    # For the parent
    px = params.pxVec[idxParent] + shapingMat
    py = params.pyVec[idxParent] + shapingMat
    # For the temp
    qx = temp[0] + shapingMat
    qy = temp[1] + shapingMat
    
     # For the start points of the segments: 
    mx = params.mxVec + shapingMat
    my = params.myVec + shapingMat
    # For the end points of the segments: 
    nx = params.nxVec + shapingMat
    ny = params.nyVec + shapingMat
    
    # Compute the four orientation combinations in  vectorized manner: 
    # Reference formula for orientation of a triplet of points: ( uno, dos, tres) 
    #  det = (dos.x - uno.x)*(tres.y - uno.y) -  (tres.x - uno.x)*(dos.y - uno.y)
    # The 4 combinations are: 
    
    # 3.1: pqm ->  p = uno , q = dos , m = tres : Shape of pqm -(nPts, nSeg)
    pqm = np.sign( ( qx - px)*(my - py) - (mx - px)*(qy - py) )  
    # 3.2: pqn ->  p = uno , q = dos , n = tres : Shape of pqm -(nPts, nSeg)
    pqn = np.sign( ( qx - px)*(ny - py) - (nx - px)*(qy - py) ) 
    # 3.3: mnp -> m = uno , n = dos , p = tres : Shape of mnp - (nPts,nSeg)
    mnp = np.sign( (nx - mx)*(py - my)  - (px - mx)*(ny - my) )
    # 3.4: mnq -> m = uno , n = dos , q = tres : Shape of mnq - (nPts,nSeg)
    mnq = np.sign( (nx - mx)*(qy - my)  - (qx - mx)*(ny - my) )
    
    # 4. Check the two cases of intersection: 
    # 4.1: General case: Case 1: 
    case1 = np.logical_and(pqm != pqn , mnp != mnq)
    # 4.2: Special case: Case 2: This happens if the segments are collinear:
    case2 = np.logical_and( np.logical_and(pqm == 0 , pqn ==0) , np.logical_and( mnp ==0 , mnq ==0) )

    # A collision occurs if either of the two cases are True: 
    # If a collision occurs between a particular point and segment , it will appear as a True element at the [i,j] th 
    # element of collisionMat  - i -> For point on the tree , j -> For segment in the Maze
    collisionvect = np.logical_or(case1,case2)
    
    # convert from collision matrox to vector
    return np.any(collisionvect)
    
def step_from_to(idxParent, rand):
    # To Grow from parent node to random node as far as you can before collision. 
    # Takes in id of parent and random node(x,y)
    # Returns res (x, y)
       
    r1 = params.pxVec[idxParent]
    r2 = params.pyVec[idxParent]
    slope = np.arctan2((rand[1] - r2), (rand[0] - r1)) 
    temp = [r1 + params.stepSize*np.cos(slope), r2 + params.stepSize*np.sin(slope)]
    
    while not checkIntersect(idxParent, temp):       
        r1 =  temp[0]
        r2 =  temp[1]
        temp[0] += params.stepSize*np.cos(slope)
        temp[1] += params.stepSize*np.sin(slope)        
        
    return [r1, r2]    

def checkCollisionVect(rand):
    #To check collisions of all points on tree witha a random point
    #takes in only the random point (x, y coordinates)  
    #returns a vector os the same length as the number of points on the tree      
    
    # Convert all the vectors and scalars to matrices of the appropriate shapes by using the boradcasting operation:
    shapingMat = np.zeros((len(params.pxVec), len(params.mxVec)))

    # For the points already on the tree:
    px = params.pxVec[:,np.newaxis] + shapingMat
    py = params.pyVec[:,np.newaxis] + shapingMat

    # For the randomly sampled point: 
    qx = rand[0] + shapingMat
    qy = rand[1] + shapingMat

    # For the start points of the segments: 
    mx = params.mxVec + shapingMat
    my = params.myVec + shapingMat
    # For the end points of the segments: 
    nx = params.nxVec + shapingMat
    ny = params.nyVec + shapingMat

    # Compute the four orientation combinations in  vectorized manner: 
    # Reference formula for orientation of a triplet of points: ( uno, dos, tres) 
    #  det = (dos.x - uno.x)*(tres.y - uno.y) -  (tres.x - uno.x)*(dos.y - uno.y)
    # The 4 combinations are: 
    
    # 3.1: pqm ->  p = uno , q = dos , m = tres : Shape of pqm -(nPts, nSeg)
    pqm = np.sign( ( qx - px)*(my - py) - (mx - px)*(qy - py) )  
    # 3.2: pqn ->  p = uno , q = dos , n = tres : Shape of pqm -(nPts, nSeg)
    pqn = np.sign( ( qx - px)*(ny - py) - (nx - px)*(qy - py) ) 
    # 3.3: mnp -> m = uno , n = dos , p = tres : Shape of mnp - (nPts,nSeg)
    mnp = np.sign( (nx - mx)*(py - my)  - (px - mx)*(ny - my) )
    # 3.4: mnq -> m = uno , n = dos , q = tres : Shape of mnq - (nPts,nSeg)
    mnq = np.sign( (nx - mx)*(qy - my)  - (qx - mx)*(ny - my) )

    # 4. Check the two cases of intersection: 
    # 4.1: General case: Case 1: 
    case1 = np.logical_and(pqm != pqn , mnp != mnq)
    # 4.2: Special case: Case 2: This happens if the segments are collinear:
    case2 = np.logical_and( np.logical_and(pqm == 0 , pqn ==0) , np.logical_and( mnp ==0 , mnq ==0) )

    # A collision occurs if either of the two cases are True: 
    # If a collision occurs between a particular point and segment , it will appear as a True element at the [i,j] th 
    # element of collisionMat  - i -> For point on the tree , j -> For segment in the Maze
    collisionMat = np.logical_or(case1,case2)

    # convert from collision matrox to vector
    collisionvect = np.any(collisionMat , axis =1)
    
    return collisionvect

def RRT_star(start, goal, obstacles): 
    #RRT* Main routine: 
    #Takes start (x,y) and goal (x,y), obstacles(start_x, start_y, end_x, end_y)
    #Will return path if found or return 0 for no path           
    
    #For the first time the tree is intialised
    if len(params.pxVec) == 0:
       
        #Setup the plot  
        mp.close('all')
        mp.axis([0.0, params.windowSize, 0.0, params.windowSize])
        mp.ion()  
        mp.show()  
        print('start: ', start)
        print('goal: ', goal)
    
        #Append the start
        node_append(start[0], start[1], None, 0.0) 
        params.old_start = start
        params.old_goal = goal
        params.old_start_id = 0
        
        #plan
        reachedGoal = 0
        
        
    else:  
       #For all other times
       
       #Setup the plot
       mp.clf()
       mp.axis([0.0, params.windowSize, 0.0, params.windowSize])
       print('start: ', start)
       print('goal: ', goal)
       mp.plot(start[0], start[1], 'go', ms = 10.0)
       mp.plot(goal[0], goal[1], 'go', ms = 10.0)   
       
       if params.old_start != start:
           #New start
           
           #Distance form new start to old start
           dist = np.sqrt((params.pyVec[params.old_start_id] - start[1])**2 + (params.pxVec[params.old_start_id] - start[0])**2) 
           #Update all costs with that value
           params.pCost += dist
           #Add new start
           node_append(start[0], start[1], None, 0.0)
           #upend old starts parent
           params.pparents[params.old_start_id] = len(params.pxVec) - 1
           params.old_start_id = len(params.pxVec) - 1
           #Yes we need a -1, because say the length of an array is 3 and the index number of last element is only 2 
           #Rewire
           #compute new collision vector based on new rand
           collisionvect = checkCollisionVect(start)
           #compute the distance of all nodes in the Tree to new rand
           distArr = np.sqrt((params.pyVec - start[1])**2 + (params.pxVec - start[0])**2)                
           # Choose the right parent
           # Find the indices of the nodes on the tree which don't have collisions:
           idxNoCollision = np.where(~collisionvect)[0]              
           #Compute new cost for all that do not collide
           newCost = np.full((len(params.pxVec)), np.inf)
           newCost[idxNoCollision] = distArr[idxNoCollision]               
           #Change parent for all that is closer
           params.pparents[newCost < params.pCost] = len(params.pxVec) - 1
           #Update old
           params.old_start = start
           

       if params.old_goal != goal:
           #Replan
           reachedGoal = 0
           params.old_goal = goal
       else:
           #No Replan
           reachedGoal = params.old_goal_id
            
    #Fatten the obstacles, asuming the obstcles do change with time
    fat_obstacles(obstacles)
    numiter = 0 
    
    # Loop until we reach the goal or exceed the specifiecd number of nodes: 
    while ( numiter < 100 ) and ( reachedGoal == 0): 
        
        numiter += 1
        if reachedGoal == 0:
            print('Sampling, num nodes: ', len(params.pxVec), 'iterno: ', numiter)            
        else:
            print('Optimising, num nodes: ', len(params.pxVec), 'iterno: ', numiter)              
            #This gets activated if you allow it to run beyond reaching the goal
            
        #Every 10 new node check if goal can be reached 
        #(This is needed so that it randomly smaples the space initially)       
        if (numiter % 50 == 0 ) or (len(params.pxVec) > params.numNodes) :             
                rand = [goal[0], goal[1]]            
        else:
            # Randomly sample a node within the specified window: 
            rand = [random.random()*params.windowSize, random.random()*params.windowSize]
        
        #print('random', rand)        
        # Compute the distance of all nodes in the Tree to the random node: 
        distArr = np.sqrt((params.pyVec - rand[1])**2 + (params.pxVec - rand[0])**2) 
        
        # Get collision info for all nodes in the Tree to the random node:
        collisionvect = checkCollisionVect(rand)
        
        # Address the special case: If all the nodes cause collision
        if np.all(collisionvect):
            #print('all collision')
            # Find the nearest neighbor of the random node in the tree:
            idxParent = np.argmin(distArr)
            #print('closest', params.pxVec[idxParent], params.pyVec[idxParent])
            
            # Step from the nearest neighbor to the random node: a.k.a Growing the tree: 
            rand = step_from_to(idxParent, rand)
            
            #Check if any progress
            if rand[0] == params.pxVec[idxParent] and rand[1] == params.pyVec[idxParent]:
                #print('no motion')
                continue
            else:
                
                #print('new random', rand) 
                #compute new collision vector based on new rand
                collisionvect = checkCollisionVect(rand)
                
                # compute the distance of all nodes in the Tree to new rand
                distArr = np.sqrt((params.pyVec - rand[1])**2 + (params.pxVec - rand[0])**2) 
                
        # Choose the right parent
        # Find the indices of the nodes on the tree which don't have collisions:
        idxNoCollision = np.where(~collisionvect)[0]
        # Compute cost for all non colliding nodes to new node          
        temp_cost = params.pCost[idxNoCollision] +  distArr[idxNoCollision]
   
        #Find the parent id  and cost based on the minimum cost
        parent_id = idxNoCollision[np.argmin(temp_cost)]
        cost = temp_cost[np.argmin(temp_cost)]
    
        #Now append the node
        #print('newnode', rand[0], rand[1], parent_id, cost )
        #print('parent',  parent_id )
        node_append(rand[0], rand[1], parent_id, cost)  
        
        #Rewire
        #Putting in a note to rewire the full tree here
        #Compute new cost for all that do not collide
        newCost = np.full((len(params.pxVec)), np.inf)
        #Obviously the parent should not be used
        newCost[idxNoCollision] = cost + distArr[idxNoCollision]
        newCost[parent_id] = np.inf
        
        #Change parent for all that is closer
        params.pparents[newCost < params.pCost] = len(params.pxVec) - 1
        
        if (rand[0] == goal[0] and rand[1] == goal[1]):
            reachedGoal = len(params.pxVec) - 1
            
    if(reachedGoal == 0):
        print('failed') 
        return([])
    else:
        print("success")
        waypoints = plotter(reachedGoal)
        params.old_goal_id = reachedGoal
        the_right_path = []
        for counter, waypoint in enumerate(waypoints):
            #print('point ', counter,' : ', params.pxVec[waypoint], params.pyVec[waypoint])
            the_right_path.append([params.pxVec[waypoint], params.pyVec[waypoint]])       
        return(list(reversed(the_right_path)))
    
##Uncomment for testing purposes
## Finally we should run the test
#if __name__ == '__main__':
#    #Trial obstacles
#    obes = np.array([[20,0,20,60],
#                     [60,60,80,20],
#                     [20,0,80,30],
#                     [60,40,60,85]])    
#    start = [0.0, 0.0] # Start
#    goal = [50.0, 5.0] # Goal
#
#    waypoint = RRT_star(start, goal, obes)
#    print('way point: ', waypoint)
#    start = [10.0, 25.0] # Start
#    goal = [45.0, 30.0] # Goal
#    waypoint = RRT_star(start, goal, obes)  
#    print('way point: ', waypoint)  
#    start = [10.0, 25.0] # Start
#    goal = [55.0, 80.0] # Goal
#    waypoint = RRT_star(start, goal, obes)  
#    print('way point: ', waypoint) 
#    start = [10.0, 55.0] # Start
#    goal = [55.0, 80.0] # Goal
#    waypoint = RRT_star(start, goal, obes)  
#    print('way point: ', waypoint)  
        
            