# -*- coding: utf-8 -*-
"""
Created on Sat Feb 10 12:50:29 2018
The main code that communicates with vrep
Takes nothing 
Runs continosuly as long as the robot needs to be operational
Gets in the current location of the goal and that of the robot 
Sends the velocity commands to the robot

@author: Bijo Sebastian 
"""
#Import libraries
import time
import numpy as np


#Import files
import rrt_star_vectorized
import params

#VRep
global vrep
global clientID
#Handles
global pioneerLeftMotorHandle
global pioneerRightMotorHandle
global pioneerHandle
#state
global x
global y
global theta
#Local goal
global lgx
global lgy
#Controller
global prev_heading_error
global total_heading_error   


  
def localise():
    #Function that will return the current location of the robot
    #PS. THE ORIENTATION MUST BE RETURNED IN RADIANS        

    global x
    global y
    global theta
    global vrep
    global clientID
    global pioneerHandle
    
    res , pioneerPosition = vrep.simxGetObjectPosition(clientID, pioneerHandle, -1 , vrep.simx_opmode_buffer)
    res , pioneerOrientation = vrep.simxGetObjectOrientation(clientID, pioneerHandle, -1 , vrep.simx_opmode_buffer)
    
    x = pioneerPosition[0]
    y = pioneerPosition[1]
    theta  =pioneerOrientation[2]
    
    return     
    
def robot_setvel(V,W):
    #Function to set the linear and rotational velocity of robot           
    
    global vrep
    global clientID
    global pioneerLeftMotorHandle
    global pioneerRightMotorHandle

    R = 0.0975 #in m 
    L = 0.4  #in m 

    # 1. Limit v,w from controller to +/- of their max
    w = max(min(W, 0.3), -0.3)
    v = max(min(V, 0.7), -0.7)
            
    # 2. Compute desired vel_r, vel_l needed to ensure w
    Vr = ((2.0*v) + (w*L))/(2*R)
    Vl = ((2.0*v) - (w*L))/(2*R)
                        
#    # 3. Find the max and min vel_r/vel_l
#    vel_rl_max = max(Vr, Vl)
#    vel_rl_min = min(Vr, Vl)
#            
#    # 4. Shift vel_r and vel_l if they exceed max/min vel
#    if (vel_rl_max > 0.2):
#        vel_r = Vr - (vel_rl_max - 1.0)
#        vel_l = Vl - (vel_rl_max - 1.0)
#    elif (vel_rl_min < -0.2):
#        vel_r = Vr - (vel_rl_min + 1.0)
#        vel_l = Vl - (vel_rl_min + 1.0)
#    else:
    vel_r = Vr
    vel_l = Vl            
    
     # 5. Set velocity
    vrep.simxSetJointTargetVelocity(clientID, pioneerLeftMotorHandle, vel_l, vrep.simx_opmode_oneshot_wait)
    vrep.simxSetJointTargetVelocity(clientID, pioneerRightMotorHandle, vel_r, vrep.simx_opmode_oneshot_wait)
    
    return    

def at_goal():
    #Event which checks if we have reached the goal(within threshold)     
    
    global x
    global y
    global lgx
    global lgy
    
    #get the current robot location
    localise()
       
    #check if we have reached goal point
    d = np.sqrt(((lgx - x)**2) + ((lgy - y)**2))
    
    if d <= 0.3:
        print("Reached goal")
        return True
    else:
        return False
                                                                       
###################CONTROLLER###################################################
def gtg():
    #The Go to goal controller
    
    global x
    global y
    global theta
    global lgx
    global lgy
    global prev_heading_error
    global total_heading_error   
    
    #Controller parameters
    Kp = 0.00656
    Kd = 0.0002
    Ki = 0.0
        
    #get the current robot location
    localise()
    
    #determine how far to rotate to face the goal point
    #PS. ALL ANGLES ARE IN RADIANS
    dt = (np.arctan2((lgy - y), (lgx - x))) - theta
    #restrict angle to (-pi,pi)
    dt = ((dt + np.pi)%(2.0*np.pi)) - np.pi
    dt = ((dt*180.0)/np.pi)
        
    #control input for angular velocity
    W = (Kp*dt) + (Ki*total_heading_error) + (Kd*(dt - prev_heading_error))
    total_heading_error = total_heading_error + dt
    prev_heading_error = dt
  
    #find distance to goal
    d = np.sqrt(((lgx - x)**2) + ((lgy - y)**2))
    
    #velocity parameters
    #velMult = 0.1#mm/s
    distThresh = 0.1#mm
    
    #control input for linear velocity
    #V = ((np.arctan((d - distThresh))) - (np.arctan(dt)))*velMult
    V = (0.12/1.5)*(np.arctan(d - distThresh))
    #request robot to execute velocity
    robot_setvel(V,W)
                                       
    return                     
                 

############################WALL Getter#######################################
def get_wall_seg(centerPos, length, zAngle):
    #Function to get the end points of a single wall segment
    
    cosA = np.cos(zAngle)
    sinA = np.sin(zAngle)

    # Start point:
    xs = centerPos[0]  - sinA*(length/2)
    ys = centerPos[1]  + cosA*(length/2)

    # End point:
    xe = centerPos[0]  + sinA*(length/2)
    ye = centerPos[1]  - cosA*(length/2)

    return [xs, ys, xe, ye]

def get_maze_segments():
    # Function to get the maze sgements from the wall handles list:
    # Format: Returns a list called mazeSegments:
    # mazeSegments is a list of segments -> Each segment is a list of two tuples corresponding to start and end points -> Each tuple contains (x,y) co-ods  of the points
    
    global vrep
    global clientID
    
    ## Get handles to:
    # 1. The Maze collection:
    res , mazeHandle = vrep.simxGetCollectionHandle(clientID, "Maze", vrep.simx_opmode_blocking)

    # Get the handles associated with each wall and the absolute position of the center of each wall:
    res, wallHandles , intData, absPositions, stringData = vrep.simxGetObjectGroupData(clientID, mazeHandle, 3,vrep.simx_opmode_blocking)

    mazeSegments = []

    if res == vrep.simx_return_ok:

        count = 1
        for wall in wallHandles:

            res, wallCenterAbsPos = vrep.simxGetObjectPosition(clientID, wall, -1, vrep.simx_opmode_oneshot_wait)

            res, wallMinY = vrep.simxGetObjectFloatParameter(clientID,wall, vrep.sim_objfloatparam_objbbox_min_y , vrep.simx_opmode_oneshot_wait)

            res, wallMaxY = vrep.simxGetObjectFloatParameter(clientID,wall, vrep.sim_objfloatparam_objbbox_max_y , vrep.simx_opmode_oneshot_wait)

            wallLength = abs(wallMaxY - wallMinY)

            # Get the orientation of the wall: Third euler angle is the angle around z-axis:
            res , wallOrient = vrep.simxGetObjectOrientation(clientID, wall, -1 , vrep.simx_opmode_oneshot_wait)

            # Get the end points of the maze wall: A list containing two tuples: [ (xs,ys) , (xe,ye)]
            wallSeg = get_wall_seg(wallCenterAbsPos, wallLength, wallOrient[2])    # Assuming all walls are on the ground and nearly flat:

            print ("Wall #" , count, " -> " , wallSeg)

            mazeSegments.append(wallSeg)

            count+=1

    else:

        print (" Failed to get individual wall handles!")


    return mazeSegments     
           
##################################MAIN######################################
def main():
    #The main function
    
    global vrep
    global clientID
    global pioneerLeftMotorHandle
    global pioneerRightMotorHandle
    global pioneerHandle
    global x
    global y
    global theta
    global lgx
    global lgy
    global prev_heading_error
    global total_heading_error   
    
    try:
        import vrep
    except:
        print ('--------------------------------------------------------------')
        print ('"vrep.py" could not be imported. This means very probably that')
        print ('either "vrep.py" or the remoteApi library could not be found.')
        print ('Make sure both are in the same folder as this file,')
        print ('or appropriately adjust the file "vrep.py"')
        print ('--------------------------------------------------------------')
        print ('')
    
    #Setup the simulation
    vrep.simxFinish(-1) # just in case, close all opened connections
    clientID = vrep.simxStart('127.0.0.1',19997,True,True,5000,5) # Connect to V-REP
    
    if clientID!=-1:

        print ('Connected to remote API server')

        ###Extract the maze segments from the VREP simulation:
        obstacles = np.array(get_maze_segments())

        ###Get a handles to Pioneer and Goal:
        # Handle to the Pioneer:
        res , pioneerHandle = vrep.simxGetObjectHandle(clientID, "Pioneer", vrep.simx_opmode_blocking)
        res,  pioneerLeftMotorHandle = vrep.simxGetObjectHandle(clientID, "left", vrep.simx_opmode_blocking)
        res,  pioneerRightMotorHandle = vrep.simxGetObjectHandle(clientID, "right", vrep.simx_opmode_blocking)

        # Get the position of the Pioneer for the first time in streaming mode
        res , pioneerPosition = vrep.simxGetObjectPosition(clientID, pioneerHandle, -1 , vrep.simx_opmode_streaming)
        res , pioneerOrientation = vrep.simxGetObjectOrientation(clientID, pioneerHandle, -1 , vrep.simx_opmode_streaming)

        # Deactivate joint actuations:Make sure Pioneer is stationary:
        res = vrep.simxSetJointTargetVelocity(clientID, pioneerLeftMotorHandle, 0, vrep.simx_opmode_streaming)
        res = vrep.simxSetJointTargetVelocity(clientID, pioneerRightMotorHandle, 0, vrep.simx_opmode_streaming)
        
        # Handle to the goal:
        res , goalHandle = vrep.simxGetObjectHandle(clientID, "Mesh17#0", vrep.simx_opmode_blocking)

        # Get the position of the Pioneer for the first time in streaming mode
        res , goalPosition = vrep.simxGetObjectPosition(clientID, goalHandle, -1 , vrep.simx_opmode_streaming)
        
        ###Start the Simulation: Keep printing out status messages!!!
        res = vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot_wait)

        if res == vrep.simx_return_ok:
                
            print ("---!!! Started Simulation !!! ---")

            for i in range(300): #Overall
                print('iter',i)
                
                robot_setvel(0.0, 0.0)
                #Estimation
                #Obtain robots position
                localise()
                
                #Obtain goal position
                res , goalPosition = vrep.simxGetObjectPosition(clientID, goalHandle, -1 , vrep.simx_opmode_buffer)
                #Planning
                if i == 0:
                    #Plan path
                    path = rrt_star_vectorized.RRT_star([x,y], goalPosition[0:2],  obstacles) 
                    
                else:                    
                    s_dist = np.sqrt((params.old_start[1] - y)**2 + (params.old_start[0] - x)**2)
                    g_dist = np.sqrt((params.old_goal[1] - goalPosition[1])**2 + (params.old_goal[0] - goalPosition[0])**2)
                    
                    if max(s_dist, g_dist) > params.stepSize:
                        
                        #Plan path
                        path = rrt_star_vectorized.RRT_star([x,y], goalPosition[0:2],  obstacles)  
               
                if len(path) == 0:
                    time.sleep(0.5)  
                    continue
                #Control
                print('path', path)                
                wpt_no = 1
                #Initialise
                prev_heading_error = 0.0
                total_heading_error = 0.0
                j = 0
                while j < 5 and wpt_no < len(path):  #Local controller              
                    lgx = path[wpt_no][0]
                    lgy = path[wpt_no][1]                                 
                                    
                    if not at_goal():
                        gtg()
                    else:
                        robot_setvel(0.0, 0.0)
                        #Initialise
                        prev_heading_error = 0.0
                        total_heading_error = 0.0
                        wpt_no +=1    
                    
                    time.sleep(0.5)  
                    j+=1
                    
    robot_setvel(0.0, 0.0)                    
    
#run
if __name__ == '__main__':
    main()                    
 