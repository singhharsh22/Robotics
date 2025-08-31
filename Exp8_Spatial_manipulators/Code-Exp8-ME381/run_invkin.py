import numpy as np
from kine import myRobot

# Simulation time
simTime=5

# Path to xml file of PUMA 560 manipulator
#xmlpath="PUMA 560/puma_scene.xml"
xmlpath="Svaya SR-L6/svaya_scene.xml"

my6R=myRobot(xmlpath)

##### Task-3: Inverse Kinematics

# Position of End-effector (in m)
ee_fin=np.array([0.2,0.2,1])

# Task - Change the values in ee_fin and check the inverse kinematics solution

# Initial Position of End-effector (in m)
def qtraj(t):
    return np.zeros(8) # Initial joint angles
ee_init=my6R.fkin(qtraj,0)

# Frequency of  trajectory
cycTime=1
#cfreq=0.5 # Cycle Time = 2 sec

# Function for end effector trjectory
def traj(t):
    if t<cycTime:     # Linear trajectory
        x=ee_init[0]+(ee_fin[0] -ee_init[0])*t/cycTime
        y=ee_init[1]+(ee_fin[1] - ee_init[1])*t/cycTime
        z=ee_init[2]+(ee_fin[2] - ee_init[2])*t/cycTime
    else:     # Circular trajectory
        x = ee_fin[0]
        y = ee_fin[1]
        z= ee_fin[2]
    return np.array([x, y, z])

# Run inverse kinematics, --- Close the window to stop
th=my6R.ikin(traj,simTime=simTime)
print('Joint Angles from IK (in rad) :', th)
print('Joint Angles from IK (in degree) :', 180 / np.pi * th)
