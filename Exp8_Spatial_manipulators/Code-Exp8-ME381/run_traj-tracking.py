import numpy as np
from kine import myRobot

# Simulation time
simTime=10

# Path to xml file of PUMA 560 manipulator
#xmlpath="PUMA 560/puma_scene.xml"
xmlpath="Svaya SR-L6/svaya_scene.xml"

my6R=myRobot(xmlpath)

##### Task-4: Trajectory tracking

# Initial Position of End-effector
def qtraj(t):
    return np.zeros(8) # Initial joint angles
o_ee=my6R.fkin(qtraj,0)

# Circular Trajectory tracking for End effector
# Centre of circle or ellipse
x0=0.25; y0=0.25; z0=0.75;
o_c=[x0,y0,z0]

# Radius of circle or ellipse
rx=0.075
ry=1*rx

# Frequency of circular trajectory
cycTime=1
#cfreq=0.5 # Cycle Time = 2 sec

# Function for end effector trjectory
def traj(t):
    if t<cycTime:     # Linear trajectory
        x=o_ee[0]+(o_c[0]+rx - o_ee[0])*t/cycTime
        y=o_ee[1]+(o_c[1] - o_ee[1])*t/cycTime
        z=o_ee[2]+(o_c[2] - o_ee[2])*t/cycTime
    else:     # Circular trajectory
        x = o_c[0] + rx * np.cos(2 * np.pi  * t /cycTime )
        y = o_c[1] + ry * np.sin(2 * np.pi * t /cycTime )
        z= o_c[2] #- 0.01* (t-cycTime)
    return np.array([x, y, z])

# Run inverse kinematics, --- Close the window to stop
th=my6R.ikin(traj,simTime=simTime)
print('Joint Angles from IK (in rad) :', th)
print('Joint Angles from IK (in degree) :', 180 / np.pi * th)
