import numpy as np
from kine import myRobot
import matplotlib.pyplot as plt
# Simulation time
simTime=5

# Path to xml file of PUMA 560 manipulator
#xmlpath="PUMA 560/puma_scene.xml"
xmlpath="Svaya SR-L6/svaya_scene.xml"

my6R=myRobot(xmlpath)
##### Task-2: Forward Kinematics

# Initial joint angles (in rad)
qi=[0,0,0,0,0,0]

# Final joint angles (in rad)
qf=qi

# Move the joints
#qf=[1,0,-1,0,0,0]
cycTime=simTime
#cfreq=0.1 # Frequency of joint traj
# Function for joint trjectory
def qtraj(t):
    th=np.zeros(8)
    a = np.empty([6, 4]) # Coefficients for cubic polynomial trajectory
    for i in np.arange(0,len(qi)):
        th[i]=qi[i]+(qf[i]-qi[i])*( (t % cycTime)/cycTime) #linear trajectory
        # Task - Write Cubic Polynomial trajectory for all joints to move from qi to qf with Zero initial and final velocity
        #a[i,:]=np.linalg.inv(np.array([[1,0,0,0],[1,cycTime,cycTime**2,cycTime**3],[0,1,0,0],[0,1,2*cycTime,3*cycTime**2]])) @ np.array([qi[i],qf[i],0,0])
        #th[i]=a[i,0] + a[i,1]*t + a[i,2] * t**2 + a[i,3] * t**3
    return np.array(th)

# Run forward kinematics -- Close the window to stop
o_ee=my6R.fkin(qtraj,simTime)
# Position of End-effector
print('Position Coord. of End effector (in mm) from FK:', 1000*o_ee)

# Trajectory plots
# Plot the joint angles (on Y-axis) with Time (on X-axis) using function qtraj
ttraj=np.arange(0,simTime,simTime/100)
qvec=[]
plt.figure
for t in ttraj:
    qvec.append(qtraj(t))
qvec=np.array(qvec)
for i in range(6):
    plt.plot(ttraj,qvec[:,i],label=f'q_{i+1}')
plt.xlabel('Time (s)')
plt.ylabel('Angle (rad)')
plt.legend()
plt.grid()
plt.show()