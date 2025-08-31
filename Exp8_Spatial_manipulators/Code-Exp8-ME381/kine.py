import mujoco
import mujoco.viewer
import numpy as np
import time

### Code of mjctrl from Kevin Zakka's github library

class myRobot():
    def __init__(self, xmlpath):
        # Load the model and data.
        self.model = mujoco.MjModel.from_xml_path(xmlpath)
        #self.data = mujoco.MjData(self.model)
        # Simulation timestep in seconds.
        self.dt: float = 0.002
        # End-effector site we wish to control, in this case a site attached to the last
        # link (wrist_3_link) of the robot.
        self.site_id = self.model.site("attachment_site").id

        # Get the dof and actuator ids for the joints we wish to control.
        joint_names = [
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "joint5",
            "joint6",
        ]
        self.dof_ids = np.array([self.model.joint(name).id for name in joint_names])
        # Note that actuator names are the same as joint names in this case.
        self.actuator_ids = np.array([self.model.actuator(name).id for name in joint_names])

        # Mocap body we will control with our mouse.
        self.mocap_id = self.model.body("target").mocapid[0]

    # Forward kinematics from given joint trajectory
    def fkin(self,qtraj, simTime=60): #def main() -> None:

        # Damping term for the pseudoinverse. This is used to prevent joint velocities from
        # becoming too large when the Jacobian is close to singular.
        #damping: float = 1e-4

        #xmlpath = "PUMA 560/puma_scene.xml"

        #assert mujoco.__version__ >= "3.1.0", "Please upgrade to mujoco 3.1.0 or later."

        # Load the model and data.
        model = self.model #mujoco.MjModel.from_xml_path(xmlpath)
        data = mujoco.MjData(model)

        # Override the simulation timestep.
        dt=self.dt
        model.opt.timestep = dt

        with mujoco.viewer.launch_passive(
            model=model, data=data, show_left_ui=False, show_right_ui=False
        ) as viewer:

            # Initialize the camera view to that of the free camera.
            mujoco.mjv_defaultFreeCamera(model, viewer.cam)

            # Toggle site frame visualization.
            viewer.opt.frame = mujoco.mjtFrame.mjFRAME_SITE
            #viewer.opt.frame = mujoco.mjtFrame.mjFRAME_WORLD

            while viewer.is_running() and data.time<=simTime:
                step_start = time.time()

                # Set the target position of the end-effector site.
                data.mocap_pos[self.mocap_id, 0:3] = np.array([0,0,0]) #circle(data.time, r, x, y, cfreq)

                # Set the target position of the joint angles.
                q = qtraj(data.time)

                # Step the simulation.
                #mujoco.mj_step(model, data)
                data.qpos[0:6]=q[:6].copy()
                mujoco.mj_fwdPosition(model, data) # Update position
                data.time=data.time+dt

                viewer.sync()
                time_until_next_step = dt - (time.time() - step_start)
                if time_until_next_step > 0:
                    time.sleep(time_until_next_step)
        print('Joint Angles for FK (in rad) :',data.qpos[0:6])
        print('Joint Angles for FK (in degree) :',180/np.pi*data.qpos[0:6])

        return data.site(self.site_id).xpos


    def ikin(self,traj,simTime=60): #def main() -> None:

        # Damping term for the pseudoinverse. This is used to prevent joint velocities from
        # becoming too large when the Jacobian is close to singular.
        damping: float = 1e-2

        #xmlpath = "PUMA 560/puma_scene.xml"

        #assert mujoco.__version__ >= "3.1.0", "Please upgrade to mujoco 3.1.0 or later."

        # Load the model and data.
        model = self.model #mujoco.MjModel.from_xml_path(xmlpath)
        data = mujoco.MjData(model)

        # Override the simulation timestep.
        dt=self.dt
        model.opt.timestep = dt

        # Mocap id
        mocap_id=self.mocap_id
        site_id=self.site_id

        # Pre-allocate numpy arrays.
        jac = np.zeros((6, model.nv))
        diag = damping * np.eye(6)
        error = np.zeros(6)
        error_pos = error[:3]
        error_ori = error[3:]
        site_quat = np.zeros(4)
        site_quat_conj = np.zeros(4)
        error_quat = np.zeros(4)

        #data.qpos = np.ones(len(data.qpos))

        with mujoco.viewer.launch_passive(
            model=model, data=data, show_left_ui=True, show_right_ui=False
        ) as viewer:

            # Initialize the camera view to that of the free camera.
            mujoco.mjv_defaultFreeCamera(model, viewer.cam)

            # Toggle site frame visualization.
            viewer.opt.frame = mujoco.mjtFrame.mjFRAME_SITE
            # iterator for decorative geometry objects
            idx_geom = 0

            while viewer.is_running() and data.time<=simTime:
                step_start = time.time()

                # Set the target position of the end-effector site.
                data.mocap_pos[mocap_id, 0:3] = traj(data.time) #circle(data.time, r, x, y, cfreq)
                #data.mocap_pos[mocap_id, 2] = z

                # Position error.
                error_pos[:] = data.mocap_pos[mocap_id] - data.site(site_id).xpos

                # Orientation error.
                mujoco.mju_mat2Quat(site_quat, data.site(site_id).xmat)
                mujoco.mju_negQuat(site_quat_conj, site_quat)
                mujoco.mju_mulQuat(error_quat, data.mocap_quat[mocap_id], site_quat_conj)
                mujoco.mju_quat2Vel(error_ori, error_quat, 1.0)

                # Get the Jacobian with respect to the end-effector site.
                mujoco.mj_jacSite(model, data, jac[:3], jac[3:], site_id)

                # Solve system of equations: J @ dq = error.
                dq = jac.T @ np.linalg.solve(jac @ jac.T + diag, error)

                # Integrate joint velocities to obtain joint positions.
                q = data.qpos.copy()
                mujoco.mj_integratePos(model, q, dq, 1)

                # Set the control signal.
                np.clip(q, *model.jnt_range.T, out=q)
                for i in self.dof_ids:
                    #data.ctrl[actuator_ids] = q[dof_ids]
                    data.ctrl[i]=0#1*(q[i]-data.qpos[i])+1*(0-data.qvel[i])

                # Step the simulation.
                #mujoco.mj_step(model, data)
                data.qpos=q.copy()
                mujoco.mj_fwdPosition(model, data) # Update position
                data.time=data.time+dt

                # mj Geometry from vyankatesh's code
                mujoco.mjv_initGeom(viewer.user_scn.geoms[idx_geom],
                type=mujoco.mjtGeom.mjGEOM_SPHERE,
                size=[0.003,0,0],
                pos = np.array([data.site_xpos[0][0],data.site_xpos[0][1],data.site_xpos[0][2]]),
                mat=np.eye(3).flatten(),
                rgba=np.array([1,0,0,0.5]))
                idx_geom += 1
                viewer.user_scn.ngeom = idx_geom
                # Reset if the number of geometries hit the limit
                if idx_geom > (viewer.user_scn.maxgeom - 50):
                    # Reset
                    idx_geom = 1

                viewer.sync()
                time_until_next_step = dt - (time.time() - step_start)
                if time_until_next_step > 0:
                    time.sleep(time_until_next_step)
        print('Position Coord. of End effector (in mm) for IK:', 1000*data.site(site_id).xpos)

        return data.qpos[0:6]



    """
    if __name__ == "__main__":
        main()
    """
