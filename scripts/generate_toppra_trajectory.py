#!/usr/bin/env python

# Toppra imports
import toppra as ta
import toppra.constraint as constraint
import toppra.algorithm as algo
import numpy as np
import matplotlib.pyplot as plt
import time

# Ros imports
import rospy
from topp_ros.srv import GenerateTrajectory, GenerateTrajectoryResponse
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

MIN_GRID_POINTS = 200

class ToppraTrajectory():

    def __init__(self):
        # Basically we have just one service waiting for request and outputting
        # trajectory
        self.generate_toppra_trajectory_service = rospy.Service(
            'generate_toppra_trajectory', GenerateTrajectory, 
            self.generateToppraTrajectoryCallback)

        self.raw_trajectory_pub = rospy.Publisher('toppra_raw_trajectory', 
            JointTrajectory, queue_size=1)

        self.raw_waypoints_pub = rospy.Publisher('toppra_raw_waypoints', 
            JointTrajectory, queue_size=1)

    def generateToppraTrajectoryCallback(self, req):
        print " "
        print "Generating TOPP-RA trajectory."
        tstart = time.time()
        res = GenerateTrajectoryResponse()
        dof = len(req.waypoints.points[0].positions)
        n = len(req.waypoints.points)

        # If there is not enough waypoints to generate a trajectory return false
        if (n <= 1 or dof == 0):
            print "You must provide at least 2 points to generate a valid trajectory."
            res.success = False
            return res

        valid = False
        p0 = req.waypoints.points[0].positions
        for position in req.waypoints.points[1:]:
            p1 = position.positions 
            diff = np.array(p1) - np.array(p0)
            if (diff > 0.0001).any():
                valid = True 
                break
        if not valid:
            print "All positions are identical. Cannot generate toppra trajectory"
            res.success = False
            return res

        try:
            # Generate trajectory.
            # First set up waypoints. We know hom many will be from n and dof.
            way_pts = np.zeros([n, dof])
            # Next fill the waypoints with data from request.
            for i in range(0, n):
                for j in range(0, dof):
                    way_pts[i][j] = req.waypoints.points[i].positions[j]

            # Part of TOPP-RA is to generate path(s \in [0,1]) from n waypoints.
            # The algorithm then parametrizes the initial path.
            print("Calculating spline interpolation")
            path = ta.SplineInterpolator(np.linspace(0, 1, n), way_pts)

            # Create velocity and acceleration bounds. Supposing symmetrical bounds around zero.
            vlim_ = np.zeros([dof])
            alim_ = np.zeros([dof])
            for i in range(0, dof):
                vlim_[i] = req.waypoints.points[0].velocities[i]
                alim_[i] = req.waypoints.points[0].accelerations[i]
            vlim = np.vstack((-vlim_, vlim_)).T
            alim = np.vstack((-alim_, alim_)).T
            pc_vel = constraint.JointVelocityConstraint(vlim)
            pc_acc = constraint.JointAccelerationConstraint(
                alim, discretization_scheme=constraint.DiscretizationType.Interpolation)
            
            # Setup a parametrization instance
            num_grid_points = np.max([MIN_GRID_POINTS, n*2])
            gridpoints = np.linspace(0, path.duration, num_grid_points)
            print("Calling TOPPRA")
            instance = algo.TOPPRA([pc_vel, pc_acc], path)

            # Retime the trajectory, only this step is necessary.
            print("Computing trajectory from instance")
            t0 = time.time()
            jnt_traj = instance.compute_trajectory()
            print("Computed traj from instance")
            # jnt_traj, aux_traj = instance.compute_trajectory(0, 0)
            #print("Parameterization time: {:} secs".format(time.time() - t0))

            # Check if trajectory generation was successful
            if jnt_traj is None:
                print("TOPPRA trajectory generation failed")
                res.success = False
                return res

            # Plot for debugging
            if req.plot:
                ts_sample = np.linspace(0, jnt_traj.duration, 100)
                qs_sample = jnt_traj(ts_sample)
                qds_sample = jnt_traj(ts_sample, 1)
                qdds_sample = jnt_traj(ts_sample, 2)
                fig, axs = plt.subplots(3, 1, sharex=True)
                for i in range(path.dof):
                    # plot the i-th joint trajectory
                    axs[0].plot(ts_sample, qs_sample[:, i], c="C{:d}".format(i))
                    axs[1].plot(ts_sample, qds_sample[:, i], c="C{:d}".format(i))
                    axs[2].plot(ts_sample, qdds_sample[:, i], c="C{:d}".format(i))
                axs[2].set_xlabel("Time (s)")
                axs[0].set_ylabel("Position (rad)")
                axs[1].set_ylabel("Velocity (rad/s)")
                axs[2].set_ylabel("Acceleration (rad/s2)")
                plt.show()

            # Convert to JointTrajectory message
            print("Converting to joint traj")
            res.trajectory = self.TOPPRA2JointTrajectory(jnt_traj, req.sampling_frequency)
            print("Converted to joint traj")
            res.success = True
            self.raw_trajectory_pub.publish(res.trajectory)
            self.raw_waypoints_pub.publish(req.waypoints)
            print "Time elapsed: ", time.time()-tstart
            return res
            
        except Exception as e:
            print("Failed to generate TOPPRA traj. Error {}".format(e))
            res.success = False
            return res 

    def TOPPRA2JointTrajectory(self, jnt_traj, f):
        # Sampling frequency is required to get the time samples correctly.
        # The number of points in ts_sample is duration*frequency.
        ts_sample = np.linspace(0, jnt_traj.get_duration(), 
            int(jnt_traj.get_duration()*f))
        # Sampling. This returns a matrix for all DOFs. Accessing specific one is 
        # simple: qs_sample[:, 0]
        qs_sample = jnt_traj.eval(ts_sample)
        qds_sample = jnt_traj.evald(ts_sample)
        qdds_sample = jnt_traj.evaldd(ts_sample)

        n = qs_sample.shape[0]
        dof = qs_sample.shape[1]

        # Transform into JointTrajectory
        joint_trajectory = JointTrajectory()
        for i in range(0, n):
            temp_point = JointTrajectoryPoint()

            for j in range(0, dof):
                temp_point.positions.append(qs_sample[i,j])
                temp_point.velocities.append(qds_sample[i,j])
                temp_point.accelerations.append(qdds_sample[i,j])

            temp_point.time_from_start = rospy.Duration.from_sec(i/f)
            joint_trajectory.points.append(temp_point)

        # Add last point with zero velocity and acceleration
        last_point = JointTrajectoryPoint()
        for i in range(0, dof):
            last_point.positions.append(qs_sample[n-1,i])
            last_point.velocities.append(0.0)
            last_point.accelerations.append(0.0)
        last_point.time_from_start = rospy.Duration.from_sec((n)/f)
        joint_trajectory.points.append(last_point)

        return joint_trajectory

if __name__ == "__main__":
    rospy.init_node("generate_toppra_trajectory")
    generator = ToppraTrajectory()
    rospy.spin()