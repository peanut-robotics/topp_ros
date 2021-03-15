#! /usr/bin/env python

import typing 
import numpy as np
import matplotlib.pyplot as plt

class PlotHelper(object):
    def __init__(self):
        pass 

    @staticmethod
    def compare_spline(spline, way_pts, spline_smoothing_factor):
        n = way_pts.shape[0]
        dof = way_pts.shape[1]

        x = np.linspace(0, 1, n)
        fig, axs = plt.subplots(dof, sharex=True)
        for i in range(dof):
            axs[i].scatter(x, way_pts[:, i], s=0.7, marker='.', color='firebrick', label='Desired points')
            axs[i].plot(x, spline[:,i], markerfacecolor='b', lw=2.5, alpha=0.5, label='Spline')
            axs[i].legend(loc="upper right")
        fig.suptitle("Spline with smoothing factor {:.5f}".format(spline_smoothing_factor))
        plt.show()
        
    @staticmethod
    def plot_trajectory(jnt_traj, dof):
        ts_sample = np.linspace(0, jnt_traj.duration, 100)
        qs_sample = jnt_traj(ts_sample)
        qds_sample = jnt_traj(ts_sample, 1)
        qdds_sample = jnt_traj(ts_sample, 2)
        fig, axs = plt.subplots(3, 1, sharex=True)
        for i in range(dof):
            # plot the i-th joint trajectory
            axs[0].plot(ts_sample, qs_sample[:, i], c="C{:d}".format(i))
            axs[1].plot(ts_sample, qds_sample[:, i], c="C{:d}".format(i))
            axs[2].plot(ts_sample, qdds_sample[:, i], c="C{:d}".format(i))
        axs[2].set_xlabel("Time (s)")
        axs[0].set_ylabel("Position (rad)")
        axs[1].set_ylabel("Velocity (rad/s)")
        axs[2].set_ylabel("Acceleration (rad/s2)")
        plt.show()
        