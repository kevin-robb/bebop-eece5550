#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
from scipy.linalg import expm
from scipy.stats import multivariate_normal, norm, expon, uniform
from math import log, exp
import imageio
from random import choices

control_pub = None
tf_listener = None
# Constants:
r = 0.033 # wheel radius (m)
w = 0.16 # chassis width (m)
dt = 0.1 # period for timer_callback
# Particle Filter parameters
X_0 = np.array([1,0,0],[0,1,0],[0,0,1])
set_size = 100
particle_set = None #[X_0 for _ in range(set_size)]
particle_weights = [1/set_size for _ in range(set_size)]
U = np.array([0],[0])
Z = []
# process noise covariance must be a symmetric positive definite matrix.
process_noise_cov = np.array([1,1/2,1/3],[1/2,1/3,1/4],[1/3,1/4,1/5])
map = None
obstacle_threshold = 0.5

def get_map():
    # TODO redo this to use the PGM file directly.
    # read in map from PNG.
    rgb_map = imageio.imread('Lab3Q1.bag_map.png')
    print("map read in with shape ", rgb_map.shape)
    # convert to grayscale for easy thresholding.
    r, g, b = rgb_map[:,:,0], rgb_map[:,:,1], rgb_map[:,:,2]
    gs_map = (0.2989 * r + 0.5870 * g + 0.1140 * b) / 255
    # threshold it to form a boolean occupancy grid.
    occ_map = [[gs_map[r][c]>obstacle_threshold for c in range(rgb_map.shape[1])] for r in range(rgb_map.shape[0])]
    # TODO make sure this is set to 'map' globally and can be used for raytracing.

    """
    ALTERNATIVE TO RAYTRACING:
    To cheat and avoid raycasting: pseudo likelihood field called "euclidean distance transform".
    Turn map into grid cells, where each grid cell contains value of euclidean distance to nearest obstacle. 
    Then for some proposed pose, use actual measurement as a lookup to find cell where beam should have ended. 
    The value of that cell should be zero if it actually hit a cell.
    Can use negative distances for points inside obstacles (sign distance transform) to account for obstructions earlier than the expected point.
    """
    # perform euclidean distance transform.
    # edt_map = [[0 if occ_map[r][c] else -1 for c in range(rgb_map.shape[1])] for r in range(rgb_map.shape[0])]


def timer_callback(event):
    # TODO may need to change this from a timer with a set period to a subscriber to the /clock topic, to sync with the bag file replay.
    mcl(U,Z,map)

def mcl(U, Z, M):
    """
    Given the prior particle set, last motion command, current sensor scan, and map m.
    Output the posterior particle set.
    """
    global particle_set
    # prediction step.
    particle_set = motion_model_sampler(particle_set, U)
    # compute normalized particle weights (requires raycasting on map).
    nonnormalized_weights = [sensor_likelihood_func(Z, x, M) for x in particle_set]
    total_w = sum(nonnormalized_weights)
    ll_weights = [w / total_w for w in nonnormalized_weights]
    # resampling step.
    particle_set = resampling_func(particle_set, ll_weights)

def resampling_func(particles, ll_weights): # (d) TODO UNFINISHED
    """
    Resample particle set from current set of particles, using log-likelihood weights for proportionality.
    """
    # get LSE term
    l_mean = sum(ll_weights) / len(ll_weights)
    l_diffs = [l-l_mean for l in ll_weights]
    LSE = log(sum([exp(dl) for dl in l_diffs]))
    # compute probabilities
    probabilities = [exp(l - l_mean - LSE) for l in ll_weights]
    # create new particle set by resampling from 'particles' using 'probabilities' as weights.
    particles = choices(particles, probabilities, set_size)
    return particles

def sensor_likelihood_func(Z, X, M): # (c) TODO UNFINISHED
    """
    Given the laser scan Z, a current pose X, and the map M.
    Find the log likelihood of measuring this scan given the actual
    """
    # use the true measurement to get the mixture model and log likelihood.
    ll = 0
    for z in Z:
        # Z is a list of laser scans at all angles.
        # TODO raycasting to get z_star for each pose.
        mm = mixture_model(z)
        ll += log(mm)

    # TODO return weight for this particle


def mixture_model(z_star):
    # NOTE this doesn't actually depend on particles or map?
    w_hit,w_short,w_max,w_rand = 0.25, 0.25, 0.25, 0.25
    sigma_hit = 0.1
    lambda_short = 0.1
    z_max = 10 #LiDAR parameter
    p_hit = norm.rvs(loc=z_star, scale=sigma_hit, size=set_size)
    p_short = expon.rvs(scale=1/lambda_short, size=set_size)
    p_max = uniform.rvs(loc=z_max, scale=0.01, size=set_size)
    p_rand = uniform.rvs(loc=0, scale=z_max, size=set_size)
    model = w_hit*p_hit + w_short*p_short + w_max*p_max + w_rand*p_rand
    return model

def motion_model_sampler(X_set, U): # (b)
    """
    Implementation of motion model sampling function.
    X_set = list of poses of all particles.
    U = (dx, dtheta)^T = commanded velocities.
    process_noise_cov = symmetric positive-definite matrix that parameterizes a mean-zero Gaussian distribution over velocity noise.
    """
    # create process noise distribution.
    noise = multivariate_normal.rvs(mean=None, cov=process_noise_cov, size=set_size)
    for k in range(X_set):
        # sample a random realization of the process noise. 
        # apply motion model (a) to predict next pose for this particle.
        X_set[k] = motion_model(X_set[k], U, noise[k])
    return X_set

def motion_model(X_0, U, dv): # (a)
    """
    Velocity motion model.
    Use current state X and motor commands U to perform forward kinematics for one timestep.
    X in SE(2) = initial (current) pose of robot.
    U = (dx, dtheta)^T = commanded velocities.
    dv in Lie(SE(2)) = additive process noise.
    """
    dx, dtheta = U[0][0], U[1][0]
    # commanded velocities in Lie(SE(2)).
    omega = np.array([0,-dtheta,dx],[dtheta,0,0],[0,0,0])
    # add error in commanded velocity.
    omega += dv
    # perform forward kinematics.
    X_next = np.matmul(X_0, expm(dt * omega))
    return X_next

def get_controls(msg):
    """
    Grab the control velocities being sent to the robot from some other node (i.e., teleop).
    """
    global U
    U = [msg.linear.x, msg.angular.z]

def get_measurements(msg):
    """
    Get the measurements coming from the LiDAR.
    """
    global Z
    Z = msg.ranges

def init_particle_set():
    global particle_set
    # TODO initialize the particles uniformly across the entire map.
    # make it a list of size 'set_size'.

def main():
    rospy.init_node('custom_mcl')
    init_particle_set()

    # subscribe to published commands.
    rospy.Subscriber('/cmd_vel', Twist, get_controls, queue_size=1)
    # TODO subscribe to lidar measurements.
    rospy.Subscriber('TODO_LIDAR_TOPIC_FROM_BAG_FILE_REPLAY', LaserScan, get_measurements, queue_size=1)

    rospy.Timer(rospy.Duration(dt), timer_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
