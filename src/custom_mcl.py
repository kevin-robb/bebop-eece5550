#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rosgraph_msgs.msg import Clock
import numpy as np
from scipy.linalg import expm
from scipy.stats import multivariate_normal, norm, expon, uniform
from math import log, exp
from random import choices
import cv2

############ Constants ###################
# ----- Clock parameters -------
prev_time = None
dt = 0
# ----- Particle Filter parameters -------
set_size = 100
particle_set = None
# process noise covariance must be a symmetric positive definite matrix. Using a hilbert matrix for now.
process_noise_cov = np.array([1, 1 / 2, 1 / 3], [1 / 2, 1 / 3, 1 / 4], [1 / 3, 1 / 4, 1 / 5])
map = None
# ------ Data from subscribers --------------
U = np.array([0], [0])
Z = []
##########################################


def timer_callback(event):
    """
    Subscribe to the '/clock' topic.
    This should keep the particle filter synched with a bag file replay.
    Use the previous time to set dt.
    """
    # handle tracking of dt between timesteps.
    global prev_time, dt
    if prev_time is None:
        prev_time = event.clock
    else:
        dt = event.clock - prev_time
        prev_time = event.clock
    # run the particle filter at this timestep.
    mcl(U, Z, map)


def mcl(U, Z, M):  # (e)
    """
    Given the prior particle set, last motion command, current sensor scan, and map m.
    Output the posterior particle set.
    """
    global particle_set
    # prediction step.
    particle_set = motion_model_sampler(particle_set, U)
    # compute normalized particle weights (requires ray-casting on map).
    nonnormalized_weights = [sensor_likelihood_func(Z, x, M) for x in particle_set]
    total_w = sum(nonnormalized_weights)
    ll_weights = [w / total_w for w in nonnormalized_weights]
    # resampling step.
    particle_set = resampling_func(particle_set, ll_weights)


def resampling_func(particles, ll_weights):  # (d) TODO UNFINISHED
    """
    Resample particle set from current set of particles, using log-likelihood weights for proportionality.
    """
    # get LSE term
    l_mean = sum(ll_weights) / len(ll_weights)
    l_diffs = [l - l_mean for l in ll_weights]
    LSE = log(sum([exp(dl) for dl in l_diffs]))
    # compute probabilities
    probabilities = [exp(l - l_mean - LSE) for l in ll_weights]
    # TODO might need to normalize here instead of above.
    # create new particle set by resampling from 'particles' using 'probabilities' as weights.
    particles = choices(particles, probabilities, set_size)
    return particles


def sensor_likelihood_func(Z_true, X, M):  # (c)
    """
    Given the true laser scan measurements Z, a single pose X, and the map M.
    Find the log likelihood of measuring this scan given the actual.
    LIDAR SPECS: Angular Resolution: 1 Degree ==> 360 measurements per particle are obtained.
    """
    # raycasting to get Z_pseudo for the given pose X.
    Z_pseudo = raycasting(X,M)

    # loop through each pseudo measurement from applying raycasting to a certain pose X,
    # and calculate the log likelihood of this pose given Z_psuedo and Z_true.
    ll_weight = sum([log(mixture_model(Z_pseudo[k], Z_true[k])) for k in range(len(Z_true))])

    # return log likelihood weight for this particle.
    return ll_weight


def raycasting(X,M): # TODO UNFINISHED
    """
    Given a pose X and the map M, determine what measurements would be gotten.
    Perform raycasting at all 360 degrees.
    Return a list Z_pseudo of all these pseudo measurements from raycasting. (length = 360)
    Z_pseudo[0] should correspond to the direction the robot is facing, and should proceed CCW (TODO verify direction).
    """
    Z_pseudo = None
    # TODO raycasting

    return Z_pseudo


def mixture_model(z_star, z_true):
    """
    Sensor model that will find the likelihood of getting our true measurement z_true given this model.
    This depends on the measurement z_star that we would have gotten if our pose X and map M are actually correct.
    """
    w_hit, w_short, w_max, w_rand = 0.75, 0.05, 0.10, 0.10 # tune these. keep their sum=1.
    sigma_hit = 0.03 # 'on the order of a few centimeters'
    lambda_short = 0.1 # tune this
    z_min = 0.12  # Min Range LIDAR
    z_max = 3.5  # Max Range LIDAR
    dz = 0.015 # ~ LiDAR resolution. not too important as long as it's constant since we normalize in the end.
    # Use cumulative distribution function to find probability of 
    #   the true measurement given this distribution.
    p_hit = norm.cdf(z_true+dz, loc=z_star, scale=sigma_hit) - norm.cdf(z_true-dz, loc=z_star, scale=sigma_hit)
    p_short = expon.cdf(z_true+dz, scale=1/lambda_short) - expon.cdf(z_true-dz, scale=1/lambda_short)
    p_max = uniform.cdf(z_true+dz, loc=z_max, scale=0.01) - uniform.cdf(z_true-dz, loc=z_max, scale=0.01)
    p_rand = uniform.cdf(z_true+dz, loc=0, scale=z_max) - uniform.cdf(z_true-dz, loc=0, scale=z_max)
    model = w_hit * p_hit + w_short * p_short + w_max * p_max + w_rand * p_rand
    return model


def motion_model_sampler(X_set, U):  # (b)
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


def motion_model(x_0, U, dv):  # (a)
    """
    Velocity motion model.
    Use current state X and motor commands U to perform forward kinematics for one timestep.
    X in SE(2) = initial noise = multivariate_normal.rvs(mean=None, cov=process_noise_cov, size=set_size)l (current) pose of robot.
    U = (dx, d_theta)^T = commanded velocities.
    dv in Lie(SE(2)) = additive process noise.
    """
    dx, dtheta = U[0][0], U[1][0]
    # commanded velocities in Lie(SE(2)).
    omega = np.array([0, -dtheta, dx], [dtheta, 0, 0], [0, 0, 0])
    # add error in commanded velocity.
    omega += dv
    # perform forward kinematics.
    X_next = np.matmul(x_0, expm(dt * omega))
    return X_next


def get_controls(msg):
    """
    Grab the control velocities being sent to the robot from some other node (i.e., teleop).
    Input: msg->twist message
    Return: List containing commanded Linear and Angular velocities
    """
    global U
    U = [msg.linear.x, msg.angular.z]


def get_measurements(msg):
    """
    Get LIDAR Range measurements
    """
    global Z
    Z = msg.ranges


def get_map_CV():
    """
    Read in the map from the PGM file and convert it to a format that can be used for raytracing.
    """
    global map
    occ_map = cv2.threshold(cv2.imread('./RecordedFiles/Lab3Q1.bag_map.pgm', 0), 127, 255, cv2.THRESH_BINARY)
    print("Map read in with shape ", occ_map[1].shape)
    cv2.imshow("Thresholded Map", occ_map[1])
    rows  = occ_map[1].shape[0]
    cols  = occ_map[1].shape[1]
    # make sure this is set to 'map' globally and can be used for raytracing.
    map = occ_map
    return [occ_map, rows, cols]


def init_particle_set():
    """
    Initialize the particles uniformly across the entire map.
    Thresholded map is available
        -> Randomly select free cells in the occupancy map
        -> Assign particles to those cells along with a random orientation value [0,360]
    Inputs: occupancy_map {(X,Y)->(row, column)}(binarized)
    Output: np.array {set_size x 3}->(X,Y,theta) for all particles
    """
    global particle_set, set_size
    [occ_map, rows, cols] = get_map_CV()
    flat_map = np.divide(occ_map[1].reshape(1, rows * cols), 255)  # Flatten map and normalize cell values
    free = np.where(flat_map == 1)  # Get linear index of points where cell is unoccupied
    # Get (X,Y) values of cells after uniformly sampling free cells
    particle_coords = np.unravel_index(np.random.choice(free[1], size=set_size, replace=True, p=None), (rows, cols))
    # Assigns a [3,100] np array to particle set, where each column {particle_set[:,i] corresponds to a pose (X,Y,theta)
    particle_set = np.stack((particle_coords[0], particle_coords[1], np.random.rand(set_size) * 360))


def main():
    rospy.init_node('custom_mcl')
    init_particle_set()

    # subscribe to published commands.
    rospy.Subscriber('/cmd_vel', Twist, get_controls, queue_size=1)

    # subscribe to lidar measurements. TODO check topic.
    rospy.Subscriber('/kobuki/laser/scan', LaserScan, get_measurements, queue_size=1)

    # subscribe to the '/clock' topic since it is being used already to replay the bag file.
    rospy.Subscriber('/clock', Clock, timer_callback)
    # rospy.Timer(rospy.Duration(dt), timer_callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
