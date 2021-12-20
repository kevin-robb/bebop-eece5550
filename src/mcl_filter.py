#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rosgraph_msgs.msg import Clock
import numpy as np
from scipy.linalg import expm
from scipy.stats import multivariate_normal, norm, expon, uniform
from math import log, exp, round, asin, sin, cos, radians
from random import choices
import cv2


class MCL_Filter:
    # filter parameters
    PARTICLE_SET_SIZE = 100
    PROCESS_NOISE_COV = np.array([1, 1/2, 1/3], 
                                [1/2, 1/3, 1/4], 
                                [1/3, 1/4, 1/5])
    dt = None
    particle_set = None
    # map parameters
    MAP_RESOLUTION  = 0.05 #[m]
    MAP_OCC = None
    MAP_EDT = None
    # mixture model parameters
    W_HIT, W_SHORT, W_MAX, W_RAND = 0.75, 0.05, 0.10, 0.10 # tune these. keep their sum=1.
    SIGMA_HIT = 0.03 # 'on the order of a few centimeters'
    LAMBDA_SHORT = 0.1 # tune this
    Z_MIN, Z_MAX = 0.12, 3.5  # LIDAR Range 
    DZ = 0.015 # ~ LiDAR resolution.
    


    def __init__(self, map_filepath):
        self.init_particle_set(map_filepath)

    # SETUP FUNCTIONS
    def init_particle_set(self, map_filepath):
        """
        Initialize the particles uniformly across the entire map.
        Thresholded map is available
            -> Randomly select free cells in the occupancy map
            -> Assign particles to those cells along with a random orientation value [0,360]
        Inputs: occupancy_map {(X,Y)->(row, column)}(binarized)
        Output: np.array {SET_SIZE x 3}->(X,Y,theta) for all particles
        """
        [occ_map, rows, cols] = self.get_map_CV(map_filepath)
        flat_map = np.divide(occ_map[1].reshape(1, rows * cols), 255)  # Flatten map and normalize cell values
        free = np.where(flat_map == 1)  # Get linear index of points where cell is unoccupied
        # Get (X,Y) values of cells after uniformly sampling free cells
        particle_coords = np.unravel_index(np.random.choice(free[1], size=self.PARTICLE_SET_SIZE, replace=True, p=None), (rows, cols))
        # Assigns a [3,100] np array to particle set, where each column {particle_set[:,i] corresponds to a pose (X,Y,theta)
        self.particle_set = np.stack((particle_coords[0], particle_coords[1], np.random.rand(self.PARTICLE_SET_SIZE) * 360))

    def get_map_CV(self, map_filepath):
        """
        Read in the map from the PGM file and convert it to a format that can be used for raytracing.
        """
        occ_map = cv2.threshold(cv2.imread(map_filepath, 0), 127, 255, cv2.THRESH_BINARY)
        print("Map read in with shape ", occ_map[1].shape)
        cv2.imshow("Thresholded Map", occ_map[1])
        rows  = occ_map[1].shape[0]
        cols  = occ_map[1].shape[1]
        # make sure this is set to 'MAP' globally and can be used for raytracing.
        self.MAP_OCC = occ_map
        self.get_map_edt(self.MAP_OCC[1])
        return [occ_map, rows, cols]

    def get_map_edt(self, map):
        """
        Perform Euclidean Distance Transform using OpenCV.
        From the binary occupancy grid map, obtain a grid where 
            each cell contains the euclidean distance to the nearest obstacle.
        This will allow us to perform faster, hackier raycasting.
        """
        MAP_EDT = cv2.distanceTransform(map, distanceType=cv2.DIST_L2, maskSize=5)
        self.MAP_EDT = np.multiply(MAP_EDT, self.MAP_RESOLUTION)

    # MAIN SETUP FUNCTIONS
    def main_mcl_loop(self, U, Z, dt):
        """
        Given the prior particle set, last motion command, current sensor scan, and map m.
        Output the posterior particle set.
        """
        # prediction step.
        self.particle_set = self.motion_model_sampler(self.particle_set, U, dt)
        # compute normalized particle weights (requires ray-casting on map).
        nonnormalized_weights = [self.sensor_likelihood_func(Z, x) for x in self.particle_set]
        total_w = sum(nonnormalized_weights)
        ll_weights = [w / total_w for w in nonnormalized_weights]
        # resampling step.
        self.particle_set = self.resampling_func(self.particle_set, ll_weights)

    def motion_model(self, x_0, U, dv, dt):
        """
        Velocity motion model.
        Use current state X and motor commands U to perform forward kinematics for one timestep.
        X in SE(2) = initial noise = multivariate_normal.rvs(mean=None, cov=PROCESS_NOISE_COV, size=SET_SIZE)l (current) pose of robot.
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

    def motion_model_sampler(self, X_set, U, dt): 
        """
        Implementation of motion model sampling function.
        X_set = list of poses of all particles.
        U = (dx, dtheta)^T = commanded velocities.
        PROCESS_NOISE_COV = symmetric positive-definite matrix that parameterizes 
            a mean-zero Gaussian distribution over velocity noise.
        """
        # create process noise distribution.
        noise = multivariate_normal.rvs(mean=None, cov=self.PROCESS_NOISE_COV, size=self.PARTICLE_SET_SIZE)
        for k in range(X_set):
            # sample a random realization of the process noise. 
            # apply motion model (a) to predict next pose for this particle.
            X_set[k] = self.motion_model(X_set[k], U, noise[k], dt)
        return X_set

    def mixture_model(self, z_star, z_true=0):
        """
        Sensor model that will find the likelihood of getting our true measurement z_true given this model.
        This depends on the measurement z_star that we would have gotten if our pose X and map M are actually correct.
        """
        # Use cumulative distribution function to find probability of 
        #   the true measurement given this distribution.
        p_hit = norm.cdf(z_true+self.DZ, loc=z_star, scale=self.SIGMA_HIT) - norm.cdf(z_true-self.DZ, loc=z_star, scale=self.SIGMA_HIT)
        p_short = expon.cdf(z_true+self.DZ, scale=1/self.LAMBDA_SHORT) - expon.cdf(z_true-self.DZ, scale=1/self.LAMBDA_SHORT)
        p_max = uniform.cdf(z_true+self.DZ, loc=self.Z_MAX, scale=0.01) - uniform.cdf(z_true-self.DZ, loc=self.Z_MAX, scale=0.01)
        p_rand = uniform.cdf(z_true+self.DZ, loc=0, scale=self.Z_MAX) - uniform.cdf(z_true-self.DZ, loc=0, scale=self.Z_MAX)
        model = self.W_HIT * p_hit + self.W_SHORT * p_short + self.W_MAX * p_max + self.W_RAND * p_rand
        return model

    def raycasting(self, X, Z_true):
        """
        Given a pose X and the true measurements Z_true.
        Use the EDT map to find the approx error of the measurement under this assumption.
        """
        Z_error = []
        for k in range(len(Z_true)):
            # start from position of X.
            pos = (X[0][2], X[1][2])
            # using X's yaw and z_true, find destination cell.
            # also take into account what angle of LiDAR beam we're looking at.
            # => treat index as degree 
            # TODO check directions CCW vs CW of LiDAR beams, and which direction is yaw=0.
            yaw = asin(X[1][0])
            offset = (Z_true[k]*cos(yaw+radians(k)), Z_true[k]*sin(yaw+radians(k)))
            # offset = (Z_true[k]*X[0][0], Z_true[k]*X[1][0])
            dest_pos = (pos[0]+round(offset[0]), pos[1]+round(offset[1]))
            # value in destination cell would be 0 (occupied) if correct.
            # => error is value in destination cell.
            err = self.MAP_EDT[dest_pos[0]][dest_pos[1]]
            Z_error += [err]
        return Z_error

    def sensor_likelihood_func(self, Z_true, X):
        """
        Given the true laser scan measurements Z, a single pose X, and the map M.
        Find the log likelihood of measuring this scan given the actual.
        LIDAR SPECS: Angular Resolution: 1 Degree ==> 360 measurements per particle are obtained.
        """
        # raycasting to get measurement 'errors' for the given pose X.
        Z_error = self.raycasting(X)

        # loop through each error from applying raycasting to a certain pose X,
        # and calculate the log likelihood of this pose given error from EDT.
        ll_weight = sum([log(self.mixture_model(Z_error[k], 0)) for k in range(len(Z_true))])

        # return log likelihood weight for this particle.
        return ll_weight

    def resampling_func(self, particles, ll_weights):
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
        particles = choices(particles, probabilities, self.PARTICLE_SET_SIZE)
        return particles















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
