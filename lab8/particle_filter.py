import numpy as np
from particle import Particle
from scipy.special import logsumexp
import math
import copy


class ParticleFilter:
    def __init__(self, virtual_create, variance_sensor, variance_distance, variance_direction, num_particles,
                 world_width, world_height):
        self.virtual_create = virtual_create

        #
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # constant
        self.variance_sensor = variance_sensor
        self.variance_distance = variance_distance
        self.variance_direction = variance_direction
        self.sd_sensor = np.sqrt(self.variance_sensor)
        self.sd_distance = np.sqrt(self.variance_distance)
        self.sd_direction = np.sqrt(self.variance_direction)

        self.numParticles = num_particles
        self.world_width = world_width  # the x
        self.world_height = world_height  # the y

        self.weights = np.empty([self.numParticles, ])
        self.particles = []

        for i in range(self.numParticles):
            x = np.random.uniform(0, self.world_width)
            y = np.random.uniform(0, self.world_height)
            theta = np.random.uniform(0, 2 * np.pi)
            prev_log_prob = math.log(1 / self.numParticles)

            particle = Particle(x, y, theta, prev_log_prob,
                                sd_sensor=self.sd_sensor,
                                sd_distance=self.sd_distance,
                                sd_direction=self.sd_direction)

            self.particles.append(particle)

    def resample(self):
        index = np.random.choice(np.arange(0, self.numParticles), self.numParticles, replace=True,
                                 p=np.exp(self.weights))
        particles = []
        for i in index:
            particles.append(copy.deepcopy(self.particles[i]))

        return particles

    def move(self, turn, distance):
        if turn == 0 and distance == 0:
            return

        for particle in self.particles:
            particle.move(turn, distance)

        self.estimate()

    def sense(self, sensor_reading):
        def set_prev_prob(particle_, weight_):
            particle_.prev_log_prob = weight_

        for i in range(self.numParticles):
            weight = self.particles[i].sense(sensor_reading)
            self.weights[i] = weight

        self.weights -= logsumexp(self.weights)
        np.vectorize(set_prev_prob)(self.particles, self.weights)

        self.particles = self.resample()

        self.estimate()

    def estimate(self):
        x_avg = np.average(np.vectorize(lambda particle: particle.x)(self.particles))

        y_avg = np.average(np.vectorize(lambda particle: particle.y)(self.particles))
        theta_avg = np.average(np.vectorize(lambda particle: particle.theta)(self.particles))

        self.x = x_avg
        self.y = y_avg
        self.theta = theta_avg

        # draw the estimated position and all other particles
        self.virtual_create.set_pose((x_avg, y_avg, 0.1), theta_avg)
