import numpy as np
from particle import Particle
from scipy.special import logsumexp


class ParticleFilter:
    def __init__(self, create, virtual_create, go_to_angle, num_particles, sleep):
        self.create = create
        self.virtual_create = virtual_create

        # odometry and turn function
        self.go_to_angle = go_to_angle
        self.sleep = sleep

        # constant
        self.variance_sensor = 0.1
        self.variance_distance = 0.01
        self.variance_direction = 0.05
        self.sd_sensor = np.sqrt(self.variance_sensor)
        self.sd_distance = np.sqrt(self.variance_distance)
        self.sd_direction = np.sqrt(self.variance_direction)

        self.numParticles = num_particles
        self.world_width = 3.0  # the x
        self.world_height = 3.0  # the y

        self.weights = np.empty([self.numParticles, ])
        self.particles = []

        for i in range(self.numParticles):
            x = np.random.uniform(0, self.world_width)
            y = np.random.uniform(0, self.world_height)
            theta = np.random.uniform(0, 2 * np.pi)
            particle = Particle(self.create, x, y, 1 / self.numParticles, theta, self.variance_sensor,
                                self.variance_distance,
                                self.variance_direction, go_to_angle=go_to_angle, sleep=self.sleep)

            self.particles.append(particle)

    def resample(self):
        return np.random.choice(self.particles, self.numParticles, replace=True, p=self.weights)

    def move(self, turn, distance):
        if turn == 0 and distance == 0:
            return

        for particle in self.particles:
            particle.move(turn, distance)

        self.estimate()

    def sense(self, sensor_reading):
        i = -1
        for particle in self.particles:
            weight = particle.sense(sensor_reading)
            self.weights[++i] = weight

        self.weights -= logsumexp(self.weights)
        self.particles = self.resample()

    def drawParticles(self):
        data = []

        # get position data from all particles
        for particle in self.particles:
            data.extend([particle.x, particle.y, 0.1, particle.theta])

            # paint all particles in simulation
            self.virtual_create.set_point_cloud(data)

    def estimate(self):
        x_avg = np.average(np.vectorize(lambda particle: particle.x)(self.particles))

        y_avg = np.average(np.vectorize(lambda particle: particle.y)(self.particles))
        theta_avg = np.average(np.vectorize(lambda particle: particle.theta)(self.particles))

        # draw the estimated position and all other particles
        self.virtual_create.set_pose((x_avg, y_avg, 0.1), theta_avg)
        self.drawParticles()
