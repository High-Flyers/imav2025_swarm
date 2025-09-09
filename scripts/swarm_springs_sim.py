import numpy as np
import matplotlib.pyplot as plt
import matplotlib

matplotlib.use('TkAgg')

SPRING_LENGTH = 3.0
SPRING_CONSTANT = 1.0

class Drone:
    def __init__(self, id, position: tuple):
        self.id = id
        self.position = np.array(position, dtype=float)
        self.velocity = np.zeros(len(position), dtype=float)

    def set_velocity(self, velocity: tuple):
        self.velocity = np.array(velocity, dtype=float)
    
    def update(self, dt: float):
        self.position += self.velocity * dt

class Swarm:
    def __init__(self, initial_positions: list):
        self.drones = [Drone(id, position) for id, position in enumerate(initial_positions)]

    def compute_velocities_2d(self):
        velocities = [np.zeros(2)] * len(self.drones)
        for i, drone_a in enumerate(self.drones):
            counter = 0
            min_dinstance = float('inf')
            tmp_velocity = np.zeros(2)
            for j, drone_b in enumerate(self.drones):
                if i != j:
                    displacement = drone_b.position - drone_a.position
                    distance = np.linalg.norm(displacement)
                    direction = displacement / distance if distance != 0 else np.zeros(2)
                    velocity_magnitude = ((distance - SPRING_LENGTH) / SPRING_CONSTANT) ** 2
                    velocity = velocity_magnitude * direction
                
                    if distance < min_dinstance:
                        min_dinstance = distance
                        tmp_velocity = velocity

                    if distance < SPRING_LENGTH * 1.2:
                        counter += 1
                        velocities[i] += velocity_magnitude * direction
            
            if counter == 0:
                velocities[i] = tmp_velocity
            else:
                velocities[i] /= counter

        return velocities

    def update(self, dt: float):
        velocities = self.compute_velocities_2d()
        for drone, velocity in zip(self.drones, velocities):
            velocity = np.clip(velocity, 0.0, 4.0)
            drone.set_velocity(velocity * dt)
            drone.update(dt)

def animate_swarm_2d(swarm: Swarm, steps: int, dt: float):
    plt.ion()
    fig, ax = plt.subplots()
    ax.set_xlim(-10, 10)
    ax.set_ylim(-10, 10)
    scat = ax.scatter([], [])

    for _ in range(steps):
        swarm.update(dt)
        positions = np.array([drone.position for drone in swarm.drones])
        scat.set_offsets(positions)
        plt.draw()
        plt.pause(0.1)

    plt.ioff()
    plt.show()

if __name__ == "__main__":
    initial_positions = [(np.random.uniform(-5, 5), np.random.uniform(-5, 5)) for _ in range(10)]
    swarm = Swarm(initial_positions)
    animate_swarm_2d(swarm, steps=1000, dt=0.1)