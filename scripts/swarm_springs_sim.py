
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('TkAgg')

SPRING_CONSTANT = 0.005

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
        self.graph = self._compute_graph()

    @staticmethod
    def generate_circle_positions(num_drones, radius=5.0, center=(0, 0)):
        angles = np.linspace(0, 2 * np.pi, num_drones, endpoint=False)
        return [
            (
                center[0] + radius * np.cos(angle),
                center[1] + radius * np.sin(angle)
            )
            for angle in angles
        ]

    def _compute_graph(self):
        # Fully connected graph: graph[i][j] = distance between drone i and j
        n = len(self.drones)
        graph = np.zeros((n, n))
        for i in range(n):
            for j in range(n):
                if i != j:
                    pos_i = self.drones[i].position
                    pos_j = self.drones[j].position
                    graph[i, j] = np.linalg.norm(pos_i - pos_j)
        return graph


    def compute_velocities_2d(self):
        n = len(self.drones)
        velocities = [np.zeros(2)] * n
        for i, drone_a in enumerate(self.drones):
            for j, drone_b in enumerate(self.drones):
                if i != j:
                    displacement = drone_b.position - drone_a.position
                    distance = np.linalg.norm(displacement)
                    direction = displacement / distance if distance != 0 else np.zeros(2)
                    spring_length = self.graph[i, j]
                    velocity_magnitude = ((distance - spring_length) / SPRING_CONSTANT) ** 2
                    if distance < spring_length:
                        velocity_magnitude = -velocity_magnitude
                    velocity = velocity_magnitude * direction

                    velocities[i] += velocity

            
            velocities[i] /= (len(self.drones) - 1)

        return velocities

    def update(self, dt: float):
        velocities = self.compute_velocities_2d()
        for (i, drone), velocity in zip(enumerate(self.drones), velocities):
            if i == 0:
                velocity = np.array((-10.0, 6.0))
            velocity = np.clip(velocity, -4.0, 4.0)
            drone.set_velocity(velocity * dt)
            drone.update(dt)

def animate_swarm_2d(swarm: Swarm, steps: int, dt: float):
    plt.ion()
    fig, ax = plt.subplots()
    ax.set_xlim(-30, 30)
    ax.set_ylim(-30, 30)
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
    num_drones = 10
    initial_positions = Swarm.generate_circle_positions(num_drones, radius=5.0, center=(0, 0))
    swarm = Swarm(initial_positions)
    animate_swarm_2d(swarm, steps=1000, dt=0.15)