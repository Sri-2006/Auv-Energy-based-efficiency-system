import heapq
from platform import node
import numpy as np

class AStarPlanner:

    def __init__(self, width, height, ocean_func, obstacle_map=None):

        self.width = width
        self.height = height
        self.get_current = ocean_func
        self.obstacle_map = obstacle_map

        self.speed = 1.0
        self.rho = 1025
        self.Cd = 0.8
        self.A = 0.3
        self.use_energy = True


    def heuristic(self, a, b):
        return np.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)

    def get_neighbors(self, node):
        x, y = node
        neighbors = []

        moves = [
            (1,0), (-1,0), (0,1), (0,-1),
            (1,1), (-1,1), (1,-1), (-1,-1)
        ]

        for dx, dy in moves:
            nx = x + dx
            ny = y + dy

            if 0 <= nx < self.width and 0 <= ny < self.height:
                # Check obstacle
                if self.obstacle_map is None or self.obstacle_map[ny][nx] == 0:
                    neighbors.append((nx, ny))

        return neighbors

    def energy_cost(self, current, neighbor):

        x1, y1 = current
        x2, y2 = neighbor

        dx = x2 - x1
        dy = y2 - y1

        distance = np.sqrt(dx**2 + dy**2)

        psi = np.arctan2(dy, dx)

        v_x = self.speed * np.cos(psi)
        v_y = self.speed * np.sin(psi)

        u_c, v_c = self.get_current(x1, y1)

        v_rel_x = v_x - u_c
        v_rel_y = v_y - v_c
        v_rel = np.sqrt(v_rel_x**2 + v_rel_y**2)

        drag = 0.5 * self.rho * self.Cd * self.A * v_rel**2
        power = drag * v_rel

        time = distance / self.speed

        energy = power * time

        return energy

    def plan(self, start, goal):

        open_set = []
        heapq.heappush(open_set, (0, start))

        came_from = {}
        g_score = {start: 0}

        while open_set:

            _, current = heapq.heappop(open_set)

            if current == goal:
                return self.reconstruct_path(came_from, current)

            for neighbor in self.get_neighbors(current):

                if self.use_energy:
                    move_cost = self.energy_cost(current, neighbor)
                else:
                    dx = neighbor[0] - current[0]
                    dy = neighbor[1] - current[1]
                    move_cost = np.sqrt(dx**2 + dy**2)      
                tentative_g = g_score[current] + move_cost

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g

                    f_score = tentative_g + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score, neighbor))

        return None

    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path
