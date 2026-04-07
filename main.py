import matplotlib.pyplot as plt
import numpy as np

from planner import AStarPlanner
from ocean import get_current, set_strength


# -------------------------------------------------
# Compute total energy of a given path
# -------------------------------------------------
def compute_path_energy(path, ocean_func):

    speed = 1.0
    rho = 1025
    Cd = 0.8
    A = 0.3

    total_energy = 0

    for i in range(len(path) - 1):

        x1, y1 = path[i]
        x2, y2 = path[i + 1]

        dx = x2 - x1
        dy = y2 - y1

        distance = np.sqrt(dx**2 + dy**2)

        psi = np.arctan2(dy, dx)

        v_x = speed * np.cos(psi)
        v_y = speed * np.sin(psi)

        u_c, v_c = ocean_func(x1, y1)

        v_rel_x = v_x - u_c
        v_rel_y = v_y - v_c
        v_rel = np.sqrt(v_rel_x**2 + v_rel_y**2)

        drag = 0.5 * rho * Cd * A * v_rel**2
        power = drag * v_rel

        time = distance / speed

        total_energy += power * time

    return total_energy


# -------------------------------------------------
# Compute geometric path length
# -------------------------------------------------
def compute_path_length(path):

    total_length = 0

    for i in range(len(path) - 1):
        dx = path[i+1][0] - path[i][0]
        dy = path[i+1][1] - path[i][1]
        total_length += np.sqrt(dx**2 + dy**2)

    return total_length


# -------------------------------------------------
# Main
# -------------------------------------------------
def main():

    width = 50
    height = 50

    start = (5, 5)
    goal = (45, 45)

    set_strength(5)

    # ----- Define obstacle map FIRST -----
    obstacle_map = np.zeros((height, width))

    for y in range(10, 40):
        obstacle_map[y][25] = 1

    # ----- Weighted A* Experiment -----
    alpha_values = [0.5, 1, 2, 3, 5]
    energy_results = []

    for a in alpha_values:

        print("Running alpha =", a)

        planner = AStarPlanner(width, height, get_current, obstacle_map)
        planner.use_energy = True
        planner.alpha = a

        path = planner.plan(start, goal)

        if path is None:
            print("No path found for alpha =", a)
            energy_results.append(float('inf'))
            continue

        print("Path computed")

        E = compute_path_energy(path, get_current)
        print("Energy computed")

        energy_results.append(E)

        print(f"Alpha {a}: Energy = {E:.2f}")


    # ----- Plot Results -----
    plt.figure()
    plt.plot(alpha_values, energy_results, marker='o')
    plt.xlabel("Heuristic Weight (alpha)")
    plt.ylabel("Total Energy (J)")
    plt.title("Effect of Heuristic Weight on Energy Optimality")
    plt.grid(True)
    plt.show()
if __name__ == "__main__":
    main()
