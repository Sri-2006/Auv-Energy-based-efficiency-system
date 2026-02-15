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

    # Set vortex strength
    set_strength(5)

    # Create obstacle map
    obstacle_map = np.zeros((height, width))

    # Vertical wall obstacle
    for y in range(10, 40):
        obstacle_map[y][25] = 1

    # -------- Shortest Path Planner --------
    planner1 = AStarPlanner(width, height, get_current, obstacle_map)
    planner1.use_energy = False
    shortest_path = planner1.plan(start, goal)

    # -------- Energy Optimal Planner --------
    planner2 = AStarPlanner(width, height, get_current, obstacle_map)
    planner2.use_energy = True
    energy_path = planner2.plan(start, goal)

    # -------- Compute Metrics --------
    E_shortest = compute_path_energy(shortest_path, get_current)
    E_energy = compute_path_energy(energy_path, get_current)

    L_shortest = compute_path_length(shortest_path)
    L_energy = compute_path_length(energy_path)

    percent_saved = (E_shortest - E_energy) / E_shortest * 100

    print("\n------ FLOW + OBSTACLE ANALYSIS ------")
    print(f"Shortest Path Energy: {E_shortest:.2f} J")
    print(f"Energy-Optimal Path Energy: {E_energy:.2f} J")
    print(f"Energy Saved: {percent_saved:.2f}%\n")

    print(f"Shortest Path Length: {L_shortest:.2f}")
    print(f"Energy-Optimal Path Length: {L_energy:.2f}")

    # -------- Plot --------
    x_short = [p[0] for p in shortest_path]
    y_short = [p[1] for p in shortest_path]

    x_energy = [p[0] for p in energy_path]
    y_energy = [p[1] for p in energy_path]

    plt.figure(figsize=(8,8))

    # Plot obstacle cells
    for y in range(height):
        for x in range(width):
            if obstacle_map[y][x] == 1:
                plt.scatter(x, y, color='black', s=10)

    plt.plot(x_short, y_short, 'b--', label='Shortest Path')
    plt.plot(x_energy, y_energy, 'r-', linewidth=2, label='Energy-Optimal Path')

    plt.scatter(start[0], start[1], color='green', s=100, label='Start')
    plt.scatter(goal[0], goal[1], color='purple', s=100, label='Goal')

    plt.grid(True)
    plt.legend()
    plt.title("Flow + Geometry Interaction")
    plt.show()


if __name__ == "__main__":
    main()
