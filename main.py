import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation
import matplotlib
matplotlib.use('TkAgg')
from planner import AStarPlanner
from ocean import get_current, set_strength


# -------------------------------------------------
# FLOW FIELD VISUALIZATION
# -------------------------------------------------
def plot_flow_field(ax, get_current):

    X, Y = np.meshgrid(range(0, 50, 2), range(0, 50, 2))
    U = np.zeros_like(X, dtype=float)
    V = np.zeros_like(Y, dtype=float)

    for i in range(X.shape[0]):
        for j in range(X.shape[1]):
            u, v = get_current(X[i][j], Y[i][j])
            U[i][j] = u
            V[i][j] = v

    ax.quiver(X, Y, U, V, color='gray', alpha=0.6)


# -------------------------------------------------
# ANIMATION FUNCTION (FIXED)
# -------------------------------------------------
def animate_path(path):

    fig, ax = plt.subplots()

    # Flow field
    plot_flow_field(ax, get_current)

    x = [p[0] for p in path]
    y = [p[1] for p in path]

    line, = ax.plot([], [], 'b-', linewidth=2)
    point, = ax.plot([], [], 'ro')

    ax.scatter(x[0], y[0], c='green', s=100, label='Start')
    ax.scatter(x[-1], y[-1], c='red', s=100, label='Goal')

    ax.set_xlim(0, 50)
    ax.set_ylim(0, 50)
    ax.set_title("AUV Path Animation")
    ax.grid()
    ax.legend()

    def update(frame):
        line.set_data(x[:frame+1], y[:frame+1])
        point.set_data([x[frame]], [y[frame]])  # IMPORTANT FIX
        return line, point

    # KEY FIX → make animation GLOBAL
    global ani
    ani = animation.FuncAnimation(
        fig,
        update,
        frames=len(path),
        interval=80,
        blit=False,
        repeat=False
    )

    plt.show()


# -------------------------------------------------
# ENERGY COMPUTATION
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
# MAIN FUNCTION
# -------------------------------------------------
def main():

    width = 50
    height = 50

    start = (5, 5)
    goal = (45, 45)

    set_strength(5)

    # Obstacle
    obstacle_map = np.zeros((height, width))
    for y in range(10, 40):
        obstacle_map[y][25] = 1

    # Alpha values
    alpha_values = [0.5, 1, 2, 3, 5]

    paths = []
    energies = []

    # -----------------------------
    # RUN PLANNER
    # -----------------------------
    for a in alpha_values:

        print("Running alpha =", a)

        planner = AStarPlanner(width, height, get_current, obstacle_map)
        planner.use_energy = True
        planner.alpha = a

        path = planner.plan(start, goal)

        if path is None:
            print("No path found!")
            continue

        E = compute_path_energy(path, get_current)

        paths.append(path)
        energies.append(E)

        print(f"Alpha {a}: Energy = {E:.2f}")

    # -----------------------------
    # PLOT PATH COMPARISON
    # -----------------------------
    fig, ax = plt.subplots()

    plot_flow_field(ax, get_current)

    for i, path in enumerate(paths):
        x = [p[0] for p in path]
        y = [p[1] for p in path]
        ax.plot(x, y, marker='o', label=f'α={alpha_values[i]}')

    ax.scatter(start[0], start[1], c='green', s=100, label='Start')
    ax.scatter(goal[0], goal[1], c='red', s=100, label='Goal')

    ax.set_title("Path Comparison with Ocean Current Field")
    ax.set_xlabel("X Position")
    ax.set_ylabel("Y Position")
    ax.legend()
    ax.grid()

    plt.show()

    # -----------------------------
    # ANIMATE BEST PATH
    # -----------------------------
    best_index = np.argmin(energies)
    print("\nAnimating best path (lowest energy)...")

    animate_path(paths[best_index])


# -------------------------------------------------
if __name__ == "__main__":
    main()