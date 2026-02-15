import matplotlib.pyplot as plt
import numpy as np
from ocean import get_current

def plot_trajectory(x_traj, y_traj):
    plt.figure(figsize=(8,6))
    plt.plot(x_traj, y_traj, linewidth=2)

    plt.xlabel("X Position (m)")
    plt.ylabel("Y Position (m)")
    plt.title("AUV Motion Under Ocean Current")

    plt.grid(True)

    # Force visible limits
    plt.xlim(-5, 80)
    plt.ylim(-20, 20)

    plt.show()



def plot_current_field():
    x_vals = np.linspace(-5, 50, 20)
    y_vals = np.linspace(-10, 10, 20)

    X, Y = np.meshgrid(x_vals, y_vals)

    U = np.zeros_like(X)
    V = np.zeros_like(Y)

    for i in range(len(x_vals)):
        for j in range(len(y_vals)):
            u_c, v_c = get_current(X[j,i], Y[j,i])
            U[j,i] = u_c
            V[j,i] = v_c

    plt.figure(figsize=(8,6))
    plt.quiver(X, Y, U, V)
    plt.title("Ocean Current Field")
    plt.xlabel("X Position")
    plt.ylabel("Y Position")
    plt.grid(True)
