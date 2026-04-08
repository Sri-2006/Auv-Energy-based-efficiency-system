import numpy as np

global_strength = 5

def set_strength(s):
    global global_strength
    global_strength = s
    (25, 25, global_strength)

def get_current(x, y, z=0):

    vortices = [
        (25, 25, 5),
        (15, 35, -4),
        (40, 20, 3),
    ]

    u_total = 0
    v_total = 0

    for xc, yc, S in vortices:

        dx = x - xc
        dy = y - yc
        r2 = dx**2 + dy**2 + 1e-5

        u = -S * dy / r2
        v =  S * dx / r2

        # 🌊 Depth scaling
        depth_factor = np.exp(-z / 10)

        u_total += u * depth_factor
        v_total += v * depth_factor

    return u_total, v_total