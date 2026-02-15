import numpy as np

vortex_strength = 0.0  # default

def set_strength(s):
    global vortex_strength
    vortex_strength = s

def get_current(x, y):

    center_x = 25
    center_y = 25

    dx = x - center_x
    dy = y - center_y

    r_squared = dx**2 + dy**2
    r_squared = max(r_squared, 10)

    u_c = -vortex_strength * dy / r_squared
    v_c = vortex_strength * dx / r_squared

    return u_c, v_c
