import numpy as np

class AUV:
    def __init__(self, x, y, psi, speed):
        self.x = x
        self.y = y
        self.psi = psi
        self.speed = speed

        # Energy parameters
        self.rho = 1025
        self.Cd = 0.8
        self.A = 0.3
        self.total_energy = 0

    def update(self, dt, u_c, v_c):

        # ----- MOTION -----
        dx = self.speed * np.cos(self.psi) + u_c
        dy = self.speed * np.sin(self.psi) + v_c

        self.x += dt * dx
        self.y += dt * dy

        # ----- ENERGY -----
        # ----- ENERGY (physically correct) -----

# Velocity relative to water
        v_x = self.speed * np.cos(self.psi)
        v_y = self.speed * np.sin(self.psi)

        v_rel_x = v_x - u_c
        v_rel_y = v_y - v_c

        v_rel = np.sqrt(v_rel_x**2 + v_rel_y**2)

        drag = 0.5 * self.rho * self.Cd * self.A * v_rel**2
        power = drag * v_rel

        self.total_energy += power * dt


    def get_position(self):
        return self.x, self.y

    def get_energy(self):
        return self.total_energy
