from vehicle import AUV
from ocean import get_current

def run_simulation(total_time, dt):

    auv = AUV(x=0, y=0, psi=0, speed=1.0)

    x_traj = []
    y_traj = []

    time_steps = int(total_time / dt)

    for _ in range(time_steps):

        # Get current at current position
        u_c, v_c = get_current(auv.x, auv.y)

        # Update vehicle
        auv.update(dt, u_c, v_c)

        # Store position (INSIDE LOOP)
        x, y = auv.get_position()
        x_traj.append(x)
        y_traj.append(y)

    total_energy = auv.get_energy()

    return x_traj, y_traj, total_energy
