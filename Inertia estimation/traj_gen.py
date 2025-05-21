import os
import numpy as np
import matplotlib.pyplot as plt

# Define the base directory for saving files
base_dir = r'c:\Users\meser\Documents\GithubForClion\Neuromeka\Projects\Internship\PID_control\Inertia estimation\trajectory_generated'
def compute_phase_durations(D, v_max, a_max, j_max):
    """
    Computes phase durations (assuming symmetric profile) according to the paper.
    """
    Tj = a_max / j_max
    Ta = 2 * Tj
    Da = a_max * Ta**2 / 2  # Displacement during accel

    Tv = (D - 2 * Da) / v_max  # Constant velocity phase
    if Tv < 0:
        # No constant velocity phase: recalculate with max acceleration and jerk only
        Tj = (D / (2 * j_max)) ** (1/3)
        Ta = 2 * Tj
        Tv = 0
    T = 2 * Ta + Tv
    return Tj, Tj, Tv

# def generate_jerk_continuous_scurve(D, v_max, a_max, j_max, dt=0.001):
#     """
#     Full 15-phase jerk-continuous S-curve implementation for 1-DOF.
#     """
#     Tj1, Tj2, Tv = compute_phase_durations(D, v_max, a_max, j_max)

#     # Compute time durations
#     T1 = Tj1
#     T2 = Tj2
#     T3 = Tj1
#     T4 = Tv
#     T_total = 2 * (T1 + T2 + T3) + T4

#     # Phase time thresholds
#     t_thresholds = np.cumsum([T1, T2, T3, T4, T3, T2, T1])
#     t_segments = np.concatenate([[0], t_thresholds])
#     t = np.arange(0, t_segments[-1], dt)

#     # Initialize profiles
#     jerk = np.zeros_like(t)
#     acc = np.zeros_like(t)
#     vel = np.zeros_like(t)
#     pos = np.zeros_like(t)

#     # Define jerk pattern (15 phases)
#     for i, ti in enumerate(t):
#         if ti < t_segments[1]:
#             jerk[i] = +j_max
#         elif ti < t_segments[2]:
#             jerk[i] = 0
#         elif ti < t_segments[3]:
#             jerk[i] = -j_max
#         elif ti < t_segments[4]:
#             jerk[i] = 0
#         elif ti < t_segments[5]:
#             jerk[i] = -j_max
#         elif ti < t_segments[6]:
#             jerk[i] = 0
#         elif ti < t_segments[7]:
#             jerk[i] = +j_max
#         else:
#             jerk[i] = 0

#     # Integrate
#     for i in range(1, len(t)):
#         acc[i] = acc[i-1] + jerk[i] * dt
#         vel[i] = vel[i-1] + acc[i] * dt
#         pos[i] = pos[i-1] + vel[i] * dt

#     # Scale displacement to match D exactly
#     pos *= D / pos[-1]
#     vel *= D / pos[-1]
#     acc *= D / pos[-1]

#     return t, pos, vel, acc, jerk

def generate_jerk_continuous_scurve(start_pos, end_pos, v_max, a_max, j_max, dt=0.001):
    """
    Full 15-phase jerk-continuous S-curve implementation for 1-DOF.
    Args:
        start_pos (float): Starting position.
        end_pos (float): Target position.
        v_max (float): Maximum velocity.
        a_max (float): Maximum acceleration.
        j_max (float): Maximum jerk.
        dt (float): Time step.
    Returns:
        t, pos, vel, acc, jerk: Time, position, velocity, acceleration, and jerk profiles.
    """
    # Calculate displacement
    D = end_pos - start_pos
    direction = np.sign(D)  # Determine the direction of motion (+1 or -1)

    # Compute phase durations
    Tj1, Tj2, Tv = compute_phase_durations(abs(D), v_max, a_max, j_max)

    # Compute time durations
    T1 = Tj1
    T2 = Tj2
    T3 = Tj1
    T4 = Tv
    T_total = 2 * (T1 + T2 + T3) + T4

    # Phase time thresholds
    t_thresholds = np.cumsum([T1, T2, T3, T4, T3, T2, T1])
    t_segments = np.concatenate([[0], t_thresholds])
    t = np.arange(0, t_segments[-1], dt)

    # Initialize profiles
    jerk = np.zeros_like(t)
    pos = np.zeros_like(t)

    # Define jerk pattern (15 phases)
    for i, ti in enumerate(t):
        if ti < t_segments[1]:
            jerk[i] = +j_max
        elif ti < t_segments[2]:
            jerk[i] = 0
        elif ti < t_segments[3]:
            jerk[i] = -j_max
        elif ti < t_segments[4]:
            jerk[i] = 0
        elif ti < t_segments[5]:
            jerk[i] = -j_max
        elif ti < t_segments[6]:
            jerk[i] = 0
        elif ti < t_segments[7]:
            jerk[i] = +j_max
        else:
            jerk[i] = 0

    # Integrate jerk to compute acceleration
    acc = np.cumsum(jerk) * dt

    # Integrate acceleration to compute velocity
    vel = np.cumsum(acc) * dt

    # Integrate velocity to compute position
    pos = np.cumsum(vel) * dt

    # Scale displacement to match D exactly
    pos *= abs(D) / pos[-1]

    # Adjust position to start from start_pos
    pos = direction * pos + start_pos

    # Adjust velocity, acceleration, and jerk for the direction of motion
    vel *= direction
    acc *= direction
    jerk *= direction

    return t, pos, vel, acc, jerk

def generate_multi_cycle_trajectory(start_pos, end_pos, v_max, a_max, j_max, dt, cycles):
    """
    Generate a multi-cycle trajectory alternating between forward and backward directions.
    Ensures smooth transitions by starting each cycle from the last position of the previous cycle.
    Args:
        start_pos (float): Starting position of the first cycle.
        end_pos (float): Target position of the first cycle.
        v_max (float): Maximum velocity.
        a_max (float): Maximum acceleration.
        j_max (float): Maximum jerk.
        dt (float): Time step.
        cycles (int): Number of cycles (forward and backward).
    Returns:
        t, pos, vel, acc, jerk: Combined time, position, velocity, acceleration, and jerk profiles.
    """
    total_t = []
    total_pos = []
    total_vel = []
    total_acc = []
    total_jerk = []

    # Initialize starting position, velocity, and time offset
    current_start_pos = start_pos
    current_end_pos = end_pos
    time_offset = 0

    for cycle in range(cycles):
        # Generate trajectory for the current cycle
        t, pos, vel, acc, jerk = generate_jerk_continuous_scurve(
            current_start_pos, current_end_pos, v_max, a_max, j_max, dt
        )

        # Adjust time to ensure continuity
        t += time_offset

        # Update the last position and time offset for the next cycle
        time_offset = t[-1] + dt

        # Append the current cycle's data to the total trajectory
        total_t.append(t)
        total_pos.append(pos)
        total_vel.append(vel)
        total_acc.append(acc)
        total_jerk.append(jerk)

        # Alternate direction for the next cycle
        current_start_pos, current_end_pos = current_end_pos, current_start_pos

    # Concatenate all cycles
    t = np.concatenate(total_t)
    pos = np.concatenate(total_pos)
    vel = np.concatenate(total_vel)
    acc = np.concatenate(total_acc)
    jerk = np.concatenate(total_jerk)

    return t, pos, vel, acc, jerk

# # Parameters

# # for position-error identification
# start_pos = 0.0     # radians
# end_pos = 2 * np.pi # radians
# v_max = 0.05           # rad/s (velocity limit for safety/identification clarity)
# a_max = 0.15         # rad/s² (very high acceleration for inertia excitation)
# j_max = 0.8        # rad/s³ (to ensure rapid acceleration rise)

# for inertia-excitation
start_pos = 0.0     # radians
end_pos = -2 * np.pi # radians
v_max = 1           # rad/s (velocity limit for safety/identification clarity)
a_max = 20         # rad/s² (very high acceleration for inertia excitation)
j_max = 150        # rad/s³ (to ensure rapid acceleration rise)

dt = 0.00025        # Time step
cycles = 4          # Number of cycles (forward and backward)

# Generate multi-cycle trajectory
t, pos, vel, acc, jerk = generate_multi_cycle_trajectory(start_pos, end_pos, v_max, a_max, j_max, dt, cycles)

# Save trajectory data to CSV files
np.savetxt(os.path.join(base_dir, 'multi_traj_pos.csv'), pos, delimiter=',')
np.savetxt(os.path.join(base_dir, 'multi_traj_vel.csv'), vel, delimiter=',')
np.savetxt(os.path.join(base_dir, 'multi_traj_acc.csv'), acc, delimiter=',')
np.savetxt(os.path.join(base_dir, 'multi_traj_jerk.csv'), jerk, delimiter=',')

# Plot the trajectory
plt.figure(figsize=(10, 8))
plt.subplot(4, 1, 1)
plt.plot(t, pos)
plt.ylabel("Position")
plt.subplot(4, 1, 2)
plt.plot(t, vel)
plt.ylabel("Velocity")
plt.subplot(4, 1, 3)
plt.plot(t, acc)
plt.ylabel("Acceleration")
plt.subplot(4, 1, 4)
plt.plot(t, jerk)
plt.ylabel("Jerk")
plt.xlabel("Time [s]")
plt.tight_layout()
plt.show()

# start_pos = 0.0     # radians
# end_pos = 2*np.pi     # radians
# v_max = 1        # rad/s (velocity limit for safety/identification clarity)
# a_max = 5.0      # rad/s² (very high acceleration for inertia excitation)
# j_max = 20.0     # rad/s³ (to ensure rapid acceleration rise)
# dt = 0.00025

# t, pos, vel, acc, jerk = generate_jerk_continuous_scurve(start_pos, end_pos, v_max, a_max, j_max, dt)

# np.savetxt(os.path.join(base_dir, 'traj_pos.csv'), pos, delimiter=',')
# np.savetxt(os.path.join(base_dir, 'traj_vel.csv'), vel, delimiter=',')
# np.savetxt(os.path.join(base_dir, 'traj_acc.csv'), acc, delimiter=',')

# plt.figure(figsize=(10, 8))
# plt.subplot(4, 1, 1); plt.plot(t, pos); plt.ylabel("Position")
# plt.subplot(4, 1, 2); plt.plot(t, vel); plt.ylabel("Velocity")
# plt.subplot(4, 1, 3); plt.plot(t, acc); plt.ylabel("Acceleration")
# plt.subplot(4, 1, 4); plt.plot(t, jerk); plt.ylabel("Jerk")
# plt.xlabel("Time [s]")
# plt.tight_layout()
# plt.show()

# def generate_sin_trajectory(amplitude, f, motion_time):
#     """
#     Sine trajectory generator (Python version)
#     Args:
#         amplitude (float): 진폭
#         f (float): 주파수 [Hz]
#         motion_time (float or np.ndarray): 시간(초)
#     Returns:
#         qdes, qdotdes, qdotdotdes: 위치, 속도, 가속도
#     """
#     omega = 2 * np.pi * f
#     qdes = amplitude * (1 - np.cos(omega * motion_time))
#     qdotdes = amplitude * omega * np.sin(omega * motion_time)
#     qddotdes = amplitude * omega**2 * np.cos(omega * motion_time)
#     return qdes, qdotdes, qddotdes

# time = 5
# freq = 1 / (2 * time)

# amplitude = 1.0
# ctr_freq = 4000

# motion_time = np.linspace(0, time, int(time * ctr_freq))

# qdes, qdotdes, qddotdes = generate_sin_trajectory(amplitude, freq, motion_time)

# print(max(qdes))
# print(max(qdotdes))
# print(max(qddotdes))

# # CSV 파일로 궤적 저장
# # Save trajectory data to CSV files
# np.savetxt(os.path.join(base_dir, 'traj_pos.csv'), qdes, delimiter=',')
# np.savetxt(os.path.join(base_dir, 'traj_vel.csv'), qdotdes, delimiter=',')
# np.savetxt(os.path.join(base_dir, 'traj_acc.csv'), qddotdes, delimiter=',')

# import matplotlib.pyplot as plt

# plt.figure(figsize=(12, 8))

# plt.subplot(3, 1, 1)
# plt.plot(motion_time, qdes)
# plt.title('qdes (Position)')
# plt.ylabel('Position')
# plt.grid(True)

# plt.subplot(3, 1, 2)
# plt.plot(motion_time, qdotdes)
# plt.title('qdotdes (Velocity)')
# plt.ylabel('Velocity')
# plt.grid(True)

# plt.subplot(3, 1, 3)
# plt.plot(motion_time, qddotdes)
# plt.title('qddotdes (Acceleration)')
# plt.xlabel('Time [s]')
# plt.ylabel('Acceleration')
# plt.grid(True)

# plt.tight_layout()
# plt.show()
