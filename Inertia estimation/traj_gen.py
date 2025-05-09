import numpy as np

def generate_sin_trajectory(amplitude, f, motion_time):
    """
    Sine trajectory generator (Python version)
    Args:
        amplitude (float): 진폭
        f (float): 주파수 [Hz]
        motion_time (float or np.ndarray): 시간(초)
    Returns:
        qdes, qdotdes, qdotdotdes: 위치, 속도, 가속도
    """
    omega = 2 * np.pi * f
    qdes = amplitude * (1 - np.cos(omega * motion_time))
    qdotdes = amplitude * omega * np.sin(omega * motion_time)
    qddotdes = amplitude * omega**2 * np.cos(omega * motion_time)
    return qdes, qdotdes, qddotdes

time = 5
freq = 1 / (2 * time)

amplitude = 1.0
ctr_freq = 4000

motion_time = np.linspace(0, time, int(time * ctr_freq))

qdes, qdotdes, qddotdes = generate_sin_trajectory(amplitude, freq, motion_time)

print(max(qdes))
print(max(qdotdes))
print(max(qddotdes))

# CSV 파일로 궤적 저장
np.savetxt('trajectory_generated/traj_pos.csv', qdes, delimiter=',')
np.savetxt('trajectory_generated/traj_vel.csv', qdotdes, delimiter=',')
np.savetxt('trajectory_generated/traj_acc.csv', qddotdes, delimiter=',')

import matplotlib.pyplot as plt

plt.figure(figsize=(12, 8))

plt.subplot(3, 1, 1)
plt.plot(motion_time, qdes)
plt.title('qdes (Position)')
plt.ylabel('Position')
plt.grid(True)

plt.subplot(3, 1, 2)
plt.plot(motion_time, qdotdes)
plt.title('qdotdes (Velocity)')
plt.ylabel('Velocity')
plt.grid(True)

plt.subplot(3, 1, 3)
plt.plot(motion_time, qddotdes)
plt.title('qddotdes (Acceleration)')
plt.xlabel('Time [s]')
plt.ylabel('Acceleration')
plt.grid(True)

plt.tight_layout()
plt.show()
