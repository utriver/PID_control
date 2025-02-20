import numpy as np
import pandas as pd
from scipy.signal import butter, filtfilt
from scipy.optimize import least_squares
import matplotlib.pyplot as plt
import json
from tqdm import tqdm

# -------------------------------
# 데이터 불러오기 및 전처리
# -------------------------------
# Load dynamic data
# preslide_data = pd.read_csv('/home/user/release/performance_test/RT-data-4.csv')
preslide_data = pd.read_csv('C:/Users/ujin3/Desktop/nrmk/CORE_automation/dynamic_data/dynamic2.csv', skiprows=1)
position = preslide_data.iloc[:, 1].values
torque_measured = preslide_data.iloc[:, 6].values
time = preslide_data.iloc[:, 0].values

# Load slide (정적) data
# slide_data = pd.read_csv('/home/user/release/performance_test/RT-data-3.csv')
slide_data = pd.read_csv('C:/Users/ujin3/Desktop/nrmk/CORE_automation/statics_data/stribeck2.csv')
slide_velocity  = slide_data.iloc[:, 0].values
slide_torque_measured = slide_data.iloc[:, 1].values

# Filtering 및 다운샘플링
fs = 4000  # Sampling frequency (Hz)
fc = 40    # Cutoff frequency (Hz)
w = fc / (fs / 2)
b, a = butter(4, w, 'low')
position_filtered = filtfilt(b, a, position)
dt = 0.00025
N = 8  # number of bristles

downsample_factor = 4
position_filtered = position_filtered[::downsample_factor]
time = time[::downsample_factor]
torque_measured = torque_measured[::downsample_factor]
dt = dt * downsample_factor

# Compute velocity and filter
velocity = np.diff(position_filtered) / dt
velocity_filtered = filtfilt(b, a, velocity)
velocity = velocity_filtered
# Align lengths: velocity와 측정 토크의 길이 맞춤
position = position_filtered[:-1]
torque_measured = torque_measured[:-1]

# -------------------------------
# 모델 상수 (고정값)
# -------------------------------
# 정적 마찰 파라미터들을 JSON 파일에서 불러오기
with open('friction_parameters.json', 'r') as f:
    friction_params = json.load(f)

# JSON 파일에서 파라미터 값 추출
static_params = friction_params["FricParameter"][0]
Fcp = static_params["F_c_p"]
Fsp = static_params["F_s_p"]
Vsp = static_params["F_v_p"]
delta_sp = static_params["delta_p"]
s2p = static_params["sigma_2_p"]

Fcn = static_params["F_c_n"]
Fsn = static_params["F_s_n"]
Vsn = static_params["F_v_n"]
delta_sn = static_params["delta_n"]
s2n = static_params["sigma_2_n"]

# -------------------------------
# 미리 계산 가능한 값들 (dynamic data)
# -------------------------------
T_dyn = len(velocity)
s_v_array = np.zeros(T_dyn)
sign_v = np.sign(velocity)
cond2_array = (np.abs(velocity) < 0.002)  # Boolean array

for t in range(T_dyn):
    if velocity[t] > 0:
        s_v_array[t] = Fcp + (Fsp - Fcp) * np.exp(- (velocity[t] / Vsp) ** delta_sp)
    else:
        s_v_array[t] = Fcn + (Fsn - Fcn) * np.exp(- (abs(velocity[t]) / Vsn) ** delta_sn)

# -------------------------------
# GMS 모델 함수 (동적 데이터)
# -------------------------------
def gms_model(p, velocity, s_v_array, sign_v, cond2_array, dt, N):
    """
    파라미터 p: 길이 3N+1 벡터
      - p[0:N] : k (스티프니스)
      - p[N:2N] : beta (alpha를 위한 자유변수, alpha = softmax(beta))
      - p[2N:3N] : sigma (댐핑 계수)
      - p[3N] : C
    """
    k = p[0:N]
    beta = p[N:2*N]
    sigma = p[2*N:3*N]  # 새로 추가된 sigma 파라미터
    C = p[3*N]
    exp_beta = np.exp(beta)
    alpha = exp_beta / np.sum(exp_beta)
    
    T = len(velocity)
    z = np.zeros((T, N))
    F_pred = np.zeros(T)
    
    for t in tqdm(range(T), desc="동적 모델 계산 중"):
        s_v = s_v_array[t]
        F_t = 0.0
        for i in range(N):
            Di = alpha[i] * s_v / k[i]
            if t == 0:
                dz = velocity[t]
                z[t, i] = dz * dt
            else:
                cond1 = np.abs(z[t-1, i]) < (alpha[i] * s_v / k[i])
                if cond1 and cond2_array[t]:
                    dz = velocity[t]
                else:
                    dz = (alpha[i] * C)/k[i]*(np.sign(velocity[t]) - z[t-1, i] / Di)
                z[t, i] = z[t-1, i] + dz * dt
            F_t += k[i] * z[t, i] + sigma[i]*dz
        if velocity[t] > 0:
            F_t += s2p * velocity[t]
        else:
            F_t += s2n * velocity[t]
        F_pred[t] = F_t
    return F_pred

def residual_dyn(p, velocity, s_v_array, sign_v, cond2_array, dt, N, torque_measured):
    F_pred = gms_model(p, velocity, s_v_array, sign_v, cond2_array, dt, N)
    return F_pred - torque_measured

# -------------------------------
# 동적 데이터에 대한 최적화 (전체 파라미터)
# -------------------------------
# 파라미터 초기 추정치 설정
# p = [k (N개), beta (N개), sigma (N개), C (1개)]
p0_dyn = np.concatenate([
    np.array([4.5959, 0.9490, 0.2289, 0.1332, 0.0341, 0.0095, 0.0108, 0.0172]) * 1e5,  # 초기 k
    -np.log(N) * np.ones(N),        # beta 초기값
    0.001 * np.ones(N),             # 초기 sigma 값 추가
    np.array([300.0])               # C 초기값
])

# least_squares 최적화 수행 (제곱 오차 최소화)
res_dyn = least_squares(
    residual_dyn,
    p0_dyn,
    args=(velocity, s_v_array, sign_v, cond2_array, dt, N, torque_measured),
    bounds=(np.concatenate([0.001*np.ones(N), -np.inf*np.ones(N), np.zeros(N), [0.0]]),
            np.concatenate([np.inf*np.ones(N), np.inf*np.ones(N), np.inf*np.ones(N), [1e6]])),
    xtol=1e-6, ftol=1e-6, verbose=2
)

p_opt = res_dyn.x
k_opt = p_opt[0:N]
beta_opt = p_opt[N:2*N]
sigma_opt = p_opt[2*N:3*N]  # 최적화된 sigma 값
C_opt = p_opt[3*N]
exp_beta = np.exp(beta_opt)
alpha_opt = exp_beta / np.sum(exp_beta)

print("Dynamic data 최적화 완료!")
print("최적의 k:", k_opt)
print("최적의 alpha:", alpha_opt)
print("최적의 sigma:", sigma_opt)
print("최적의 C:", C_opt)

# -------------------------------
# Slide data (정적) 사전 계산
# -------------------------------
T_slide = len(slide_velocity)
s_v_array_slide = np.zeros(T_slide)
sign_v_slide = np.sign(slide_velocity)
cond2_array_slide = (np.abs(slide_velocity) < 0.004)

for t in range(T_slide):
    if slide_velocity[t] > 0:
        s_v_array_slide[t] = Fcp + (Fsp - Fcp) * np.exp(- (slide_velocity[t] / Vsp) ** delta_sp)
    else:
        s_v_array_slide[t] = Fcn + (Fsn - Fcn) * np.exp(- (abs(slide_velocity[t]) / Vsn) ** delta_sn)

# -------------------------------
# GMS 모델 함수 (Slide data, k와 alpha는 고정)
# -------------------------------
def gms_model_slide(C, fixed_k, fixed_alpha,fixed_sigma, velocity, s_v_array, sign_v, cond2_array, dt, N):
    T = len(velocity)
    z = np.zeros((T, N))
    F_pred = np.zeros(T)
    
    for t in tqdm(range(T), desc="슬라이드 모델 계산 중"):
        s_v = s_v_array[t]
        F_t = 0.0
        for i in range(N):
            Di = fixed_alpha[i] * s_v / fixed_k[i]
            if t == 0:
                dz = velocity[t]
                z[t, i] = dz * dt
            else:
                cond1 = np.abs(z[t-1, i]) < (fixed_alpha[i] * s_v / fixed_k[i])
                if cond1 and cond2_array[t]:
                    dz = velocity[t]
                else:
                    dz = (fixed_alpha[i] * C)/fixed_k[i]*(np.sign(velocity[t]) - z[t-1, i] / Di)
                z[t, i] = z[t-1, i] + dz * dt
            F_t += fixed_k[i] * z[t, i] + fixed_sigma[i] * dz
        if velocity[t] > 0:
            F_t += s2p * velocity[t]
        else:
            F_t += s2n * velocity[t]
        F_pred[t] = F_t
    return F_pred

def residual_slide(C, fixed_k, fixed_alpha,fixed_sigma, velocity, s_v_array, sign_v, cond2_array, dt, N, slide_torque_measured):
    F_pred = gms_model_slide(C, fixed_k, fixed_alpha,fixed_sigma, velocity, s_v_array, sign_v, cond2_array, dt, N)
    return F_pred - slide_torque_measured

# -------------------------------
# Slide data에 대한 최적화 (C만 최적화)
# -------------------------------
# 동적 데이터 최적화 결과에서 얻은 k_opt와 alpha_opt를 사용하여, slide data에 대해 C만 최적화
C0 = np.array([C_opt])  # 초기값은 동적 데이터 최적화 결과 사용

res_slide = least_squares(
    residual_slide,
    C0,
    args=(k_opt, alpha_opt,sigma_opt, slide_velocity, s_v_array_slide, sign_v_slide, cond2_array_slide, dt, N, slide_torque_measured),
    bounds=([0.0], [1e5]),
    xtol=1e-4, ftol=1e-4, verbose=2
)

C_opt_slide = res_slide.x[0]
print("Slide data 최적화 완료!")
print("최종 최적의 C 값:", C_opt_slide)

# -------------------------------
# 결과 시각화
# -------------------------------
# Dynamic data 결과 그래프
F_predicted_dyn_opt = gms_model(p_opt, velocity, s_v_array, sign_v, cond2_array, dt, N)
ss_res = np.sum((torque_measured - F_predicted_dyn_opt)**2)
ss_tot = np.sum((torque_measured - np.mean(torque_measured))**2)
r_squared = (1 - ss_res/ss_tot) * 100
rmse_val = np.sqrt(np.mean((torque_measured - F_predicted_dyn_opt)**2))
nrmse = (rmse_val / (np.max(torque_measured) - np.min(torque_measured))) * 100
print(f"Dynamic Data R-squared: {r_squared:.2f}%")
print(f"Dynamic Data NRMSE: {nrmse:.2f}%")

plt.figure(figsize=(5, 3))
plt.plot(position, torque_measured, label='Measured Torque',marker='o',markersize=2)
plt.plot(position, F_predicted_dyn_opt, label='Predicted Torque',marker='*',markersize=2)
plt.title(f'GMS Model Prediction (Dynamic Data, R²: {r_squared:.2f}%)')
plt.xlabel('Position')
plt.ylabel('Torque (Nm)')
plt.legend()
plt.show()

# Slide data 결과 그래프
F_predicted_slide_opt = gms_model_slide(C_opt_slide, k_opt, alpha_opt, sigma_opt, slide_velocity, s_v_array_slide, sign_v_slide, cond2_array_slide, dt, N)
plt.figure(figsize=(5, 3))
plt.plot(slide_velocity, slide_torque_measured, 'b.', label='Measured', markersize=2)
plt.plot(slide_velocity, F_predicted_slide_opt, 'r*', label='Predicted', markersize=2)
plt.title('Velocity-Torque Characteristic (Slide Data)')
plt.xlabel('Velocity (rad/s)')
plt.ylabel('Torque (Nm)')
plt.legend()
plt.grid(True)
plt.show()

# 최적화된 파라미터들을 JSON 파일로 저장
with open('friction_parameters.json', 'r') as f:
    existing_params = json.load(f)

# 기존 파라미터에 새로운 GMS 파라미터 추가
existing_params["FricParameter"][0].update({
    "k": k_opt.tolist(),
    "alpha": alpha_opt.tolist(),
    "sigma": sigma_opt.tolist(),
    "C": float(C_opt_slide)
})

# JSON 파일로 저장
with open('friction_parameters.json', 'w') as f:
    json.dump(existing_params, f, indent=4)

print("최적화된 파라미터들이 JSON 파일에 추가되었습니다.")
