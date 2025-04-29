import casadi as ca
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
preslide_data_path = r'C:\Users\chunggeun.kim\Documents\NRMK\1. Project\GMS 마찰보상\friction_data\RT_test\RT-data-4.csv'
slide_data_path = r'C:\Users\chunggeun.kim\Documents\NRMK\1. Project\GMS 마찰보상\friction_data\RT_test\RT-data-4.csv'
output_path = r'C:\Users\chunggeun.kim\Documents\NRMK\1. Project\GMS 마찰보상\friction_Id\friction_parameters.json'

preslide_data = pd.read_csv(preslide_data_path, skiprows=1)
position = preslide_data.iloc[:, 1].values
torque_measured = preslide_data.iloc[:, 6].values
time = preslide_data.iloc[:, 0].values

# Load slide (정적) data
slide_data = pd.read_csv(slide_data_path)
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
with open(output_path, 'r') as f:
    friction_params = json.load(f)

# JSON 파일에서 파라미터 값 추출
static_params = friction_params["FricParameter"][0]
Fcp = static_params["F_c_p"]
Fv1p = static_params["F_v1_p"]
Fv2p = static_params["F_v2_p"]

Fcn = static_params["F_c_n"]
Fv1n = static_params["F_v1_n"]
Fv2n = static_params["F_v2_n"]

k_static = static_params["k_static"]

def sigmoid_functions(x, function_type):
    """
    다양한 시그모이드 함수들을 구현
    
    Parameters:
    x: 입력값 (scalar 또는 numpy array)
    function_type: 함수 종류 (1-6)
    
    Returns:
    시그모이드 함수 결과값
    """
    
    if function_type == 1:
        return -0.5 * np.tanh(x) + 0.5
    
    elif function_type == 2:
        return -np.arctan(x) / np.pi + 0.5
    
    elif function_type == 3:
        return -0.5 * x / np.sqrt(1 + x**2) + 0.5
    
    elif function_type == 4:
        return -0.5 * x / (1 + np.abs(x)) + 0.5
    
    elif function_type == 5:
        return 1 / (1 + np.exp(x))
    
    elif function_type == 6:
        return -np.arctan(np.sinh(x)) / np.pi + 0.5
    
    else:
        raise ValueError("함수 타입은 1-6 사이의 값이어야 합니다.")

# -------------------------------
# 미리 계산 가능한 값들 (dynamic data)
# -------------------------------
T_dyn = len(velocity)
s_v_array = np.zeros(T_dyn)
cond2_array = (np.abs(velocity) < 0.0041)  # Boolean array

for t in range(T_dyn):
    m_v = (2.0/ca.pi) * ca.arctan(k_static*velocity[t])
    Fc = ca.if_else(velocity[t] >= 0, Fcp, Fcn)
    s_v_array[t] = Fc * m_v
    
# -------------------------------
# GMS 모델 함수 (동적 데이터)
# -------------------------------
def gms_model(p, velocity, s_v_array, cond2_array, dt, N):
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
    C = 3000  # C는 고정된 값으로 사용
    exp_beta = np.exp(beta)
    alpha = exp_beta / np.sum(exp_beta)
    
    T = len(velocity)
    z = np.zeros((T, N))
    F_pred = np.zeros(T)
    global gamma1
    global gamma2
    gamma1 = 1846
    gamma2 = 14

    for t in tqdm(range(T), desc="동적 모델 계산 중"):
        s_v = s_v_array[t]
        F_t = 0.0
        for i in range(N):
            Di = alpha[i] * s_v / k[i]
            x1i = gamma1*(np.abs(z[t-1,i])-Di)/Di
            x2i = gamma2*(np.abs(velocity[t])-0.004)/0.004
            wai = sigmoid_functions(x1i,1)*sigmoid_functions(x2i,1)
            wbi=1-wai
            if t == 0:
                dz = velocity[t]
                z[t, i] = dz * dt
            else:
                cond1 = np.abs(z[t-1, i]) < (alpha[i] * s_v / k[i])
                if cond1 and cond2_array[t]:
                    dz = velocity[t]
                else:
                    dz = velocity[t]*wai+wbi*(alpha[i] * C)/k[i]*(np.sign(velocity[t]) - z[t-1, i] / Di)
                    # dz = (alpha[i] * C)/k[i]*(np.sign(velocity[t]) - z[t-1, i] / Di)

                z[t, i] = z[t-1, i] + dz * dt
            F_t += k[i] * z[t, i] + sigma[i]*dz
        
        F_t += ca.if_else(
            velocity[t] >= 0, 
            Fv1p*(1-ca.exp(-ca.fabs(velocity[t]/Fv2p)))*ca.sign(velocity[t]),
            Fv1n*(1-ca.exp(-ca.fabs(velocity[t]/Fv2n)))*ca.sign(velocity[t])
        )
        F_pred[t] = F_t
    return F_pred

def residual_dyn(p, velocity, s_v_array, cond2_array, dt, N, torque_measured):
    F_pred = gms_model(p, velocity, s_v_array, cond2_array, dt, N)
    return F_pred - torque_measured

# -------------------------------
# 동적 데이터에 대한 최적화 (전체 파라미터)
# -------------------------------
# 파라미터 초기 추정치 설정
# p = [k (N개), beta (N개), sigma (N개), C (1개)]
p0_dyn = np.concatenate([
    np.array([4.5959, 0.9490, 0.2289, 0.1332, 0.0341, 0.0095, 0.0108, 0.0172]) * 1e5,  # 초기 k
    1/N * np.ones(N),        # beta 초기값
    0.01 * np.ones(N),       # 초기 sigma 값 추가             
])

# least_squares 최적화 수행 (제곱 오차 최소화)
res_dyn = least_squares(
    residual_dyn,
    p0_dyn,
    args=(velocity, s_v_array, cond2_array, dt, N, torque_measured),
    bounds=(np.concatenate([0.001*np.ones(N), -np.inf*np.ones(N), np.zeros(N)]),
            np.concatenate([np.inf*np.ones(N), np.inf*np.ones(N), np.inf*np.ones(N)])),
    xtol=1e-6, ftol=1e-6, verbose=2
)

p_opt = res_dyn.x
k_opt = p_opt[0:N]
beta_opt = p_opt[N:2*N]
sigma_opt = p_opt[2*N:3*N]  # 최적화된 sigma 값
exp_beta = np.exp(beta_opt)
alpha_opt = exp_beta / np.sum(exp_beta)

print("Dynamic data 최적화 완료!")
print("최적의 k:", k_opt)
print("최적의 alpha:", alpha_opt)
print("최적의 sigma:", sigma_opt)


# -------------------------------
# Slide data (정적) 사전 계산
# -------------------------------
T_slide = len(slide_velocity)
s_v_array_slide = np.zeros(T_slide)
sign_v_slide = np.sign(slide_velocity)
cond2_array_slide = (np.abs(slide_velocity) < 0.004)

for t in range(T_slide):
    m_v = (2.0/ca.pi) * ca.arctan(k_static*slide_velocity[t])
    Fc = ca.if_else(slide_velocity[t] >= 0, Fcp, Fcn)
    s_v_array_slide[t] = Fc * m_v

# -------------------------------
# GMS 모델 함수 (Slide data, k와 alpha는 고정)
# -------------------------------
def gms_model_slide(C, fixed_k, fixed_alpha, fixed_sigma, velocity, s_v_array, cond2_array, dt, N):
    T = len(velocity)
    z = np.zeros((T, N))
    F_pred = np.zeros(T)
    gamma1 = 1846
    gamma2 = 14
    
    for t in tqdm(range(T), desc="슬라이드 모델 계산 중"):
        s_v = s_v_array[t]
        F_t = 0.0
        for i in range(N):
            Di = fixed_alpha[i] * s_v / fixed_k[i]
            x1i = gamma1*(np.abs(z[t-1,i])-Di)/Di
            x2i = gamma2*(np.abs(velocity[t])-0.004)/0.004
            wai = sigmoid_functions(x1i,1)*sigmoid_functions(x2i,1)
            wbi=1-wai
            if t == 0:
                dz = velocity[t]
                z[t, i] = dz * dt
            else:
                cond1 = np.abs(z[t-1, i]) < (fixed_alpha[i] * s_v / fixed_k[i])
                if cond1 and cond2_array[t]:
                    dz = velocity[t]
                else:
                    # dz = (fixed_alpha[i] * C)/fixed_k[i]*(np.sign(velocity[t]) - z[t-1, i] / Di)
                    dz = velocity[t]*wai+wbi*(fixed_alpha[i] * C)/fixed_k[i]*(np.sign(velocity[t]) - z[t-1, i] / Di)
                z[t, i] = z[t-1, i] + dz * dt
            F_t += fixed_k[i] * z[t, i] + fixed_sigma[i] * dz
        F_t += ca.if_else(
            velocity[t] >= 0, 
            Fv1p*(1-ca.exp(-ca.fabs(velocity[t]/Fv2p)))*ca.sign(velocity[t]),
            Fv1n*(1-ca.exp(-ca.fabs(velocity[t]/Fv2n)))*ca.sign(velocity[t])
        )
        F_pred[t] = F_t
    return F_pred

def residual_slide(C, fixed_k, fixed_alpha,fixed_sigma, velocity, s_v_array, cond2_array, dt, N, slide_torque_measured):
    F_pred = gms_model_slide(C, fixed_k, fixed_alpha,fixed_sigma, velocity, s_v_array, cond2_array, dt, N)
    return F_pred - slide_torque_measured

# -------------------------------
# Slide data에 대한 최적화 (C만 최적화)
# -------------------------------
# 동적 데이터 최적화 결과에서 얻은 k_opt와 alpha_opt를 사용하여, slide data에 대해 C만 최적화
C_opt = 3000
C0 = np.array([C_opt])  # 초기값은 동적 데이터 최적화 결과 사용

res_slide = least_squares(
    residual_slide,
    C0,
    args=(k_opt, alpha_opt,sigma_opt, slide_velocity, s_v_array_slide, cond2_array_slide, dt, N, slide_torque_measured),
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
F_predicted_dyn_opt = gms_model(p_opt, velocity, s_v_array, cond2_array, dt, N)
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
F_predicted_slide_opt = gms_model_slide(C_opt_slide, k_opt, alpha_opt, sigma_opt, slide_velocity, s_v_array_slide, cond2_array_slide, dt, N)
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
with open(output_path, 'r') as f:
    existing_params = json.load(f)

# 기존 파라미터에 새로운 GMS 파라미터 추가
existing_params["FricParameter"][0].update({
    "k": k_opt.tolist(),
    "alpha": alpha_opt.tolist(),
    "sigma": sigma_opt.tolist(),
    "C": float(C_opt_slide),
    "gamma1": gamma1,
    "gamma2": gamma2
})

# JSON 파일로 저장
with open(output_path, 'w') as f:
    json.dump(existing_params, f, indent=4)

print("최적화된 파라미터들이 JSON 파일에 추가되었습니다.")
