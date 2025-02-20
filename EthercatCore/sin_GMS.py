import json
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
def loadparameter():
    try:
        with open('C:/Users/ujin3/Desktop/nrmk/EthercatCore/friction_parameters1.json', 'r') as f:
            params = json.load(f)
            
        # Get first parameter set
        fric_params = params['FricParameter'][0]
        
        # GMS 파라미터 로드 
        F_c_p = fric_params['F_c_p']
        F_s_p = fric_params['F_s_p']
        F_v_p = fric_params['F_v_p'] 
        F_c_n = fric_params['F_c_n']
        F_s_n = fric_params['F_s_n']
        F_v_n = fric_params['F_v_n']
        sigma_2_p = fric_params['sigma_2_p']
        sigma_2_n = fric_params['sigma_2_n']
        C = fric_params['C']
        gamma1 = fric_params['gamma1']
        gamma2 = fric_params['gamma2']
        
        # 배열 파라미터 로드
        alpha = fric_params['alpha']
        k = fric_params['k']
        sigma = fric_params['sigma']
         
        return F_c_p, F_s_p, F_v_p, F_c_n, F_s_n, F_v_n, sigma_2_p, sigma_2_n, C, gamma1, gamma2, alpha, k, sigma

    except Exception as e:
        print(f"파일 로드 중 에러 발생: {e}")
        return False
    
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

    
def gms_model(C, k,alpha, sigma, velocity, s_v_array,  cond2_array, dt, N, sigma_2_p, sigma_2_n, gamma1, gamma2):
    T = len(velocity)
    z = np.zeros((T, N))
    F_pred = np.zeros(T)
    
    for t in range(T):
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
                z[t, i] = z[t-1, i] + dz * dt
            F_t += k[i] * z[t, i] + sigma[i] * dz
        if velocity[t] > 0:
            F_t += sigma_2_p * velocity[t]
        else:
            F_t += sigma_2_n * velocity[t]
        F_pred[t] = F_t
    return F_pred

if __name__ == '__main__':
    F_c_p, F_s_p, F_v_p,  F_c_n, F_s_n, F_v_n, sigma_2_p, sigma_2_n, C, gamma1, gamma2, alpha, k, sigma = loadparameter()
    N = 8

    velocity = 0.5*np.sin(np.linspace(0, 8*np.pi, 1000))
    T_dyn = len(velocity)
    s_v_array = np.zeros(T_dyn)
    cond2_array = (np.abs(velocity) < 0.001)
    dt = 0.00025
    # velocity 배열 생성 수정
    velocity_p = np.linspace(0, 0.001, 1000)
    velocity_n = np.linspace(-0.001, 0, 1000)[:-1]  # 중복 방지를 위해 마지막 요소 제외
    # velocity = np.concatenate([velocity_n, velocity_p])

    position = np.zeros(len(velocity))
    for i in range(1, len(velocity)):
        position[i] = position[i-1] + velocity[i] * dt

    
    # s_v_array 계산 수정
    for t in range(T_dyn):
        if velocity[t] >= 0:
            s_v_array[t] = F_c_p + (F_s_p - F_c_p) * np.exp(-(velocity[t] / F_v_p) ** 2)
        else:
            s_v_array[t] = F_c_n + (F_s_n - F_c_n) * np.exp(-(abs(velocity[t]) / F_v_n) ** 2)

    F_pred = gms_model(C, k, alpha, sigma, velocity, s_v_array, cond2_array, dt, N,
                      sigma_2_p, sigma_2_n, gamma1, gamma2)
    csv_path = "C:/Users/ujin3/Desktop/nrmk/CORE_automation/statics_data/stribeck2.csv"
    data = pd.read_csv(csv_path, header=None)
    preslide_data = pd.read_csv('C:/Users/ujin3/Desktop/nrmk/CORE_automation/dynamic_data/dynamic2.csv', skiprows=1)
    position_preslide = preslide_data.iloc[:, 1].values
    torque_measured = preslide_data.iloc[:, 6].values

    v_data = data.iloc[:, 0].values   # 속도 데이터
    t_data = data.iloc[:, 1].values   # 토크 데이터    
    # 속도-토크 그래프 (상단)
    # plt.scatter(v_data, t_data, c='blue', alpha=0.5, label='측정 데이터')
    plt.plot(velocity, F_pred)
    plt.xlabel('Velocity')
    plt.ylabel('Force')
    plt.grid(True)
    plt.show()