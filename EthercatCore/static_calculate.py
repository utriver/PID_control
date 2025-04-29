import casadi as ca
import numpy as np
import pandas as pd
import json

# ---------------------------
# 1. 데이터 로드 (CSV 파일 읽기)
# ---------------------------
# 예제에서는 CSV 파일에 첫 번째 열: velocity, 두 번째 열: torque가 있다고 가정합니다.
csv_path = r'C:\Users\chunggeun.kim\Documents\NRMK\1. Project\GMS 마찰보상\friction_data\RT_test\RT-data-3.csv'
data = pd.read_csv(csv_path, header=None)
v_data = data.iloc[:, 0].values   # 속도 데이터
t_data = data.iloc[:, 1].values   # 토크 데이터

# v_data와 t_data에서 v_data 또는 t_data가 0인 값 제외
mask = (v_data != 0) & (t_data != 0)
v_data = v_data[mask]
t_data = t_data[mask]

n_data = len(v_data)

# ---------------------------
# 2. CasADi 최적화 문제 설정
# ---------------------------
opti = ca.Opti()

# 파라미터 벡터: [Fc+, Fc-, Fv1+, Fv1-, Fv2+, Fv2-]
p = opti.variable(6)  # 6개의 파라미터

# 새로운 마찰 모델 정의
def friction_model(v, p, k=10000000.0):
    m_v = (2.0/ca.pi) * ca.arctan(k*v)
    Fc = ca.if_else(v >= 0, p[0], p[1])
    friction = Fc * m_v
    viscous_friction = ca.if_else(v >= 0, p[2]*(1-ca.exp(-ca.fabs(v/p[4]))), -p[3]*(1-ca.exp(-ca.fabs(v/-p[5]))))
    
    return friction + viscous_friction

# 목적 함수: 모든 데이터 포인트에 대해 예측 토크와 측정 토크의 제곱 오차의 합
obj = 0
for i in range(n_data):
    pred = friction_model(v_data[i], p)
    obj += (pred - t_data[i])**2

opti.minimize(obj)

# ---------------------------
# 3. 제약 조건 및 초기값 설정
# ---------------------------
# 모든 파라미터는 음수가 되지 않도록 하한을 0으로 설정
for i in range(6):
    opti.subject_to(p[i] >= 0)

# 초기 파라미터 설정 [Fc+, Fc-, Fv1+, Fv1-, Fv2+, Fv2-]
p0 = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]  # 초기값 설정
opti.set_initial(p, p0)

# ---------------------------
# 4. 솔버 설정 및 최적화 실행
# ---------------------------
# IPOPT 솔버 사용, 지원되는 옵션으로 변경
opti.solver("ipopt", {
    "print_time": False,
    "ipopt": {
        "print_level": 0,
        "tol": 1e-8
    }
})
sol = opti.solve()

# 최적화된 파라미터 추출
p_opt = sol.value(p)

# JSON 파일로 저장하기 위한 데이터 구조 생성
friction_data = {
    "FricParameter": [
        {
            "index": 0,
            "F_c_p": p_opt[0],      # Fc+
            "F_c_n": p_opt[1],      # Fc-
            "F_v1_p": p_opt[2],     # Fv1+
            "F_v1_n": p_opt[3],     # Fv1-
            "F_v2_p": p_opt[4],     # Fv2+
            "F_v2_n": p_opt[5]      # Fv2-
        }
    ]
}

# JSON 파일로 저장
output_path = r'C:\Users\chunggeun.kim\Documents\NRMK\1. Project\GMS 마찰보상\friction_Id\friction_parameters.json'
with open(output_path, 'w') as f:
    json.dump(friction_data, f, indent=4)

print(f"\nJSON 파일이 {output_path}에 저장되었습니다.")

# ---------------------------
# 5. 결과 시각화
# ---------------------------
import matplotlib.pyplot as plt
import numpy as np

# 실제 데이터 포인트 플롯
plt.figure(figsize=(12, 8))

# 속도-토크 그래프 (상단)
plt.scatter(v_data, t_data, c='blue', alpha=0.5, label='Recorded Data')

# 최적화된 모델의 예측값 계산을 위한 속도 범위
v_range = np.linspace(min(v_data), max(v_data), 1000)
t_pred = [sol.value(friction_model(v, p)) for v in v_range]

# 예측 곡선 플롯
plt.plot(v_range, t_pred, 'r-', label='Optimized Model', linewidth=2)
plt.xlabel('Velocity (rad/s)')
plt.ylabel('Torque (Nm)')
plt.title('Velocity-Torque Relationship')
plt.grid(True)
plt.legend()
plt.show()