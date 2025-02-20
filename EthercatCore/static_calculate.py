import casadi as ca
import numpy as np
import pandas as pd
import json

# ---------------------------
# 1. 데이터 로드 (CSV 파일 읽기)
# ---------------------------
# 예제에서는 CSV 파일에 첫 번째 열: velocity, 두 번째 열: torque가 있다고 가정합니다.
csv_path = "C:/Users/ujin3/Desktop/nrmk/CORE_automation/statics_data/stribeck2.csv"
data = pd.read_csv(csv_path, header=None)
v_data = data.iloc[:, 0].values   # 속도 데이터
t_data = data.iloc[:, 1].values   # 토크 데이터

n_data = len(v_data)

# ---------------------------
# 2. CasADi 최적화 문제 설정
# ---------------------------
opti = ca.Opti()

# 파라미터 벡터 (크기 10): 
# [Fc+ , Fs+ , Vs+  , s2+ , Fc- , Fs- , Vs- , s2-]
p = opti.variable(8)

# Stribeck 마찰 모델 정의 (v에 따라 조건부로 분기)
def stribeck_friction(v, p):
    # v >= 0 인 경우
    expr_pos = p[0] + (p[1] - p[0]) * ca.exp(-((ca.fabs(v) / p[2]) ** 1)) + p[3] * v
    # v < 0 인 경우
    expr_neg = -p[4] - (p[5] - p[4]) * ca.exp(-((ca.fabs(v) / p[6]) ** 1)) + p[7] * v
    return ca.if_else(v >= 0, expr_pos, expr_neg)

# 목적 함수: 모든 데이터 포인트에 대해 예측 토크와 측정 토크의 제곱 오차의 합
obj = 0
for i in range(n_data):
    pred = stribeck_friction(v_data[i], p)
    obj += (pred - t_data[i])**2

opti.minimize(obj)

# ---------------------------
# 3. 제약 조건 및 초기값 설정
# ---------------------------
# 모든 파라미터는 음수가 되지 않도록 하한을 0으로 설정합니다.
for i in range(8):
    opti.subject_to(p[i] >= 0)

# Vs 파라미터 (양쪽) 상한을 10으로 제한 (인덱스 2와 7)
opti.subject_to(p[2] <= 10)
opti.subject_to(p[6] <= 10)

# 초기 파라미터 설정 (C++ 코드의 초기값 사용)
p0 = [11,12, 0.000200, 20.797302,
      11,12, 0.000200, 20.797302]
opti.set_initial(p, p0)

# ---------------------------
# 4. 솔버 설정 및 최적화 실행
# ---------------------------
# IPOPT 솔버 사용, 지원되는 옵션으로 변경
opti.solver("ipopt", {
    "print_time": False,
    "ipopt": {
        "print_level": 0,
        "tol": 1e-6
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
            "F_c_n": p_opt[4],  # Fc-
            "F_c_p": p_opt[0],   # Fc+
            "F_s_n": p_opt[5], # Fs-
            "F_s_p": p_opt[1],  # Fs+
            "F_v_n": p_opt[6],  # s2-
            "F_v_p": p_opt[2],  # s2+
            "sigma_2_n": p_opt[7],
            "sigma_2_p": p_opt[3]
        }
    ]
}

# JSON 파일로 저장
output_path = "friction_parameters.json"
with open(output_path, 'w') as f:
    json.dump(friction_data, f, indent=2)

print(f"\nJSON 파일이 {output_path}에 저장되었습니다.")

# ---------------------------
# 5. 결과 시각화
# ---------------------------
import matplotlib.pyplot as plt
import numpy as np

# 실제 데이터 포인트 플롯
plt.figure(figsize=(12, 8))

# 속도-토크 그래프 (상단)
plt.scatter(v_data, t_data, c='blue', alpha=0.5, label='측정 데이터')

# 최적화된 모델의 예측값 계산을 위한 속도 범위
v_range = np.linspace(min(v_data), max(v_data), 1000)
t_pred = [sol.value(stribeck_friction(v, p)) for v in v_range]

# 예측 곡선 플롯
plt.plot(v_range, t_pred, 'r-', label='최적화된 모델')
plt.xlabel('속도 (rad/s)')
plt.ylabel('토크 (Nm)')
plt.title('속도-토크 관계')
plt.grid(True)
plt.legend()
plt.show()



