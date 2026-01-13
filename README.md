# PID_control
로봇 정밀 제어를 위한 마찰 모델 식별 및 보상 시스템

## 📝 프로젝트 개요 (Project Description)

로봇의 정밀한 움직임을 구현하기 위해서는 관절 내 마찰 토크($\tau_f$)를 정확히 파악하는 것이 필수적입니다. 본 프로젝트에서는 로봇 동역학 모델에서 관성, 코리올리, 중력 항을 제거하여 순수 마찰력을 추출하고, 이를 기반으로 Stribeck(정적) 및 GMS(동적) 마찰 모델의 파라미터를 식별하는 시스템을 구축했습니다.

**핵심 알고리즘 및 접근법**

* Static Identification: Trapezoidal Velocity 입력을 활용한 Stribeck 모델 ($F_s, F_c, F_v, \delta, v_s$) 식별

* Dynamic Identification: Low Torque 입력을 활용한 LuGre 및 GMS(Generalized Maxwell Slip) 모델 식별

* Friction Compensation: 식별된 파라미터를 제어기에 적용하여 저속 구간의 위치 오차를 획기적으로 개선

## 🛠 기술 스택 (Tech Stack)

|분류|상세 내용|
|--------|------------------------------|
|Languages|C++ (실시간 데이터 획득), Python (비선형 최적화), Bash (자동화 셸 스크립트)|
|Algorithms|Nonlinear Optimization, Robot Dynamics Modeling, Parameter ID|
|Libraries|CasADi / SciPy (Optimization), NumPy, Matplotlib|
|Tools|Linux Shell, Git Log Management|



## 🚀 사용 방법 (Usage)
본 프로젝트는 데이터 획득부터 최적화까지 셸 스크립트 하나로 자동화되어 있습니다.

**1. 시스템 환경 설정:** 필요한 라이브러리(Python 환경 및 C 컴파일러)가 설치되어 있는지 확인합니다.

**2. 자동화 프로세스 실행:**
```
chmod +x scripts/run_id_process.sh
./scripts/run_id_process.sh
```

  * 이 스크립트는 C 파일을 빌드하여 로봇 구동 및 데이터를 수집한 뒤, 곧바로 Python 최적화 코드를 실행하여 마찰 파라미터를 산출합니다.

**3. 결과 확인**: 산출된 $F_s, F_c, F_v$ 등의 파라미터와 실제 데이터 피팅 결과를 시각적으로 확인합니다.

## 📈 연구 및 성과 (Research Results)

**1. 동역학 모델 기반 외란 제거**
마찰 토크 $\tau_f$를 구하기 위해 동역학 식에서 각 항을 다음과 같이 제거함:

* **Inertia ($M(q)$)**: 가속도가 0인 등속 구간 설정
* **Coriolis ($C$)**: 단일 관절 구동으로 상호 간섭 배제
* **Gravity ($g$)**: 동일 위치 정/역방향 구동 데이터의 평균 활용

**2. 저속 제어 정밀도 향상**
* **성능 테스트 결과**: $0.001 \sin(2\omega t) rad/s$의 극저속 구간에서 GMS 모델 적용 시 위치 오차가 거의 0에 수렴함을 확인.
* **비교 분석**: 고속 영역에서는 Static/GMS 모델의 성능이 유사하나, 속도가 낮아질수록 GMS 모델의 보상 효과가 압도적으로 높음.
