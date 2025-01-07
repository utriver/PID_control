import pandas as pd
import matplotlib.pyplot as plt

# CSV 파일 읽기
data = pd.read_csv('SampleLoggingFile.csv')

# 데이터 확인
print(data.head())

# 시간에 따른 q와 qdes 플롯
plt.figure(figsize=(12, 8))
plt.plot(data['Time'], data['Actual Position'], label='Actual Position (q)', linewidth=2)
plt.plot(data['Time'], data['Desired Position'], label='Desired Position (qdes)', linestyle='--', linewidth=2)
plt.xlabel('Time (s)')
plt.ylabel('Position')
plt.title('Actual vs Desired Position Over Time')
plt.legend()
plt.grid(True)
plt.show()
