import numpy as np
import matplotlib.pyplot as plt
from scipy.fft import fft, ifft, fftfreq
import time

# 샘플링 주파수 및 버퍼 크기 설정
fs = 50  # 샘플링 주파수 (Hz)
buffer_size = 10  # 버퍼 크기
duration = 20  # 시뮬레이션 시간 (초)
n_samples = int(fs * duration)

# 예제 데이터 생성: 실제 속도 센서를 시뮬레이션
np.random.seed(0)
time_vector = np.arange(0, duration, 1/fs)
speed_data = 10 + 2 * np.sin(2 * np.pi * 0.1 * time_vector) + np.random.randn(len(time_vector))  # 예제 속도 데이터


# 실시간 처리를 위한 데이터 버퍼 생성
buffer = np.zeros(buffer_size)
output_buffer = np.zeros(n_samples)

# 필터 설정 (예: 1Hz 이하만 통과시키기)
cutoff_frequency = 1

# 실시간 처리를 시뮬레이션하는 루프
for i in range(n_samples):
    # 버퍼에 새로운 데이터 추가 (시뮬레이션)
    buffer = np.roll(buffer, -1)
    buffer[-1] = speed_data[i]

    # 버퍼가 가득 찼을 때만 처리
    if i >= buffer_size - 1:
        # 현재 버퍼에 대해 푸리에 변환 수행
        f_signal = fft(buffer)
        frequencies = fftfreq(buffer_size, 1/fs)

        # 필터 적용 (저역통과 필터)
        f_signal[np.abs(frequencies) > cutoff_frequency] = 0

        # 역 푸리에 변환을 통해 시간 도메인 신호로 변환
        filtered_signal = ifft(f_signal)

        # 필터링된 신호를 출력 버퍼에 저장
        output_buffer[i - buffer_size + 1:i + 1] = filtered_signal.real

    # 실시간 그래프 업데이트 (속도를 위해 매번 업데이트는 생략 가능)
    if i % fs == 0:
        plt.clf()
        plt.plot(time_vector[:i+1], speed_data[:i+1], label='Original Speed Data')
        plt.plot(time_vector[:i+1], output_buffer[:i+1], label='Filtered Speed Data', linestyle='--')
        plt.title('Real-Time Noise Filtering Simulation')
        plt.xlabel('Time (s)')
        plt.ylabel('Speed')
        plt.legend()
        plt.grid()
        plt.pause(0.01)

# 최종 결과 시각화
plt.figure(figsize=(12, 6))
# plt.plot(time_vector, speed_data, label='Original Speed Data')
plt.plot(time_vector, output_buffer, label='Filtered Speed Data', linestyle='--')
plt.title('Cleaned Speed Sensor Data (Time Domain)')
plt.xlabel('Time (s)')
plt.ylabel('Speed')
plt.legend()
plt.grid()
plt.show()
