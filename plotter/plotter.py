import pandas as pd
import matplotlib.pyplot as plt
from datetime import datetime
import numpy as np
import glob
import os

# 현재 디렉토리에 있는 모든 .txt 파일을 찾음
txt_files = glob.glob('*.txt')

for file_path in txt_files:
    # 데이터 로드
    df = pd.read_csv(file_path, delimiter=',', header=None, names=['Time', 'Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6', 'Joint7'])
    df['Time'] = pd.to_datetime(df['Time'], format='%M:%S.%f')
    df['TimeInSeconds'] = df['Time'].apply(lambda x: (x - datetime(1900, 1, 1)).total_seconds())
    df['TimeInSeconds'] -= df['TimeInSeconds'].iloc[0]

    # NumPy 배열로 변환
    time_in_seconds = df['TimeInSeconds'].values
    values = df.iloc[:, 1:].values

    # 그래프 크기 설정
    plt.figure(figsize=(15, 10))

    # 시간 구간 정의
    time_intervals = [(0, 10*15), (10*15, 10*15+5*15), (10*15+5*15, df['TimeInSeconds'].iloc[-1])]

    # 각 조인트에 대한 subplot 및 최대-최소값 직선으로 표시
    for i in range(values.shape[1]-1):
        plt.subplot(4, 2, i + 1)
        plt.plot(time_in_seconds, values[:, i], label=f'Joint{i+1}')

        # 각 시간 구간에 대해 최대-최소값 찾기 및 직선으로 표시
        for start, end in time_intervals:
            # 해당 구간의 데이터
            mask = (time_in_seconds >= start) & (time_in_seconds <= end)
            segment = values[mask, i]
            time_segment = time_in_seconds[mask]

            if len(segment) == 0:  # 구간에 데이터가 없으면 스킵
                continue

            max_val = np.max(segment)
            min_val = np.min(segment)

            # 최대값과 최소값 직선으로 표시
            plt.hlines(max_val, xmin=start, xmax=end, colors='red')
            plt.hlines(min_val, xmin=start, xmax=end, colors='blue')

            # 최대값과 최소값 텍스트로 표시
            plt.text(end, max_val, f'{max_val:.2f}', ha='right', va='bottom', color='red')
            plt.text(end, min_val, f'{min_val:.2f}', ha='right', va='top', color='blue')

        plt.title(f'Joint{i+1}')
        plt.legend()

    # 마지막 플롯에는 모든 조인트 값을 표시하지만 최대/최소 직선은 그리지 않음
    plt.subplot(4, 2, 8)
    for i in range(values.shape[1]-1):
        plt.plot(time_in_seconds, values[:, i], label=f'Joint{i+1}')
    plt.title('All Values')

    plt.tight_layout()

    # 파일 이름 설정 및 저장
    output_filename = os.path.splitext(file_path)[0] + ".png"
    plt.savefig(output_filename)
    plt.close()  # 현재 그림 닫기
    print(f"Plot saved as {output_filename}")
