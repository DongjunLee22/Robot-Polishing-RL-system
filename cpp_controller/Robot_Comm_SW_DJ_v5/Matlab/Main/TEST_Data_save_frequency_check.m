clear;
close all;
clc;

%% Load Data
data = readmatrix('Data_Speedl_rt_F_45.txt');
t = (data(:,1) - data(1,1)) * 0.001;    % 시간(초)
F = data(:,26:28);                      % 힘 데이터 (Fx,Fy,Fz)

%% 1) 샘플 간격 계산
dt = diff(t);

%% 2) 평균 및 분산 확인
mean_dt = mean(dt);
std_dt  = std(dt);
fprintf('평균 샘플 간격: %.6f s\n', mean_dt);
fprintf('샘플 간격 표준편차: %.6f s\n', std_dt);

%% 3) 허용오차 설정 및 판정
tol = 1e-4;  % ±0.1 ms 허용
if all(abs(dt - 0.001) < tol)
    fprintf('모든 샘플 간격이 1 kHz(0.001 s) 조건을 만족합니다.\n');
else
    idx_bad = find(abs(dt - 0.001) >= tol);
    fprintf('샘플 간격 불일치 개수: %d (첫 %d개 인덱스 표시)\n', numel(idx_bad), min(5,numel(idx_bad)));
    disp(idx_bad(1:min(5,numel(idx_bad)))');
end

%% 4) 샘플 간격 분포 시각화
figure;
histogram(dt,50);
xlabel('샘플 간격 (s)');
ylabel('빈도');
title('샘플 간격 분포');

%% 5) FFT를 이용한 지배 주파수 확인
% 분석할 신호 채널 선택 (예: Fz)
x_sig = detrend(F(:,3));         % DC 성분 제거
Fs    = 1/mean_dt;               % 평균 샘플링 주파수
N     = length(x_sig);
X     = fft(x_sig);
f     = (0:N-1)*(Fs/N);
mag   = abs(X)/N;

%% 6) 스펙트럼 플롯
figure;
plot(f(1:floor(N/2)), mag(1:floor(N/2)));
xlabel('주파수 (Hz)');
ylabel('진폭');
title('스펙트럼 분석');
xlim([0 2000]);
grid on;

%% 7) 지배 주파수 추출
[~, idx_peak] = max(mag(2:floor(N/2)));  % DC 제외
dominant_freq = f(idx_peak+1);
fprintf('지배 주파수: %.2f Hz\n', dominant_freq);
if abs(dominant_freq - 1000) < 1
    fprintf('→ 지배 주파수가 1 kHz에 가깝습니다.\n');
else
    fprintf('→ 지배 주파수가 1 kHz와 차이가 있습니다.\n');
end
