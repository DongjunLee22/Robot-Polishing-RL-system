function [f, Pxx] = fun_computeFFTAnalysis(t, y)
%COMPUTEFFTANALYSIS  단일-측 스펙트럼 계산
%   [f, Pxx] = computeFFTAnalysis(t, y)
%   t: 시간 벡터
%   y: 신호(예: 접촉력 응답)

    % 샘플링 주파수
    Fs = 1/mean(diff(t));
    L  = length(y);

    % 평균 제거 후 FFT
    Y  = fft(y - mean(y));
    P2 = abs(Y/L);
    P1 = P2(1:floor(L/2)+1);
    P1(2:end-1) = 2*P1(2:end-1);

    % 주파수 벡터
    f  = Fs*(0:floor(L/2)) / L;
    Pxx = P1;
end
