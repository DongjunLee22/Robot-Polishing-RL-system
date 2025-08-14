function iae = fun_computeIAE(t, e)
%COMPUTEIAE  IAE (Integral of Absolute Error) 계산
%   iae = computeIAE(t, e)
%   t: 시간 벡터
%   e: 오차 e(t) = y_target - y_measured

    iae = trapz(t, abs(e));
end
