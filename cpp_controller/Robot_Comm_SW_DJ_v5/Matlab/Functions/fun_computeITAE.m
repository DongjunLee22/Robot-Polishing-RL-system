function itae = fun_computeITAE(t, e)
%COMPUTEITAE  ITAE (Integral of Time-weighted Absolute Error) 계산
%   itae = computeITAE(t, e)

    itae = trapz(t, t .* abs(e));
end
