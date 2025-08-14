function ise = fun_computeISE(t, e)
%COMPUTEISE  ISE (Integral of Squared Error) 계산
%   ise = computeISE(t, e)

    ise = trapz(t, e.^2);
end
