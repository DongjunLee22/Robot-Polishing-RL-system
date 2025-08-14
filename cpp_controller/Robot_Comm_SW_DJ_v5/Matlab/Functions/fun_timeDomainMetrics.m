function [Mp, tr, ts, ess] = fun_timeDomainMetrics(t, y_norm, y_target)
    % y_norm      : normalized response (0 → |y_target|)
    % y_target    : normalized 목표값 (크기만 사용, 부호는 dir로 처리)
    
    dir           = sign(y_target);
    y_abs_target  = abs(y_target);

    % 1) Rise Time: 10% → 90%
    t10 = t(find(dir*y_norm >= 0.1*y_abs_target, 1, 'first'));
    t90 = t(find(dir*y_norm >= 0.9*y_abs_target, 1, 'first'));
    tr = t90 - t10;

    % 2) Overshoot
    y_max = max(dir*y_norm);
    Mp    = (y_max - y_abs_target) / y_abs_target * 100;

    % 3) Settling Time: ±2% band around target
    tol = 0.02 * y_abs_target;
    % indices where response is outside the ±2% band
    idx_out = find(abs(dir*y_norm - y_abs_target) > tol);
    if isempty(idx_out)
        ts = 0;
    else
        last_out = idx_out(end);
        if last_out < numel(t)
            ts = t(last_out + 1);
        else
            ts = t(end);
        end
    end

    % 4) Steady-state error
    ess = (dir*y_norm(end) - y_abs_target);

end
