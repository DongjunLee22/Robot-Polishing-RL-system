#include "PIDController.h"

PIDController::PIDController(double kp,
    double ki,
    double kd,
    double minOutput,
    double maxOutput) noexcept
    : m_kp(kp)
    , m_ki(ki)
    , m_kd(kd)
    , m_minOutput(minOutput)
    , m_maxOutput(maxOutput)
{
}

void PIDController::setGains(double kp, double ki, double kd) noexcept {
    m_kp = kp;
    m_ki = ki;
    m_kd = kd;
}

void PIDController::setOutputLimits(double minOutput,
    double maxOutput) noexcept {
    m_minOutput = minOutput;
    m_maxOutput = maxOutput;
}

PIDController::Result PIDController::calculate(double setpoint,
    double measurement,
    double dt) noexcept
{
    // 1) 오차 계산
    const double error = setpoint - measurement;
    m_lastError.store(error);

    // 2) 적분 항 업데이트 (anti-windup 은 제한 이후 적용 가능)
    m_integral += error * dt;

    // 3) 미분 항 계산 및 1차 필터
    const double rawDeriv = (error - m_prevError) / dt;
    m_derivative = (m_derivativeTau * m_derivative + dt * rawDeriv)
        / (m_derivativeTau + dt);

    // 4) P + I + D 합산
    double output = m_kp * error
        + m_ki * m_integral
        + m_kd * m_derivative;

    // 5) 출력 한계 제한 설정(clamping)
    output = std::clamp(output, m_minOutput, m_maxOutput);

    // 6) 상태 저장
    m_prevError = error;

    return { output, error };
}

void PIDController::reset() noexcept {
    m_prevError = 0.0;
    m_integral = 0.0;
    m_derivative = 0.0;
    m_lastError.store(0.0);
}
