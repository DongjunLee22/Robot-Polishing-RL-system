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
    // 1) ���� ���
    const double error = setpoint - measurement;
    m_lastError.store(error);

    // 2) ���� �� ������Ʈ (anti-windup �� ���� ���� ���� ����)
    m_integral += error * dt;

    // 3) �̺� �� ��� �� 1�� ����
    const double rawDeriv = (error - m_prevError) / dt;
    m_derivative = (m_derivativeTau * m_derivative + dt * rawDeriv)
        / (m_derivativeTau + dt);

    // 4) P + I + D �ջ�
    double output = m_kp * error
        + m_ki * m_integral
        + m_kd * m_derivative;

    // 5) ��� �Ѱ� ���� ����(clamping)
    output = std::clamp(output, m_minOutput, m_maxOutput);

    // 6) ���� ����
    m_prevError = error;

    return { output, error };
}

void PIDController::reset() noexcept {
    m_prevError = 0.0;
    m_integral = 0.0;
    m_derivative = 0.0;
    m_lastError.store(0.0);
}
