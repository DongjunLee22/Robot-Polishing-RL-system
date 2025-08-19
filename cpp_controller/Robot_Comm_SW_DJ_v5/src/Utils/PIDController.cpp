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

void PIDController::setOutputLimits(double minOutput, double maxOutput) noexcept {
	m_minOutput = minOutput;
	m_maxOutput = maxOutput;
}

PIDController::Result PIDController::calculate(double setpoint, double measurement, double dt) noexcept
{
	// 0) dt ������ üũ
	if (dt < 1e-9) {
		// ���� ��°��� �״�� ��ȯ�ϰų�, P�׸� ����ϴ� ���� ó�� ����
		return { m_lastOutput.load(), m_lastError.load() };
	}

	// 1) ���� ���
	const double error = setpoint - measurement;
	m_lastError.store(error);

	// 2) ��� �� ���
	const double p_term = m_kp * error;

	// 3) �̺� �� ��� �� 1�� ����
	const double rawDeriv = (m_prevMeasurement - measurement) / dt;
	m_derivative = (m_derivativeTau * m_derivative + dt * rawDeriv)
		/ (m_derivativeTau + dt);
	const double d_term = m_kd * m_derivative;

	// 4) ���� ���� ������ P + D �ջ�
	// Integral Windup ������ ���� ���� ���� ���߿� ����
	const double pd_output = p_term + d_term;

	// 5) ���� �� ���
	const double i_term = m_ki * (m_integral + error * dt);

	// 6) ��� �Ѱ�(clamping) �� Anti-Windup ����
	double output = pd_output + i_term;
	double clamped_output = std::clamp(output, m_minOutput, m_maxOutput);

	// ����� ��ȭ���� �ʾ��� ���� ���� ���� ���� (Conditional Integration)
	if (output == clamped_output) {
		m_integral += error * dt;
	}

	// 7) ���� ����
	m_prevError = error;
	m_prevMeasurement = measurement; // ���� ������ ������Ʈ
	m_lastOutput.store(clamped_output);   // ������ ��°� ������Ʈ

	return { clamped_output, error };
}

void PIDController::reset() noexcept {
	m_prevError = 0.0;
	m_prevMeasurement = 0.0;
	m_integral = 0.0;
	m_derivative = 0.0;
	m_lastError.store(0.0);
	m_lastOutput.store(0.0);
}
