#pragma once

#include <algorithm>    // std::clamp
#include <atomic>


class PIDController {
public:
	// ��� ����ü ����
	struct Result {
		double output;
		double error;
	};

	// ������
	PIDController(
		double kp = 0.0,                                                        // PID ��� ���� �ʱ�ȭ
		double ki = 0.0,    										            // PID ���� ���� �ʱ�ȭ
		double kd = 0.0,    						                            // PID �̺� ���� �ʱ�ȭ
		double minOutput = -1.0,                                                // PID ��� �ּ� �Ѱ� �ʱ�ȭ
		double maxOutput = +1.0) noexcept;                                      // PID ��� �ִ� �Ѱ� �ʱ�ȭ

	void setGains(double kp, double ki, double kd) noexcept;                    // PID ���� ���� �Լ�
	void setOutputLimits(double minOutput, double maxOutput) noexcept;          // PID ��� �Ѱ� ���� �Լ�

	Result calculate(                                                           // PID ��� �Լ�                                                       
		double setpoint,                                                        // ��ǥ��
		double measurement,                                                     // ������
		double dt) noexcept;                    				                // �ð� ���� (�� ����)

	void setIntegral(double integralValue) {
		this->m_integral = integralValue;
	}

	void reset() noexcept;                                                      // PID ���� �ʱ�ȭ

	double getKp()        const noexcept { return m_kp; }                       // PID ��� ���� �д� �Լ�
	double getKi()        const noexcept { return m_ki; }                       // PID ���� ���� �д� �Լ�
	double getKd()        const noexcept { return m_kd; }						// PID �̺� ���� �д� �Լ�
	double getLastError() const noexcept { return m_lastError.load(); }			// ������ ���� �д� �Լ�

private:

	double m_kp;																// PID ��� ����
	double m_ki;																// PID ���� ����
	double m_kd;																// PID �̺� ����

	double m_minOutput;															// PID ��� �ּ� �Ѱ�
	double m_maxOutput;															// PID ��� �ִ� �Ѱ�

	double m_prevError{ 0.0 };													// ���� ���� (�̺� ����)	
	double m_prevMeasurement{ 0.0 };											// ���� ������ (�̺� ����)

	double m_integral{ 0.0 };													// ���� �� (������ ���� �ջ�)
	double m_derivative{ 0.0 };													// �̺� �� (���� ��ȭ��)	

	double m_derivativeTau{ 0.01 };												// �̺� ���� Ÿ�� (1�� ���͸��� ���� �ð� ���, �� ����)

	std::atomic<double> m_lastOutput{ 0.0 };													// ������ ��°� (���� PID ��°� �����)
	std::atomic<double> m_lastError{ 0.0 };										// ������ ���� (������ ������ ���� atomic ����)
};
