#define _USE_MATH_DEFINES
#include <cmath>
#include "LowPassFilter.h"

LowPassFilter::LowPassFilter(double sampleRate, double cutoffFreq) {
    // bilinear transform�� �̿��� 2�� Butterworth ����
	double K = std::tan(M_PI * cutoffFreq / sampleRate);    // ������ ���͸��� ���� ��ȯ
	double K2 = K * K;
    double norm = 1.0 + std::sqrt(2.0) * K + K2;

	// ���� ��� ���
    b0 = K2 / norm;
    b1 = 2.0 * K2 / norm;
    b2 = K2 / norm;
    a1 = 2.0 * (K2 - 1.0) / norm;
    a2 = (1.0 - std::sqrt(2.0) * K + K2) / norm;
}

void LowPassFilter::reset() {
    // ���� �Է� �� ��� ���� �ʱ�ȭ
    x1 = x2 = y1 = y2 = 0.0;  
}

double LowPassFilter::filter(double input) {
	// ���͸��� ��� ���
    double out = b0 * input + b1 * x1 + b2 * x2
        - a1 * y1 - a2 * y2;

	// ���� �Է� �� ��� �� ������Ʈ
    x2 = x1;  x1 = input;
    y2 = y1;  y1 = out;
    return out;
}
