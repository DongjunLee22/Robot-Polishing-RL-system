#define _USE_MATH_DEFINES
#include <cmath>
#include "LowPassFilter.h"

LowPassFilter::LowPassFilter(double sampleRate, double cutoffFreq) {
    // bilinear transform을 이용한 2차 Butterworth 설계
	double K = std::tan(M_PI * cutoffFreq / sampleRate);    // 디지털 필터링을 위한 변환
	double K2 = K * K;
    double norm = 1.0 + std::sqrt(2.0) * K + K2;

	// 필터 계수 계산
    b0 = K2 / norm;
    b1 = 2.0 * K2 / norm;
    b2 = K2 / norm;
    a1 = 2.0 * (K2 - 1.0) / norm;
    a2 = (1.0 - std::sqrt(2.0) * K + K2) / norm;
}

void LowPassFilter::reset() {
    // 이전 입력 및 출력 값을 초기화
    x1 = x2 = y1 = y2 = 0.0;  
}

double LowPassFilter::filter(double input) {
	// 필터링된 출력 계산
    double out = b0 * input + b1 * x1 + b2 * x2
        - a1 * y1 - a2 * y2;

	// 이전 입력 및 출력 값 업데이트
    x2 = x1;  x1 = input;
    y2 = y1;  y1 = out;
    return out;
}
