#ifndef LOWPASSFILTER_H_
#define LOWPASSFILTER_H_

#include <cmath>

// 2차 Butterworth 로우패스 필터 클래스
class LowPassFilter {
public:
	// 생성자: 샘플링 주기와 컷오프 주파수를 받아 필터 계수 계산
	//   - sampleRate : 샘플링 주파수 (Hz)
	//   - cutoffFreq : 컷오프 주파수 (Hz)
    LowPassFilter(double sampleRate, double cutoffFreq);

	// 필터 초기화
    void reset();

	// 필터링 함수: 입력 신호를 받아 필터링된 출력을 반환
    double filter(double input);

private:
	double b0, b1, b2, a1, a2;  // 필터 계수
	double x1 = 0.0, x2 = 0.0;	// 입력 신호의 이전 값들
	double y1 = 0.0, y2 = 0.0;	// 출력 신호의 이전 값들
};

#endif // LOWPASSFILTER_H_
