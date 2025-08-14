#ifndef LOWPASSFILTER_H_
#define LOWPASSFILTER_H_

#include <cmath>

// 2�� Butterworth �ο��н� ���� Ŭ����
class LowPassFilter {
public:
	// ������: ���ø� �ֱ�� �ƿ��� ���ļ��� �޾� ���� ��� ���
	//   - sampleRate : ���ø� ���ļ� (Hz)
	//   - cutoffFreq : �ƿ��� ���ļ� (Hz)
    LowPassFilter(double sampleRate, double cutoffFreq);

	// ���� �ʱ�ȭ
    void reset();

	// ���͸� �Լ�: �Է� ��ȣ�� �޾� ���͸��� ����� ��ȯ
    double filter(double input);

private:
	double b0, b1, b2, a1, a2;  // ���� ���
	double x1 = 0.0, x2 = 0.0;	// �Է� ��ȣ�� ���� ����
	double y1 = 0.0, y2 = 0.0;	// ��� ��ȣ�� ���� ����
};

#endif // LOWPASSFILTER_H_
