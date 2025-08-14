#pragma once
#ifndef TRANSFORMATION_TORQUE_H
#define TRANSFORMATION_TORQUE_H

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
class Transformation_Torque {
private:
	float T_Biased_wrt_sensor[3];       // ���� �������� ���̾�� ��ũ ����
	float R_zyz[3][3];                  // ȸ�� ��� (ZYZ Euler angles)
	float R_zyz_trans[3][3];            // ȸ�� ����� ��ġ ���

	// ȸ�� ����� ����ϴ� �Լ�
    void calculateRotationMatrix(float ZYZ_a_rad, float ZYZ_b_rad, float ZYZ_c_rad);

public:
	// ������: ���� �������� ���̾�� ��ũ ���Ϳ� ���� ��ġ�� �޾� ȸ�� ����� �ʱ�ȭ
    Transformation_Torque(float T_Biased[3], float pos_actual[6]);

    // ��ũ ���͸� ��ȯ�ϴ� �Լ�
    void transformTorque(float T_wrt_base[3]);
};

#endif // TRANSFORMATION_TORQUE_H
