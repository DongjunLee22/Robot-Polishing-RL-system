#ifndef TRANSFORMATION_FORCE_H
#define TRANSFORMATION_FORCE_H

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ���˷��� ���͸� ��ȯ�ϴ� �Լ�
class Transformation_Force {
private:
	float F_Biased_wrt_sensor[3];   // ���� �������� ���̾�� �� ���� 
	float R_zyz[3][3];              // ZYZ ȸ�� ���
	float R_zyz_trans[3][3];        // ZYZ ȸ�� ����� ��ġ ���

	// ȸ�� ����� ����ϴ� �Լ�
    void calculateRotationMatrix(float ZYZ_a_rad, float ZYZ_b_rad, float ZYZ_c_rad);

public:
	// ������: ���� �������� ���̾�� �� ���Ϳ� ���� ��ġ�� �޾� ȸ�� ����� �ʱ�ȭ
    Transformation_Force(float F_Biased[3], float pos_actual[6]);

	// �� ���͸� ��ȯ�ϴ� �Լ�
    void transformForce(float F_wrt_base[3]);
};

#endif // TRANSFORMATION_FORCE_H
