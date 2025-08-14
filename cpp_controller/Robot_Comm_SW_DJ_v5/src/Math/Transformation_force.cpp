#include "Transformation_force.h"

Transformation_Force::Transformation_Force(float F_Biased[3], float pos_actual[6]) {
	// ���̾�� �� ���͸� ���� �������� ����
    F_Biased_wrt_sensor[0] = F_Biased[0];
    F_Biased_wrt_sensor[1] = F_Biased[1];
    F_Biased_wrt_sensor[2] = F_Biased[2];

	// ZYZ ȸ�� ����� ����ϱ� ���� ���� ��ġ�� �������� ��ȯ
    float ZYZ_a_rad = (float)pos_actual[3] * (float)M_PI / 180;
    float ZYZ_b_rad = (float)pos_actual[4] * (float)M_PI / 180;
    float ZYZ_c_rad = (float)pos_actual[5] * (float)M_PI / 180;

	// ȸ�� ��� ���
    calculateRotationMatrix(ZYZ_a_rad, ZYZ_b_rad, ZYZ_c_rad);
}

void Transformation_Force::calculateRotationMatrix(float ZYZ_a_rad, float ZYZ_b_rad, float ZYZ_c_rad) {
    // �� �࿡ ���� ȸ�� ��� ���
    float Rz_alpha[3][3] = {
        {cos(ZYZ_a_rad), -sin(ZYZ_a_rad), 0},
        {sin(ZYZ_a_rad), cos(ZYZ_a_rad), 0},
        {0, 0, 1}
    };

    float Ry_beta[3][3] = {
        {cos(ZYZ_b_rad), 0, sin(ZYZ_b_rad)},
        {0, 1, 0},
        {-sin(ZYZ_b_rad), 0, cos(ZYZ_b_rad)}
    };

    float Rz_gamma[3][3] = {
        {cos(ZYZ_c_rad), -sin(ZYZ_c_rad), 0},
        {sin(ZYZ_c_rad), cos(ZYZ_c_rad), 0},
        {0, 0, 1}
    };

	// ȸ�� ����� ������ ���� ���� ZYZ ȸ�� ��� ���
    float R_temp[3][3] = { 0 };
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            for (int k = 0; k < 3; ++k) {
                R_temp[i][j] += Rz_alpha[i][k] * Ry_beta[k][j];
            }
        }
    }

    // ���� ȸ�� ��� ���
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            R_zyz[i][j] = 0;
            for (int k = 0; k < 3; ++k) {
                R_zyz[i][j] += R_temp[i][k] * Rz_gamma[k][j];
            }
        }
    }

    // ȸ�� ����� ��ġ ��� ���
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            R_zyz_trans[j][i] = R_zyz[i][j];
        }
    }
}

void Transformation_Force::transformForce(float F_wrt_base[3]) {
	// ���� ������ ���̾�� �� ���͸� ZYZ ȸ�� ����� ����Ͽ� ��ȯ
    for (int i = 0; i < 3; ++i) {
        F_wrt_base[i] = 0;
        for (int j = 0; j < 3; ++j) {
            F_wrt_base[i] += R_zyz[i][j] * F_Biased_wrt_sensor[j];
        }
    }
}
