#include "Transformation_force.h"

Transformation_Force::Transformation_Force(float F_Biased[3], float pos_actual[6]) {
	// 바이어스된 힘 벡터를 센서 기준으로 설정
    F_Biased_wrt_sensor[0] = F_Biased[0];
    F_Biased_wrt_sensor[1] = F_Biased[1];
    F_Biased_wrt_sensor[2] = F_Biased[2];

	// ZYZ 회전 행렬을 계산하기 위해 현재 위치를 라디안으로 변환
    float ZYZ_a_rad = (float)pos_actual[3] * (float)M_PI / 180;
    float ZYZ_b_rad = (float)pos_actual[4] * (float)M_PI / 180;
    float ZYZ_c_rad = (float)pos_actual[5] * (float)M_PI / 180;

	// 회전 행렬 계산
    calculateRotationMatrix(ZYZ_a_rad, ZYZ_b_rad, ZYZ_c_rad);
}

void Transformation_Force::calculateRotationMatrix(float ZYZ_a_rad, float ZYZ_b_rad, float ZYZ_c_rad) {
    // 각 축에 대한 회전 행렬 계산
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

	// 회전 행렬의 곱셈을 통해 최종 ZYZ 회전 행렬 계산
    float R_temp[3][3] = { 0 };
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            for (int k = 0; k < 3; ++k) {
                R_temp[i][j] += Rz_alpha[i][k] * Ry_beta[k][j];
            }
        }
    }

    // 최종 회전 행렬 계산
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            R_zyz[i][j] = 0;
            for (int k = 0; k < 3; ++k) {
                R_zyz[i][j] += R_temp[i][k] * Rz_gamma[k][j];
            }
        }
    }

    // 회전 행렬의 전치 행렬 계산
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            R_zyz_trans[j][i] = R_zyz[i][j];
        }
    }
}

void Transformation_Force::transformForce(float F_wrt_base[3]) {
	// 센서 기준의 바이어스된 힘 벡터를 ZYZ 회전 행렬을 사용하여 변환
    for (int i = 0; i < 3; ++i) {
        F_wrt_base[i] = 0;
        for (int j = 0; j < 3; ++j) {
            F_wrt_base[i] += R_zyz[i][j] * F_Biased_wrt_sensor[j];
        }
    }
}
