#pragma once
#ifndef TRANSFORMATION_TORQUE_H
#define TRANSFORMATION_TORQUE_H

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
class Transformation_Torque {
private:
	float T_Biased_wrt_sensor[3];       // 센서 기준으로 바이어스된 토크 벡터
	float R_zyz[3][3];                  // 회전 행렬 (ZYZ Euler angles)
	float R_zyz_trans[3][3];            // 회전 행렬의 전치 행렬

	// 회전 행렬을 계산하는 함수
    void calculateRotationMatrix(float ZYZ_a_rad, float ZYZ_b_rad, float ZYZ_c_rad);

public:
	// 생성자: 센서 기준으로 바이어스된 토크 벡터와 현재 위치를 받아 회전 행렬을 초기화
    Transformation_Torque(float T_Biased[3], float pos_actual[6]);

    // 토크 벡터를 변환하는 함수
    void transformTorque(float T_wrt_base[3]);
};

#endif // TRANSFORMATION_TORQUE_H
