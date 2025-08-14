#ifndef TRANSFORMATION_FORCE_H
#define TRANSFORMATION_FORCE_H

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// 접촉력의 벡터를 변환하는 함수
class Transformation_Force {
private:
	float F_Biased_wrt_sensor[3];   // 센서 기준으로 바이어스된 힘 벡터 
	float R_zyz[3][3];              // ZYZ 회전 행렬
	float R_zyz_trans[3][3];        // ZYZ 회전 행렬의 전치 행렬

	// 회전 행렬을 계산하는 함수
    void calculateRotationMatrix(float ZYZ_a_rad, float ZYZ_b_rad, float ZYZ_c_rad);

public:
	// 생성자: 센서 기준으로 바이어스된 힘 벡터와 현재 위치를 받아 회전 행렬을 초기화
    Transformation_Force(float F_Biased[3], float pos_actual[6]);

	// 힘 벡터를 변환하는 함수
    void transformForce(float F_wrt_base[3]);
};

#endif // TRANSFORMATION_FORCE_H
