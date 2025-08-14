#pragma once
#include <string>
#include <vector>

// RT_Trajectory 구조체 정의
struct RT_Trajectory {
	float joint_ang[6]; // 조인트 각도 [rad]
};

// RT_Trajectory 구조체의 크기를 정의
bool LoadTrajectoryData(const std::string& file,
    std::vector<RT_Trajectory>& out);
