#pragma once
#include <string>
#include <vector>

// RT_Trajectory ����ü ����
struct RT_Trajectory {
	float joint_ang[6]; // ����Ʈ ���� [rad]
};

// RT_Trajectory ����ü�� ũ�⸦ ����
bool LoadTrajectoryData(const std::string& file,
    std::vector<RT_Trajectory>& out);
