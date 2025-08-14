#include "TrajectoryIO.h"
#include <fstream>
#include <sstream>
#include <iostream>

bool LoadTrajectoryData(const std::string& file,
    std::vector<RT_Trajectory>& traj)
{
    std::ifstream fs(file);
    if (!fs.is_open()) { std::cerr << "���� ���� ����\n"; return false; }

	std::string line;                       // ���Ͽ��� �� �پ� �б� ���� ���ڿ� ����
	
	// ���Ͽ��� �� ������ �б�
    while (std::getline(fs, line)) {
		// ���� ��������� �ǳʶٱ�
        if (line.empty()) continue;
		
        std::istringstream iss(line);       // ���� ������� ������ traj�� �߰�
		
        RT_Trajectory rt{};                 // RT_Trajectory rt{}; // RT_Trajectory ����ü �ʱ�ȭ

		// ����Ʈ ������ �о����
        for (int i = 0; i < 6; ++i) if (!(iss >> rt.joint_ang[i])) return false;
		traj.push_back(rt);                 // �о�� ����Ʈ ������ traj ���Ϳ� �߰�
    }
    return true;
}