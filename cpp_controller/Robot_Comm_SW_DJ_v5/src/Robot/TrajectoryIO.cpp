#include "TrajectoryIO.h"
#include <fstream>
#include <sstream>
#include <iostream>

bool LoadTrajectoryData(const std::string& file,
    std::vector<RT_Trajectory>& traj)
{
    std::ifstream fs(file);
    if (!fs.is_open()) { std::cerr << "파일 열기 실패\n"; return false; }

	std::string line;                       // 파일에서 한 줄씩 읽기 위한 문자열 변수
	
	// 파일에서 줄 단위로 읽기
    while (std::getline(fs, line)) {
		// 줄이 비어있으면 건너뛰기
        if (line.empty()) continue;
		
        std::istringstream iss(line);       // 줄이 비어있지 않으면 traj에 추가
		
        RT_Trajectory rt{};                 // RT_Trajectory rt{}; // RT_Trajectory 구조체 초기화

		// 조인트 각도를 읽어오기
        for (int i = 0; i < 6; ++i) if (!(iss >> rt.joint_ang[i])) return false;
		traj.push_back(rt);                 // 읽어온 조인트 각도를 traj 벡터에 추가
    }
    return true;
}