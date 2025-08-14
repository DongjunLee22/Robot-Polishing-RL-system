#pragma once
#include <vector>

// C++ -> Python 전송용 패킷
#pragma pack(push, 1)
struct PythonCommPacket {
	unsigned short	sof;				// 0xAAAA (2 bytes)
	float			contactForceZ;		// z방향 접촉력 (4 bytes)
	float           chamberPressure;	// 공압 챔버 압력 (4 bytes)
	float			chamberVoltage;		// 공압 챔버 전압 (4 bytes)
	float           pidControlValue;	// PID 제어 값 (4 bytes)
	unsigned char   pidFlag;			// PID 플래그 (1 byte)
	unsigned short  checksum;			// 데이터 무결성 검증 (2 bytes)
};
#pragma pack(pop)

// Python -> C++ 수신용 패킷
#pragma pack(push, 1)
struct RLAgentPacket {
	unsigned short	sof;				// 0xBBBB(2 bytes)
	float			rlVoltageValue;		// RL 에이전트 전압 (4 bytes)
	unsigned char   confirmFlag;		// RL 에이전트로부터 메세지 수신 확인용 플래그 (1 byte)
	unsigned short	checksum;			// 데이터 무결성 검증 (2 bytes)
};
#pragma pack(pop)

// ==========================================================
// 프로토콜 관련 함수 선언
// ==========================================================

// CRC-16 체크섬 계산 함수
unsigned short calculate_crc16(const unsigned char* data, size_t length);

// PythonCommPacket을 전송용 바이트 벡터로 변환 (Packing)
std::vector<char> PackRobotStatus(float forceZ, float pressure, float voltage, float pidVal, unsigned char flag);

// 수신된 바이트 데이터를 RLAgentPacket 구조체로 변환 (Unpacking)
// 성공 시 true, 체크섬 오류 등 실패 시 false 반환
bool UnpackRLAgentCommand(const char* buffer, int length, RLAgentPacket& outPacket);