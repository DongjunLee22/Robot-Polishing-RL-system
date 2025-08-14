#pragma once

#pragma pack(push, 1)
// C++ -> Python 전송용 패킷
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


#pragma pack(push, 1)
// Python -> C++ 수신용 패킷
struct RLAgentPacket {
	unsigned short	sof;				// 0xBBBB(2 bytes)
	float			rlVoltageValue;		// RL 에이전트 전압 (4 bytes)
	unsigned char   confirmFlag;		// RL 에이전트로부터 메세지 수신 확인용 플래그 (1 byte)
	unsigned short	checksum;			// 데이터 무결성 검증 (2 bytes)
};
#pragma pack(pop)