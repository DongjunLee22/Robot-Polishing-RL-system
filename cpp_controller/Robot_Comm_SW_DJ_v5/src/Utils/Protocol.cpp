#include "pch.h" // MFC 프로젝트의 프리컴파일 헤더
#include "Protocol.h"
#include <winsock2.h> // htons, ntohs, htonl, ntohl 함수 사용을 위해 추가

#pragma comment(lib, "ws2_32.lib") // winsock2 라이브러리 링크

// CRC-16 체크섬 계산 함수
unsigned short calculate_crc16(const unsigned char* data, size_t length)
{
	unsigned short crc = 0xFFFF;
	for (size_t i = 0; i < length; i++) {
		crc ^= (unsigned short)data[i];
		for (int j = 0; j < 8; j++) {
			if (crc & 0x0001) {
				crc = (crc >> 1) ^ 0xA001; // CRC-16/MODBUS
			}
			else {
				crc = crc >> 1;
			}
		}
	}
	return crc;
}

// Packing 함수
std::vector<char> PackRobotStatus(float current_forceZ, float target_forceZ, float error_forceZ, float error_forceZ_dot, float error_forceZ_int,
	float cur_PID_output, unsigned char Sander_Flag)
{
	PythonCommPacket packet;
	packet.sof = 0xAAAA;
	packet.RL_currentForceZ = current_forceZ;
	packet.RL_targetForceZ = target_forceZ;
	packet.RL_forceZError = error_forceZ;
	packet.RL_forceZErrordot = error_forceZ_dot;
	packet.RL_forceZErrorintegral = error_forceZ_int;
	packet.RL_currentPID = cur_PID_output;
	packet.RL_sanderactiveFlag = Sander_Flag;

	// 바이트 순서 변환
	packet.sof = htons(packet.sof);
	*(unsigned long*)&packet.RL_currentForceZ = htonl(*(unsigned long*)&packet.RL_currentForceZ);
	*(unsigned long*)&packet.RL_targetForceZ = htonl(*(unsigned long*)&packet.RL_targetForceZ);
	*(unsigned long*)&packet.RL_forceZError = htonl(*(unsigned long*)&packet.RL_forceZError);
	*(unsigned long*)&packet.RL_forceZErrordot = htonl(*(unsigned long*)&packet.RL_forceZErrordot);
	*(unsigned long*)&packet.RL_forceZErrorintegral = htonl(*(unsigned long*)&packet.RL_forceZErrorintegral);
	*(unsigned long*)&packet.RL_currentPID = htonl(*(unsigned long*)&packet.RL_currentPID);
	*(unsigned long*)&packet.RL_sanderactiveFlag = htonl(*(unsigned long*)&packet.RL_sanderactiveFlag);

	// 체크섬 계산 및 설정
	packet.checksum = calculate_crc16((const unsigned char*)&packet, sizeof(packet) - sizeof(unsigned short));
	packet.checksum = htons(packet.checksum);

	const char* pBegin = reinterpret_cast<const char*>(&packet);
	return std::vector<char>(pBegin, pBegin + sizeof(packet));
}

// Unpacking 함수
bool UnpackRLAgentCommand(const char* buffer, int length, RLAgentPacket& outPacket)
{
	if (length < sizeof(RLAgentPacket)) {
		return false;
	}

	// 원본 데이터를 복사하여 사용 (원본 버퍼를 직접 수정하지 않기 위해)
	RLAgentPacket received_packet = *reinterpret_cast<const RLAgentPacket*>(buffer);

	// 1. 체크섬 검증
	unsigned short received_checksum = ntohs(received_packet.checksum);
	unsigned short calculated_checksum = calculate_crc16(
		(const unsigned char*)&received_packet,
		sizeof(RLAgentPacket) - sizeof(unsigned short)
	);

	if (received_checksum != calculated_checksum) {
		TRACE("Checksum error in Protocol Unpacking!\n");
		return false; // 체크섬 불일치 시 실패
	}

	// 2. 바이트 순서 변환 (Network to Host)
	outPacket.sof = ntohs(received_packet.sof);

	float rl_Pressure = received_packet.RL_ResidualP;
	*(unsigned long*)&rl_Pressure = ntohl(*(unsigned long*)&rl_Pressure);
	outPacket.RL_ResidualP = rl_Pressure;

	outPacket.RL_MessagerecvFlag = received_packet.RL_MessagerecvFlag;
	outPacket.checksum = received_checksum; // 호스트 바이트 순서로 저장

	return true;
}