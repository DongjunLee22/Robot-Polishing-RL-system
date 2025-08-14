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
std::vector<char> PackRobotStatus(float forceZ, float pressure, float voltage, float pidVal, unsigned char flag)
{
	PythonCommPacket packet;
	packet.sof = 0xAAAA;
	packet.contactForceZ = forceZ;
	packet.chamberPressure = pressure;
	packet.chamberVoltage = voltage;
	packet.pidControlValue = pidVal;
	packet.pidFlag = flag;

	// 바이트 순서 변환
	packet.sof = htons(packet.sof);
	*(unsigned long*)&packet.contactForceZ = htonl(*(unsigned long*)&packet.contactForceZ);
	*(unsigned long*)&packet.chamberPressure = htonl(*(unsigned long*)&packet.chamberPressure);
	*(unsigned long*)&packet.chamberVoltage = htonl(*(unsigned long*)&packet.chamberVoltage);
	*(unsigned long*)&packet.pidControlValue = htonl(*(unsigned long*)&packet.pidControlValue);

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

	float rl_voltage = received_packet.rlVoltageValue;
	*(unsigned long*)&rl_voltage = ntohl(*(unsigned long*)&rl_voltage);
	outPacket.rlVoltageValue = rl_voltage;

	outPacket.confirmFlag = received_packet.confirmFlag;
	outPacket.checksum = received_checksum; // 호스트 바이트 순서로 저장

	return true;
}