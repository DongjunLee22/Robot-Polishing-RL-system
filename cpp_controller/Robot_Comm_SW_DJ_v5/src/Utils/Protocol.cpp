#include "pch.h" // MFC ������Ʈ�� ���������� ���
#include "Protocol.h"
#include <winsock2.h> // htons, ntohs, htonl, ntohl �Լ� ����� ���� �߰�

#pragma comment(lib, "ws2_32.lib") // winsock2 ���̺귯�� ��ũ

// CRC-16 üũ�� ��� �Լ�
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

// Packing �Լ�
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

	// ����Ʈ ���� ��ȯ
	packet.sof = htons(packet.sof);
	*(unsigned long*)&packet.RL_currentForceZ = htonl(*(unsigned long*)&packet.RL_currentForceZ);
	*(unsigned long*)&packet.RL_targetForceZ = htonl(*(unsigned long*)&packet.RL_targetForceZ);
	*(unsigned long*)&packet.RL_forceZError = htonl(*(unsigned long*)&packet.RL_forceZError);
	*(unsigned long*)&packet.RL_forceZErrordot = htonl(*(unsigned long*)&packet.RL_forceZErrordot);
	*(unsigned long*)&packet.RL_forceZErrorintegral = htonl(*(unsigned long*)&packet.RL_forceZErrorintegral);
	*(unsigned long*)&packet.RL_currentPID = htonl(*(unsigned long*)&packet.RL_currentPID);
	*(unsigned long*)&packet.RL_sanderactiveFlag = htonl(*(unsigned long*)&packet.RL_sanderactiveFlag);

	// üũ�� ��� �� ����
	packet.checksum = calculate_crc16((const unsigned char*)&packet, sizeof(packet) - sizeof(unsigned short));
	packet.checksum = htons(packet.checksum);

	const char* pBegin = reinterpret_cast<const char*>(&packet);
	return std::vector<char>(pBegin, pBegin + sizeof(packet));
}

// Unpacking �Լ�
bool UnpackRLAgentCommand(const char* buffer, int length, RLAgentPacket& outPacket)
{
	if (length < sizeof(RLAgentPacket)) {
		return false;
	}

	// ���� �����͸� �����Ͽ� ��� (���� ���۸� ���� �������� �ʱ� ����)
	RLAgentPacket received_packet = *reinterpret_cast<const RLAgentPacket*>(buffer);

	// 1. üũ�� ����
	unsigned short received_checksum = ntohs(received_packet.checksum);
	unsigned short calculated_checksum = calculate_crc16(
		(const unsigned char*)&received_packet,
		sizeof(RLAgentPacket) - sizeof(unsigned short)
	);

	if (received_checksum != calculated_checksum) {
		TRACE("Checksum error in Protocol Unpacking!\n");
		return false; // üũ�� ����ġ �� ����
	}

	// 2. ����Ʈ ���� ��ȯ (Network to Host)
	outPacket.sof = ntohs(received_packet.sof);

	float rl_Pressure = received_packet.RL_ResidualP;
	*(unsigned long*)&rl_Pressure = ntohl(*(unsigned long*)&rl_Pressure);
	outPacket.RL_ResidualP = rl_Pressure;

	outPacket.RL_MessagerecvFlag = received_packet.RL_MessagerecvFlag;
	outPacket.checksum = received_checksum; // ȣ��Ʈ ����Ʈ ������ ����

	return true;
}