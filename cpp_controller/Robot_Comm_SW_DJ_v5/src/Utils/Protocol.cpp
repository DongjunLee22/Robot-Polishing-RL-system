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
	packet.sof = 0xAAAA;									// Start of Frame (2����Ʈ)
	packet.RL_currentForceZ = current_forceZ;				// ���� z���� ���˷� (4����Ʈ)
	packet.RL_targetForceZ = target_forceZ;					// ��ǥ z���� ���˷� (4����Ʈ)
	packet.RL_forceZError = error_forceZ;					// z���� ���˷� ���� (4����Ʈ)
	packet.RL_forceZErrordot = error_forceZ_dot;			// z���� ���˷� ���� �̺а� (4����Ʈ)
	packet.RL_forceZErrorintegral = error_forceZ_int;		// z���� ���˷� ���� ���а� (4����Ʈ)
	packet.RL_currentPID = cur_PID_output;					// ���� PID ��°� (4����Ʈ)
	packet.RL_sanderactiveFlag = Sander_Flag;				// Sander Ȱ��ȭ �÷��� (1����Ʈ)

	// ��Ŷ ũ�� ��� (������)
	printf("PythonCommPacket size: %zu bytes\n", sizeof(packet));

	// ����Ʈ ���� ��ȯ
	packet.sof = htons(packet.sof);
	*(unsigned long*)&packet.RL_currentForceZ = htonl(*(unsigned long*)&packet.RL_currentForceZ);
	*(unsigned long*)&packet.RL_targetForceZ = htonl(*(unsigned long*)&packet.RL_targetForceZ);
	*(unsigned long*)&packet.RL_forceZError = htonl(*(unsigned long*)&packet.RL_forceZError);
	*(unsigned long*)&packet.RL_forceZErrordot = htonl(*(unsigned long*)&packet.RL_forceZErrordot);
	*(unsigned long*)&packet.RL_forceZErrorintegral = htonl(*(unsigned long*)&packet.RL_forceZErrorintegral);
	*(unsigned long*)&packet.RL_currentPID = htonl(*(unsigned long*)&packet.RL_currentPID);

	// üũ�� ��� �� ����
	packet.checksum = calculate_crc16((const unsigned char*)&packet, sizeof(packet) - sizeof(unsigned short));
	packet.checksum = htons(packet.checksum);				// üũ�� (2����Ʈ)

	// ���� ������ ������ ũ�� ��� (������)
	std::vector<char> result(reinterpret_cast<const char*>(&packet),
		reinterpret_cast<const char*>(&packet) + sizeof(packet));
	printf("Sending packet size: %zu bytes\n", result.size());

	return result;
	
	// 29����Ʈ ���� ����� �Ҵ�
	//std::vector<char> buffer(29);
	//size_t offset = 0;

	//// SOF (2����Ʈ)
	//unsigned short sof = htons(0xAAAA);
	//memcpy(&buffer[offset], &sof, sizeof(sof));
	//offset += sizeof(sof);

	//// 6�� float �� (�� 4����Ʈ)
	//float values[] = {
	//	current_forceZ,
	//	target_forceZ,
	//	error_forceZ,
	//	error_forceZ_dot,
	//	error_forceZ_int,
	//	cur_PID_output
	//};

	//for (auto value : values) {
	//	unsigned long netValue = htonl(*reinterpret_cast<unsigned long*>(&value));
	//	memcpy(&buffer[offset], &netValue, sizeof(netValue));
	//	offset += sizeof(netValue);
	//}

	//// RL_sanderactiveFlag (1����Ʈ)
	//buffer[offset++] = Sander_Flag;  // 1����Ʈ�� �״�� ����

	//// üũ�� ��� (2����Ʈ)
	//unsigned short checksum = calculate_crc16(
	//	reinterpret_cast<const unsigned char*>(buffer.data()),
	//	buffer.size() - sizeof(unsigned short));
	//unsigned short net_checksum = htons(checksum);
	//memcpy(&buffer[offset], &net_checksum, sizeof(net_checksum));

	//printf("Manual buffer size: %zu bytes\n", buffer.size());

	//return buffer;
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