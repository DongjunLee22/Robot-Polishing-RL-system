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
std::vector<char> PackRobotStatus(float forceZ, float pressure, float voltage, float pidVal, unsigned char flag)
{
	PythonCommPacket packet;
	packet.sof = 0xAAAA;
	packet.contactForceZ = forceZ;
	packet.chamberPressure = pressure;
	packet.chamberVoltage = voltage;
	packet.pidControlValue = pidVal;
	packet.pidFlag = flag;

	// ����Ʈ ���� ��ȯ
	packet.sof = htons(packet.sof);
	*(unsigned long*)&packet.contactForceZ = htonl(*(unsigned long*)&packet.contactForceZ);
	*(unsigned long*)&packet.chamberPressure = htonl(*(unsigned long*)&packet.chamberPressure);
	*(unsigned long*)&packet.chamberVoltage = htonl(*(unsigned long*)&packet.chamberVoltage);
	*(unsigned long*)&packet.pidControlValue = htonl(*(unsigned long*)&packet.pidControlValue);

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

	float rl_voltage = received_packet.rlVoltageValue;
	*(unsigned long*)&rl_voltage = ntohl(*(unsigned long*)&rl_voltage);
	outPacket.rlVoltageValue = rl_voltage;

	outPacket.confirmFlag = received_packet.confirmFlag;
	outPacket.checksum = received_checksum; // ȣ��Ʈ ����Ʈ ������ ����

	return true;
}