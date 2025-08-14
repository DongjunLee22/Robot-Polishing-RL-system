#pragma once
#include <vector>

// C++ -> Python ���ۿ� ��Ŷ
#pragma pack(push, 1)
struct PythonCommPacket {
	unsigned short	sof;				// 0xAAAA (2 bytes)
	float			contactForceZ;		// z���� ���˷� (4 bytes)
	float           chamberPressure;	// ���� è�� �з� (4 bytes)
	float			chamberVoltage;		// ���� è�� ���� (4 bytes)
	float           pidControlValue;	// PID ���� �� (4 bytes)
	unsigned char   pidFlag;			// PID �÷��� (1 byte)
	unsigned short  checksum;			// ������ ���Ἲ ���� (2 bytes)
};
#pragma pack(pop)

// Python -> C++ ���ſ� ��Ŷ
#pragma pack(push, 1)
struct RLAgentPacket {
	unsigned short	sof;				// 0xBBBB(2 bytes)
	float			rlVoltageValue;		// RL ������Ʈ ���� (4 bytes)
	unsigned char   confirmFlag;		// RL ������Ʈ�κ��� �޼��� ���� Ȯ�ο� �÷��� (1 byte)
	unsigned short	checksum;			// ������ ���Ἲ ���� (2 bytes)
};
#pragma pack(pop)

// ==========================================================
// �������� ���� �Լ� ����
// ==========================================================

// CRC-16 üũ�� ��� �Լ�
unsigned short calculate_crc16(const unsigned char* data, size_t length);

// PythonCommPacket�� ���ۿ� ����Ʈ ���ͷ� ��ȯ (Packing)
std::vector<char> PackRobotStatus(float forceZ, float pressure, float voltage, float pidVal, unsigned char flag);

// ���ŵ� ����Ʈ �����͸� RLAgentPacket ����ü�� ��ȯ (Unpacking)
// ���� �� true, üũ�� ���� �� ���� �� false ��ȯ
bool UnpackRLAgentCommand(const char* buffer, int length, RLAgentPacket& outPacket);