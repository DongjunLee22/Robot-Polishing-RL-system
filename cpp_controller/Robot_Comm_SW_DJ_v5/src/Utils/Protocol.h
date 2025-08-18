#pragma once
#include <vector>

// C++ -> Python ���ۿ� ��Ŷ
#pragma pack(push, 1)
struct PythonCommPacket {
	unsigned short	sof;						// 0xAAAA (2 bytes)
	float			RL_currentForceZ;			// ������ z���� ���˷� (4 bytes)
	float			RL_targetForceZ;			// ��ǥ z���� ���˷� (4 bytes)
	float           RL_forceZError;				// z���� ���˷� ���� (4 bytes)
	float           RL_forceZErrordot;			// z���� ���˷� ������ �̺а� (4 bytes)
	float           RL_forceZErrorintegral;		// z���� ���˷� ������ ���а� (4 bytes)
	float           RL_currentChamberPressure;	// ���� ���� è�� �з� (4 bytes)
	unsigned char   RL_sanderactiveFlag;		// Sander ���� ���� �÷��� (1 byte)
	unsigned short  checksum;					// ������ ���Ἲ ���� (2 bytes)
};
#pragma pack(pop)

// Python -> C++ ���ſ� ��Ŷ
#pragma pack(push, 1)
struct RLAgentPacket {
	unsigned short	sof;						// 0xBBBB(2 bytes)
	float			RL_ResidualP;				// ����(RL ������Ʈ)�κ��� ���� ���� ���� (4 bytes)
	unsigned char   RL_MessagerecvFlag;			// ����(RL ������Ʈ)�κ��� �޼��� ���� Ȯ�ο� �÷��� (1 byte) - 0: �޼��� ���� X / 1: �޼��� ���� O
	unsigned char   RL_EpisodeFlag;				// ����(RL ������Ʈ)������ ���Ǽҵ� ���� ���� Ȯ�ο� �÷��� (1 byte)   - 0: ���Ǽҵ� ���� O / 1: ���Ǽҵ� ���� X
	unsigned short	checksum;					// ������ ���Ἲ ���� (2 bytes)
};
#pragma pack(pop)

// ==========================================================
// �������� ���� �Լ� ����
// ==========================================================

// CRC-16 üũ�� ��� �Լ�
unsigned short calculate_crc16(const unsigned char* data, size_t length);

// PythonCommPacket�� ���ۿ� ����Ʈ ���ͷ� ��ȯ (Packing)
std::vector<char> PackRobotStatus(float current_forceZ, float target_forceZ, float error_forceZ, float error_forceZ_dot, float error_forceZ_int,
	float cur_chamber_P, unsigned char Sander_Flag);

// ���ŵ� ����Ʈ �����͸� RLAgentPacket ����ü�� ��ȯ (Unpacking)
// ���� �� true, üũ�� ���� �� ���� �� false ��ȯ
bool UnpackRLAgentCommand(const char* buffer, int length, RLAgentPacket& outPacket);