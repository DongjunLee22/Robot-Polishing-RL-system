#pragma once

#pragma pack(push, 1)
// C++ -> Python ���ۿ� ��Ŷ
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


#pragma pack(push, 1)
// Python -> C++ ���ſ� ��Ŷ
struct RLAgentPacket {
	unsigned short	sof;				// 0xBBBB(2 bytes)
	float			rlVoltageValue;		// RL ������Ʈ ���� (4 bytes)
	unsigned char   confirmFlag;		// RL ������Ʈ�κ��� �޼��� ���� Ȯ�ο� �÷��� (1 byte)
	unsigned short	checksum;			// ������ ���Ἲ ���� (2 bytes)
};
#pragma pack(pop)