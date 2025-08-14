// RobotState.h
#pragma once

#include <array>
#include <atomic>
#include <mutex>
#include <functional>

// ���ӽ����̽� RobotComm ����
namespace RobotComm {	
	inline constexpr char VERSION[] = "v1.0";	// ���� ����
	inline constexpr float PERIOD_S = 0.001f;	// �κ� �ֱ� [s]
	inline constexpr int LOSS = -1;				// �κ� ������ �ս� �� ����
}

// �κ� ���� ������ �����ϴ� Ŭ����
class RobotState {
public:
	// �κ� ���� ������ ����ü
	struct Snapshot {						
        std::array<float, 6> tcpPos{};
		std::array<float, 6> flangePos{};
        std::array<float, 6> jointAng{};
        std::array<float, 6> tcpVel{};
		std::array<float, 6> flangeVel{};
        std::array<float, 6> jointVel{};
		std::array<float, 3> desVel{};
    };

    // ������
    RobotState() noexcept;

    // ���� �ʱ�ȭ
    void reset() noexcept;

    // ������ ���� �޼���
	Snapshot updateSnapshot() noexcept;											// ���� ���¸� ���������� ������Ʈ�ϰ� ��ȯ
	Snapshot getSnapshot() const noexcept;										// ���� ���� �������� ��ȯ (�б� ����)

    // �� ���� �޼���
	void withLock(std::function<void(const RobotState&)> cb) const noexcept;	// ���ؽ� ��ȣ�� �ݹ� ����

    // ��ġ ���� �޼���
	void saveInitialtcpPosition(const float* data, size_t sz) noexcept;			// ���� TCP ��ġ�� �ʱ� ��ġ�� ����
	void saveInitialflangePosition(const float* data, size_t sz) noexcept;		// ���� Flange ��ġ�� �ʱ� ��ġ�� ����
	void saveInitialjointPosition(const float* data, size_t sz) noexcept;		// ���� ���� ������ �ʱ� ��ġ�� ����

	// �������̼� ������ ���� �޼���
	void saveRelaxationtcpPosition(const float* tcpPos, size_t sz) noexcept;	// ���� TCP ��ġ�� �������̼� ���������� ����
	void saveRelaxationflangePosition(const float* flangePos, size_t sz)		// ���� Flange ��ġ�� �������̼� ���������� ����
		noexcept;
	void saveRelaxationjointPosition(const float* jointAng, size_t sz)			// ���� ���� ������ �������̼� ������ ����
		noexcept;
		
    // ������ ������Ʈ �޼���
	void updateAllRawData(const float* tP, const float* fP, const float* jA,	// ��� ���� ������ ������Ʈ
        const float* tV, const float* fV, const float* jV, const float* desV,
		size_t sz) noexcept; 
	void updatetcpPosRaw(const float* d, size_t sz) noexcept;					// TCP ���� ��ġ ������Ʈ    
	void updateflangePosRaw(const float* d, size_t sz) noexcept;				// Flange ���� ��ġ ������Ʈ
	void updateJointAngRaw(const float* d, size_t sz) noexcept;					// ���� ���� ���� ������ ������Ʈ
	void updatetcpVelRaw(const float* d, size_t sz) noexcept;					// TCP �ӵ� ���� ������ ������Ʈ
	void updateflangeVelRaw(const float* d, size_t sz) noexcept;				// Flange �ӵ� ���� ������ ������Ʈ
	void updateJointVelRaw(const float* d, size_t sz) noexcept;					// ���� �ӵ� ���� ������ ������Ʈ

	void updatetcpPosition(const std::array<float, 6>& np)noexcept;				// TCP ��ġ ������Ʈ
	void updateflangePosition(const std::array<float, 6>& np) noexcept;			// Flange ��ġ ������Ʈ
	void updateJointAngles(const std::array<float, 6>& na) noexcept;			// ���� ���� ������Ʈ
	void updatetcpVelocity(const std::array<float, 6>& nv) noexcept;			// TCP �ӵ� ������Ʈ
	void updateflangeVelocity(const std::array<float, 6>& nv) noexcept;			// Flange �ӵ� ������Ʈ
	void updateJointVelocity(const std::array<float, 6>& nv) noexcept;			// ���� �ӵ� ������Ʈ

    // �迭 ��ȯ �޼���
	std::array<float, 6> gettcpPositionArray() const noexcept;					// TCP ��ġ �迭 ��ȯ
	std::array<float, 6> getflangePositionArray() const noexcept;				// Flange ��ġ �迭 ��ȯ
	std::array<float, 6> getJointAngleArray() const noexcept;					// ���� ���� �迭 ��ȯ
	std::array<float, 6> gettcpVelocityArray() const noexcept;					// TCP �ӵ� �迭 ��ȯ
	std::array<float, 6> getflangeVelocityArray() const noexcept;				// Flange �ӵ� �迭 ��ȯ
	std::array<float, 6> getJointVelocityArray() const noexcept;				// ���� �ӵ� �迭 ��ȯ
	std::array<float, 6> getInitialtcpPositionArray() const noexcept;			// �ʱ� TCP ��ġ �迭 ��ȯ
	std::array<float, 6> getInitialflangePositionArray() const noexcept;		// �ʱ� ��ġ �迭 ��ȯ
	std::array<float, 6> getInitialJointAngleArray() const noexcept;			// �ʱ� ���� ���� �迭 ��ȯ
	std::array<float, 6> getRelaxationtcpPositionArray() const noexcept;		// �������̼� TCP ������ �迭 ��ȯ
	std::array<float, 6> getRelaxationflangePositionArray() const noexcept;		// �������̼� Flange ������ �迭 ��ȯ
	std::array<float, 6> getRelaxationAngleArray() const noexcept;				// �������̼� ���� �迭 ��ȯ

	// ���� �� ���� �޼���
	float gettcpPosition(size_t i) const noexcept;								// Ư�� �ε����� TCP ��ġ ��������
	float getflangePosition(size_t i) const noexcept;							// Ư�� �ε����� Flange ��ġ ��������
	float getJointAngle(size_t i) const noexcept;								// Ư�� �ε����� ���� ���� ��������
	float gettcpVelocity(size_t i) const noexcept;								// Ư�� �ε����� TCP �ӵ� ��������
	float getflangeVelocity(size_t i) const noexcept;							// Ư�� �ε����� Flange �ӵ� ��������
	float getJointVelocity(size_t i) const noexcept;							// Ư�� �ε����� ���� �ӵ� ��������

    // ������ ���� �޼���
	void copytcpPositionData(float* out, size_t size) const noexcept;			// TCP ��ġ ������ ����
	void copyflangePositionData(float* out, size_t size) const noexcept;		// Flange ��ġ ������ ����
	void copyJointAngleData(float* out, size_t size) const noexcept;			// ���� ���� ������ ����
	void copytcpVelocityData(float* out, size_t size) const noexcept;			// TCP �ӵ� ������ ����
	void copyflangeVelocityData(float* out, size_t size) const noexcept;		// Flange �ӵ� ������ ����
	void copyJointVelocityData(float* out, size_t size) const noexcept;			// ���� �ӵ� ������ ����
	void copyInitialtcpPositionData(float* out, size_t size) const noexcept;	// �ʱ� TCP ��ġ ������ ����
	void copyInitialflangePositionData(float* out, size_t size) const noexcept;	// �ʱ� ��ġ ������ ����
	void copyInitialJointAngleData(float* out, size_t size) const noexcept;		// �ʱ� ���� ���� ������ ����
	void copyRelaxationtcpPositionData(float* out, size_t size) noexcept;		// �������̼� TCP ������ ������ ����
	void copyRelaxationflangePositionData(float* out, size_t size) noexcept;	// �������̼� Flange ������ ������ ����
	void copyRelaxationAngleData(float* out, size_t size) noexcept;				// �������̼� ���� ������ ����

private:
    mutable std::mutex m_mutex;
	std::array<float, 6> m_tcpPos{};											// TCP ��ġ				[mm, deg]
	std::array<float, 6> m_flangePos{};											// �÷��� ��ġ			[mm, deg]
	std::array<float, 6> m_jointAng{};											// ���� ����			[deg]
	std::array<float, 6> m_tcpVel{};											// TCP �ӵ�				[mm/s, deg/s]
	std::array<float, 6> m_flangeVel{};											// Flange �ӵ�			[mm/s, deg/s]
	std::array<float, 6> m_jointVel{};											// ���� �ӵ�			[deg/s]
	std::array<float, 3> m_desVel{};											// ��ǥ �ӵ�			[mm/s] (x, y, z)
	std::array<float, 6> m_initialtcpPos{};										// �ʱ� TCP ��ġ		[mm, deg]
	std::array<float, 6> m_initialflangePos{};									// �ʱ� Flange ��ġ		[mm, deg]
	std::array<float, 6> m_initialjointAng{};									// �ʱ� ���� ����		[deg]
	std::array<float, 6> m_relaxationtcpPos{};									// TCP �������̼� ������[mm, deg]
	std::array<float, 6> m_relaxationflangePos{};								// Flange �������̼� ������[mm, deg]
	std::array<float, 6> m_relaxationjoint{};									// �������̼� ����		[deg]
	Snapshot m_snapshot{};														// ���� ���� ������
};