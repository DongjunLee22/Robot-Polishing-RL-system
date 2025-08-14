#pragma once

#include <array>
#include <cstddef>
#include <mutex>

class FTState {
public:
	// ������ ����ü ����
    struct Snapshot {
        std::array<float, 3> biasedForce;
		std::array<float, 3> biasedTorque;
        std::array<float, 3> forceBase;
		std::array<float, 3> torqueBase;
        std::array<float, 3> filteredForce;
        std::array<float, 3> filteredTorque;
    };

    // ������
    FTState() = default;

    // Getter
	std::array<float, 3> getSensorForce() const noexcept;               // ���� �� ���� �Լ�
	std::array<float, 3> getSensorTorque() const noexcept;              // ���� ��ũ ���� �Լ�
	std::array<float, 3> getRawForce() const noexcept;                  // �������� ���� ���� �� ���� �Լ�
	std::array<float, 3> getRawTorque() const noexcept;                 // �������� ���� ���� ��ũ ���� �Լ�
	std::array<float, 3> getBiasForce() const noexcept;                 // ���̾ �� ���� �Լ�
	std::array<float, 3> getBiasTorque() const noexcept;                // ���̾ ��ũ ���� �Լ�
	std::array<float, 3> getBiasSumForce() const noexcept;              // ���̾ �ջ� �� ���� �Լ�
	std::array<float, 3> getBiasSumTorque() const noexcept;             // ���̾ �ջ� ��ũ ���� �Լ�
	std::array<float, 3> getBiasedForce() const noexcept;               // ���̾ ����� �� ���� �Լ�
	std::array<float, 3> getBiasedTorque() const noexcept;              // ���̾ ����� ��ũ ���� �Լ�
	std::array<float, 3> getForceBase() const noexcept;                 // �κ� ���̽� ��ǥ��� ��ȯ�� �� ���� �Լ�
	std::array<float, 3> getTorqueBase() const noexcept;                // �κ� ���̽� ��ǥ��� ��ȯ�� ��ũ ���� �Լ�
	std::array<float, 3> getFilteredForce() const noexcept;             // ���͸��� �� ���� �Լ�
	std::array<float, 3> getFilteredTorque() const noexcept;            // ���͸��� ��ũ ���� �Լ�

    // Setter
	// FT ���� ���� ������ �ѹ��� �����ϴ� �Լ�
    void setAll_FT_cal(
        const std::array<float, 3>& biasedForce,
        const std::array<float, 3>& biasedTorque,
        const std::array<float, 3>& forceBase,
        const std::array<float, 3>& torqueBase,
        const std::array<float, 3>& filteredForce,
        const std::array<float, 3>& filteredTorque) noexcept;

	void setSensorForce(const std::array<float, 3>& val) noexcept;      // ���� �� ���� �Լ�
	void setSensorTorque(const std::array<float, 3>& val) noexcept;     // ���� ��ũ ���� �Լ�
	void setRawForce(const std::array<float, 3>& val) noexcept;         // �������� ���� ���� �� ���� �Լ�
	void setRawTorque(const std::array<float, 3>& val) noexcept;        // �������� ���� ���� ��ũ ���� �Լ�
	void setBiasForce(const std::array<float, 3>& val) noexcept;        // ���̾ �� ���� �Լ�
	void setBiasTorque(const std::array<float, 3>& val) noexcept;       // ���̾ ��ũ ���� �Լ�
	void setBiasSumForce(const std::array<float, 3>& val) noexcept;     // ���̾ �ջ� �� ���� �Լ�
	void setBiasSumTorque(const std::array<float, 3>& val) noexcept;    // ���̾ �ջ� ��ũ ���� �Լ�
	void setBiasedForce(const std::array<float, 3>& val) noexcept;      // ���̾ ����� �� ���� �Լ�
	void setBiasedTorque(const std::array<float, 3>& val) noexcept;     // ���̾ ����� ��ũ ���� �Լ�  
	void setForceBase(const std::array<float, 3>& val) noexcept;        // �κ� ���̽� ��ǥ��� ��ȯ�� �� ���� �Լ�
	void setTorqueBase(const std::array<float, 3>& val) noexcept;       // �κ� ���̽� ��ǥ��� ��ȯ�� ��ũ ���� �Լ�
	void setFilteredForce(const std::array<float, 3>& val) noexcept;    // ���͸��� �� ���� �Լ�
	void setFilteredTorque(const std::array<float, 3>& val) noexcept;   // ���͸��� ��ũ ���� �Լ�

    // ������ �ʱ�ȭ �Լ�
    void clearAll() noexcept;

	// ���̾ �ջ� �� ��� ���� �Լ�
	void addBiasSumForce(const std::array<float, 3>& val) noexcept;     // ���̾ �ջ� �� �߰� �Լ�
	void addBiasSumTorque(const std::array<float, 3>& val) noexcept;    // ���̾ �ջ� ��ũ �߰� �Լ�
	void resetBiasSum() noexcept;                                       // ���̾ �ջ� �ʱ�ȭ �Լ�
	void updateBiasAverage() noexcept;									// ���̾ ��� ������Ʈ �Լ�

	// ������ ������Ʈ �Լ�
    Snapshot updateSnapshot() noexcept;
	// �������� ���� �Լ�
    Snapshot getSnapshot() const noexcept;

private:
    mutable std::mutex mtx_;

	// FT ���� ������ ���� ����
	std::array<float, 3> sensorForce_{ 0.0f, 0.0f, 0.0f };				// ���� ��
	std::array<float, 3> sensorTorque_{ 0.0f, 0.0f, 0.0f };				// ���� ��ũ
	std::array<float, 3> rawForce_{ 0.0f, 0.0f, 0.0f };					// �������� ���� ���� ��
	std::array<float, 3> rawTorque_{ 0.0f, 0.0f, 0.0f };				// �������� ���� ���� ��ũ
	std::array<float, 3> biasForce_{ 0.0f, 0.0f, 0.0f };				// ���̾ ��
	std::array<float, 3> biasTorque_{ 0.0f, 0.0f, 0.0f };				// ���̾ ��ũ
	std::array<float, 3> biasSumForce_{ 0.0f, 0.0f, 0.0f };				// ���̾ �ջ� ��
	std::array<float, 3> biasSumTorque_{ 0.0f, 0.0f, 0.0f };			// ���̾ �ջ� ��ũ
	std::array<float, 3> biasedForce_{ 0.0f, 0.0f, 0.0f };				// ���̾ ����� ��
	std::array<float, 3> biasedTorque_{ 0.0f, 0.0f, 0.0f };				// ���̾ ����� ��ũ
	std::array<float, 3> forceBase_{ 0.0f, 0.0f, 0.0f };				// �κ� ���̽� ��ǥ��� ��ȯ�� ��
	std::array<float, 3> torqueBase_{ 0.0f, 0.0f, 0.0f };				// �κ� ���̽� ��ǥ��� ��ȯ�� ��ũ
	std::array<float, 3> filteredForce_{ 0.0f, 0.0f, 0.0f };			// ���͸��� ��
	std::array<float, 3> filteredTorque_{ 0.0f, 0.0f, 0.0f };			// ���͸��� ��ũ
	Snapshot m_snapshot_;												// ������ ������
	size_t biasCount_ = 1000;											// ���̾ ��� Ƚ�� ī����
};
