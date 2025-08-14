#pragma once

#include <array>
#include <cstddef>
#include <mutex>

class FTState {
public:
	// 스냅샷 구조체 정의
    struct Snapshot {
        std::array<float, 3> biasedForce;
		std::array<float, 3> biasedTorque;
        std::array<float, 3> forceBase;
		std::array<float, 3> torqueBase;
        std::array<float, 3> filteredForce;
        std::array<float, 3> filteredTorque;
    };

    // 생성자
    FTState() = default;

    // Getter
	std::array<float, 3> getSensorForce() const noexcept;               // 센서 힘 수신 함수
	std::array<float, 3> getSensorTorque() const noexcept;              // 센서 토크 수신 함수
	std::array<float, 3> getRawForce() const noexcept;                  // 센서에서 읽은 원시 힘 수신 함수
	std::array<float, 3> getRawTorque() const noexcept;                 // 센서에서 읽은 원시 토크 수신 함수
	std::array<float, 3> getBiasForce() const noexcept;                 // 바이어스 힘 수신 함수
	std::array<float, 3> getBiasTorque() const noexcept;                // 바이어스 토크 수신 함수
	std::array<float, 3> getBiasSumForce() const noexcept;              // 바이어스 합산 힘 수신 함수
	std::array<float, 3> getBiasSumTorque() const noexcept;             // 바이어스 합산 토크 수신 함수
	std::array<float, 3> getBiasedForce() const noexcept;               // 바이어스 적용된 힘 수신 함수
	std::array<float, 3> getBiasedTorque() const noexcept;              // 바이어스 적용된 토크 수신 함수
	std::array<float, 3> getForceBase() const noexcept;                 // 로봇 베이스 좌표계로 변환된 힘 수신 함수
	std::array<float, 3> getTorqueBase() const noexcept;                // 로봇 베이스 좌표계로 변환된 토크 수신 함수
	std::array<float, 3> getFilteredForce() const noexcept;             // 필터링된 힘 수신 함수
	std::array<float, 3> getFilteredTorque() const noexcept;            // 필터링된 토크 수신 함수

    // Setter
	// FT 센서 관련 데이터 한번에 설정하는 함수
    void setAll_FT_cal(
        const std::array<float, 3>& biasedForce,
        const std::array<float, 3>& biasedTorque,
        const std::array<float, 3>& forceBase,
        const std::array<float, 3>& torqueBase,
        const std::array<float, 3>& filteredForce,
        const std::array<float, 3>& filteredTorque) noexcept;

	void setSensorForce(const std::array<float, 3>& val) noexcept;      // 센서 힘 설정 함수
	void setSensorTorque(const std::array<float, 3>& val) noexcept;     // 센서 토크 설정 함수
	void setRawForce(const std::array<float, 3>& val) noexcept;         // 센서에서 읽은 원시 힘 설정 함수
	void setRawTorque(const std::array<float, 3>& val) noexcept;        // 센서에서 읽은 원시 토크 설정 함수
	void setBiasForce(const std::array<float, 3>& val) noexcept;        // 바이어스 힘 설정 함수
	void setBiasTorque(const std::array<float, 3>& val) noexcept;       // 바이어스 토크 설정 함수
	void setBiasSumForce(const std::array<float, 3>& val) noexcept;     // 바이어스 합산 힘 설정 함수
	void setBiasSumTorque(const std::array<float, 3>& val) noexcept;    // 바이어스 합산 토크 설정 함수
	void setBiasedForce(const std::array<float, 3>& val) noexcept;      // 바이어스 적용된 힘 설정 함수
	void setBiasedTorque(const std::array<float, 3>& val) noexcept;     // 바이어스 적용된 토크 설정 함수  
	void setForceBase(const std::array<float, 3>& val) noexcept;        // 로봇 베이스 좌표계로 변환된 힘 설정 함수
	void setTorqueBase(const std::array<float, 3>& val) noexcept;       // 로봇 베이스 좌표계로 변환된 토크 설정 함수
	void setFilteredForce(const std::array<float, 3>& val) noexcept;    // 필터링된 힘 설정 함수
	void setFilteredTorque(const std::array<float, 3>& val) noexcept;   // 필터링된 토크 설정 함수

    // 데이터 초기화 함수
    void clearAll() noexcept;

	// 바이어스 합산 및 계산 관련 함수
	void addBiasSumForce(const std::array<float, 3>& val) noexcept;     // 바이어스 합산 힘 추가 함수
	void addBiasSumTorque(const std::array<float, 3>& val) noexcept;    // 바이어스 합산 토크 추가 함수
	void resetBiasSum() noexcept;                                       // 바이어스 합산 초기화 함수
	void updateBiasAverage() noexcept;									// 바이어스 평균 업데이트 함수

	// 스냅샷 업데이트 함수
    Snapshot updateSnapshot() noexcept;
	// 스냅샷을 수신 함수
    Snapshot getSnapshot() const noexcept;

private:
    mutable std::mutex mtx_;

	// FT 센서 데이터 관련 변수
	std::array<float, 3> sensorForce_{ 0.0f, 0.0f, 0.0f };				// 센서 힘
	std::array<float, 3> sensorTorque_{ 0.0f, 0.0f, 0.0f };				// 센서 토크
	std::array<float, 3> rawForce_{ 0.0f, 0.0f, 0.0f };					// 센서에서 읽은 원시 힘
	std::array<float, 3> rawTorque_{ 0.0f, 0.0f, 0.0f };				// 센서에서 읽은 원시 토크
	std::array<float, 3> biasForce_{ 0.0f, 0.0f, 0.0f };				// 바이어스 힘
	std::array<float, 3> biasTorque_{ 0.0f, 0.0f, 0.0f };				// 바이어스 토크
	std::array<float, 3> biasSumForce_{ 0.0f, 0.0f, 0.0f };				// 바이어스 합산 힘
	std::array<float, 3> biasSumTorque_{ 0.0f, 0.0f, 0.0f };			// 바이어스 합산 토크
	std::array<float, 3> biasedForce_{ 0.0f, 0.0f, 0.0f };				// 바이어스 적용된 힘
	std::array<float, 3> biasedTorque_{ 0.0f, 0.0f, 0.0f };				// 바이어스 적용된 토크
	std::array<float, 3> forceBase_{ 0.0f, 0.0f, 0.0f };				// 로봇 베이스 좌표계로 변환된 힘
	std::array<float, 3> torqueBase_{ 0.0f, 0.0f, 0.0f };				// 로봇 베이스 좌표계로 변환된 토크
	std::array<float, 3> filteredForce_{ 0.0f, 0.0f, 0.0f };			// 필터링된 힘
	std::array<float, 3> filteredTorque_{ 0.0f, 0.0f, 0.0f };			// 필터링된 토크
	Snapshot m_snapshot_;												// 스냅샷 데이터
	size_t biasCount_ = 1000;											// 바이어스 계산 횟수 카운터
};
