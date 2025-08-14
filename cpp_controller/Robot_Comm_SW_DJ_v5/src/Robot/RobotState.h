// RobotState.h
#pragma once

#include <array>
#include <atomic>
#include <mutex>
#include <functional>

// 네임스페이스 RobotComm 정의
namespace RobotComm {	
	inline constexpr char VERSION[] = "v1.0";	// 버전 정보
	inline constexpr float PERIOD_S = 0.001f;	// 로봇 주기 [s]
	inline constexpr int LOSS = -1;				// 로봇 데이터 손실 값 설정
}

// 로봇 상태 정보를 관리하는 클래스
class RobotState {
public:
	// 로봇 상태 스냅샷 구조체
	struct Snapshot {						
        std::array<float, 6> tcpPos{};
		std::array<float, 6> flangePos{};
        std::array<float, 6> jointAng{};
        std::array<float, 6> tcpVel{};
		std::array<float, 6> flangeVel{};
        std::array<float, 6> jointVel{};
		std::array<float, 3> desVel{};
    };

    // 생성자
    RobotState() noexcept;

    // 상태 초기화
    void reset() noexcept;

    // 스냅샷 관련 메서드
	Snapshot updateSnapshot() noexcept;											// 현재 상태를 스냅샷으로 업데이트하고 반환
	Snapshot getSnapshot() const noexcept;										// 현재 상태 스냅샷을 반환 (읽기 전용)

    // 락 관련 메서드
	void withLock(std::function<void(const RobotState&)> cb) const noexcept;	// 뮤텍스 보호된 콜백 실행

    // 위치 저장 메서드
	void saveInitialtcpPosition(const float* data, size_t sz) noexcept;			// 현재 TCP 위치를 초기 위치로 저장
	void saveInitialflangePosition(const float* data, size_t sz) noexcept;		// 현재 Flange 위치를 초기 위치로 저장
	void saveInitialjointPosition(const float* data, size_t sz) noexcept;		// 현재 관절 각도를 초기 위치로 저장

	// 릴랙세이션 포지션 저장 메서드
	void saveRelaxationtcpPosition(const float* tcpPos, size_t sz) noexcept;	// 현재 TCP 위치를 릴랙세이션 포지션으로 저장
	void saveRelaxationflangePosition(const float* flangePos, size_t sz)		// 현재 Flange 위치를 릴랙세이션 포지션으로 저장
		noexcept;
	void saveRelaxationjointPosition(const float* jointAng, size_t sz)			// 현재 관절 각도를 릴랙세이션 각도로 저장
		noexcept;
		
    // 데이터 업데이트 메서드
	void updateAllRawData(const float* tP, const float* fP, const float* jA,	// 모든 원시 데이터 업데이트
        const float* tV, const float* fV, const float* jV, const float* desV,
		size_t sz) noexcept; 
	void updatetcpPosRaw(const float* d, size_t sz) noexcept;					// TCP 원시 위치 업데이트    
	void updateflangePosRaw(const float* d, size_t sz) noexcept;				// Flange 원시 위치 업데이트
	void updateJointAngRaw(const float* d, size_t sz) noexcept;					// 관절 각도 원시 데이터 업데이트
	void updatetcpVelRaw(const float* d, size_t sz) noexcept;					// TCP 속도 원시 데이터 업데이트
	void updateflangeVelRaw(const float* d, size_t sz) noexcept;				// Flange 속도 원시 데이터 업데이트
	void updateJointVelRaw(const float* d, size_t sz) noexcept;					// 관절 속도 원시 데이터 업데이트

	void updatetcpPosition(const std::array<float, 6>& np)noexcept;				// TCP 위치 업데이트
	void updateflangePosition(const std::array<float, 6>& np) noexcept;			// Flange 위치 업데이트
	void updateJointAngles(const std::array<float, 6>& na) noexcept;			// 관절 각도 업데이트
	void updatetcpVelocity(const std::array<float, 6>& nv) noexcept;			// TCP 속도 업데이트
	void updateflangeVelocity(const std::array<float, 6>& nv) noexcept;			// Flange 속도 업데이트
	void updateJointVelocity(const std::array<float, 6>& nv) noexcept;			// 관절 속도 업데이트

    // 배열 반환 메서드
	std::array<float, 6> gettcpPositionArray() const noexcept;					// TCP 위치 배열 반환
	std::array<float, 6> getflangePositionArray() const noexcept;				// Flange 위치 배열 반환
	std::array<float, 6> getJointAngleArray() const noexcept;					// 관절 각도 배열 반환
	std::array<float, 6> gettcpVelocityArray() const noexcept;					// TCP 속도 배열 반환
	std::array<float, 6> getflangeVelocityArray() const noexcept;				// Flange 속도 배열 반환
	std::array<float, 6> getJointVelocityArray() const noexcept;				// 관절 속도 배열 반환
	std::array<float, 6> getInitialtcpPositionArray() const noexcept;			// 초기 TCP 위치 배열 반환
	std::array<float, 6> getInitialflangePositionArray() const noexcept;		// 초기 위치 배열 반환
	std::array<float, 6> getInitialJointAngleArray() const noexcept;			// 초기 관절 각도 배열 반환
	std::array<float, 6> getRelaxationtcpPositionArray() const noexcept;		// 릴랙세이션 TCP 포지션 배열 반환
	std::array<float, 6> getRelaxationflangePositionArray() const noexcept;		// 릴랙세이션 Flange 포지션 배열 반환
	std::array<float, 6> getRelaxationAngleArray() const noexcept;				// 릴랙세이션 각도 배열 반환

	// 개별 값 접근 메서드
	float gettcpPosition(size_t i) const noexcept;								// 특정 인덱스의 TCP 위치 가져오기
	float getflangePosition(size_t i) const noexcept;							// 특정 인덱스의 Flange 위치 가져오기
	float getJointAngle(size_t i) const noexcept;								// 특정 인덱스의 관절 각도 가져오기
	float gettcpVelocity(size_t i) const noexcept;								// 특정 인덱스의 TCP 속도 가져오기
	float getflangeVelocity(size_t i) const noexcept;							// 특정 인덱스의 Flange 속도 가져오기
	float getJointVelocity(size_t i) const noexcept;							// 특정 인덱스의 관절 속도 가져오기

    // 데이터 복사 메서드
	void copytcpPositionData(float* out, size_t size) const noexcept;			// TCP 위치 데이터 복사
	void copyflangePositionData(float* out, size_t size) const noexcept;		// Flange 위치 데이터 복사
	void copyJointAngleData(float* out, size_t size) const noexcept;			// 관절 각도 데이터 복사
	void copytcpVelocityData(float* out, size_t size) const noexcept;			// TCP 속도 데이터 복사
	void copyflangeVelocityData(float* out, size_t size) const noexcept;		// Flange 속도 데이터 복사
	void copyJointVelocityData(float* out, size_t size) const noexcept;			// 관절 속도 데이터 복사
	void copyInitialtcpPositionData(float* out, size_t size) const noexcept;	// 초기 TCP 위치 데이터 복사
	void copyInitialflangePositionData(float* out, size_t size) const noexcept;	// 초기 위치 데이터 복사
	void copyInitialJointAngleData(float* out, size_t size) const noexcept;		// 초기 관절 각도 데이터 복사
	void copyRelaxationtcpPositionData(float* out, size_t size) noexcept;		// 릴랙세이션 TCP 포지션 데이터 복사
	void copyRelaxationflangePositionData(float* out, size_t size) noexcept;	// 릴랙세이션 Flange 포지션 데이터 복사
	void copyRelaxationAngleData(float* out, size_t size) noexcept;				// 릴랙세이션 각도 데이터 복사

private:
    mutable std::mutex m_mutex;
	std::array<float, 6> m_tcpPos{};											// TCP 위치				[mm, deg]
	std::array<float, 6> m_flangePos{};											// 플랜지 위치			[mm, deg]
	std::array<float, 6> m_jointAng{};											// 관절 각도			[deg]
	std::array<float, 6> m_tcpVel{};											// TCP 속도				[mm/s, deg/s]
	std::array<float, 6> m_flangeVel{};											// Flange 속도			[mm/s, deg/s]
	std::array<float, 6> m_jointVel{};											// 관절 속도			[deg/s]
	std::array<float, 3> m_desVel{};											// 목표 속도			[mm/s] (x, y, z)
	std::array<float, 6> m_initialtcpPos{};										// 초기 TCP 위치		[mm, deg]
	std::array<float, 6> m_initialflangePos{};									// 초기 Flange 위치		[mm, deg]
	std::array<float, 6> m_initialjointAng{};									// 초기 관절 각도		[deg]
	std::array<float, 6> m_relaxationtcpPos{};									// TCP 릴랙세이션 포지션[mm, deg]
	std::array<float, 6> m_relaxationflangePos{};								// Flange 릴랙세이션 포지션[mm, deg]
	std::array<float, 6> m_relaxationjoint{};									// 릴랙세이션 각도		[deg]
	Snapshot m_snapshot{};														// 현재 상태 스냅샷
};