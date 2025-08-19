// AirControl.h  
#pragma once  
#include <string>
#include <vector>
#include <atomic>  
#include <chrono>  
#include <algorithm>    // std::clamp  
#include <math.h>
#include "NIDAQmx.h"    // NI-DAQmx  

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// [출력] : PC -> 비례압력 레귤레이터
// [입력] : 비례압력 레귤레이터 -> PC

/// \brief NI-DAQ를 이용한 Chamber/Spindle 공압 제어기  
class AirControl {
public:
	struct Config
	{
		std::string deviceName = "Dev1";        // NI-DAQ 장치 이름
		std::string aoTaskName = "AO_Task";     // AO 태스크 이름
		std::string aiTaskName = "AI_Task";     // AI 태스크 이름

		// 물리 채널 목록 (ao0, ao1 등)
		std::vector<std::string> aoPhysicalChannels = { "ao0", "ao1" }; // Chamber, Spindle
		std::vector<std::string> aiPhysicalChannels = { "ai0", "ai2" }; // Chamber, Spindle 피드백

		// 가상 채널 이름 목록 (코드 내에서 사용할 이름)
		std::vector<std::string> aoChannelNames = { "ChamberChannel", "SpindleChannel" };
		std::vector<std::string> aiChannelNames = { "ChamberFeedback", "SpindleFeedback" };

		// 하드웨어 제약 조건
		double maxVoltage = 10.0;                // 최대 전압 (V 단위)
		double maxChamberPressureMPa = 0.4;      // 최대 Chamber 공압 (MPa 단위)
		double maxSpindlePressureMPa = 0.6;      // 최대 Spindle 공압 (MPa 단위)

		// 하드웨어 설정
		const double MFT_diameter_mm = 94.9;						// MFT 직경 (mm)
		double MFT_radius_m = MFT_diameter_mm / 2.0 * 1e-3;		    // MFT 반지름 (m)					
		double A_m2 = M_PI * MFT_radius_m * MFT_radius_m;			// MFT 단면적 (m^2)

		double max_force_N = maxChamberPressureMPa * 1e6 * A_m2;    // 최대 힘 (N 단위) => Chamber 공압에 따른 최대 힘        
		double pid_limit = max_force_N * 1.2;


		// --- DAQ 샘플링 및 콜백 설정 ---
		uInt64 sampleRateHz = 1000;
		int32 samplesPerCallback = 100; // 이 개수만큼 샘플이 모이면 콜백 호출
	};

	// 생성자/소멸자
	explicit AirControl(const Config& config = Config{});
	~AirControl() { releaseTasks(); }

	// MFT 단면적 getter 메서드 추가
	double getA_m2() const noexcept { return m_config.A_m2; }

	// 현재 사용 중인 태스크를 초기화하고 시작
	bool initTasks();

	// 현재 사용 중인 태스크를 해제하고 초기화 상태로 되돌림
	void releaseTasks() noexcept;

	// 현재 태스크가 초기화되었는지 여부 조회
	bool isInitialized() const noexcept;

	// [출력] 공압 제어 루프를 실행하여 압력과 전압을 업데이트
	void updateOutputs() noexcept;

	// Chamber 공압 설정 및 하드웨어를 고려한 최대 출력값 클램핑
	void setDesiredChamberPressure(double pMPa) noexcept {
		m_desiredChamber = std::clamp(pMPa, 0.0, m_config.maxChamberPressureMPa);
	}
	// Chamber 공압 조회
	double desiredChamberPressure() const noexcept { return m_desiredChamber.load(); }

	// Spindle 공압 설정 및 하드웨어를 고려한 최대 출력값 클램핑
	void setDesiredSpindlePressure(double pMPa) noexcept {
		m_desiredSpindle = std::clamp(pMPa, 0.0, m_config.maxSpindlePressureMPa);
	}
	// Spindle 공압 조회
	double desiredSpindlePressure() const noexcept { return m_desiredSpindle.load(); }

	// [출력] Chamber & Spindle 전압 및 공압 조회
	double sendChamberVoltage() const noexcept { return m_sendVoltChamber.load(); }
	double sendSpindleVoltage() const noexcept { return m_sendVoltSpindle.load(); }
	double sendChamberPressure() const noexcept { return m_sendPressChamber.load(); }
	double sendSpindlePressure() const noexcept { return m_sendPressSpindle.load(); }

	// [입력] Chamber & Spindle 피드백 전압 및 공압 조회
	double feedbackChamberVoltage() const noexcept { return m_feedbackVoltChamber.load(); }
	double feedbackSpindleVoltage() const noexcept { return m_feedbackVoltSpindle.load(); }
	double feedbackChamberPressure() const noexcept { return m_feedbackPressChamber.load(); }
	double feedbackSpindlePressure() const noexcept { return m_feedbackPressSpindle.load(); }


private:
	// AO 및 AI 태스크를 생성하고 채널을 추가한 뒤 시작
	bool setupAoTask();
	bool setupAiTask();
	bool checkDaqError(int32 error, const char* context) noexcept;
	static int32 CVICALLBACK EveryNCallback(TaskHandle taskHandle, int32 everyNsamplesEventType, uInt32 nSamples, void* callbackData);

	Config m_config;

	TaskHandle m_aoTask{ nullptr };                                                     // 출력(AO) 태스크 핸들 (통합)
	TaskHandle m_aiTask{ nullptr };                                                     // 입력(AI) 태스크 핸들 (통합)

	// --- 공통 ---
	std::atomic<long long> m_totalSamplesRead{ 0 };                                     // 총 읽은 샘플 수
	std::chrono::steady_clock::time_point m_startTime;                                  // 태스크 시작 시각
	std::atomic<double> m_desiredChamber{ 0.0 }, m_desiredSpindle{ 0.0 };               // 공압 목표치 (MPa 단위)

	// 출력(AO) 값 ---
	std::atomic<double> m_sendVoltChamber{ 0.0 }, m_sendVoltSpindle{ 0.0 };             // 송신 전압값 (V 단위)
	std::atomic<double> m_sendPressChamber{ 0.0 }, m_sendPressSpindle{ 0.0 };           // 송신 공압값 (MPa 단위)

	// --- 입력(AI) 값 ---
	std::atomic<double> m_feedbackVoltChamber{ 0.0 }, m_feedbackVoltSpindle{ 0.0 };     // 피드백 전압값 (V 단위)
	std::atomic<double> m_feedbackPressChamber{ 0.0 }, m_feedbackPressSpindle{ 0.0 };   // 피드백 공압값 (MPa 단위)
};
