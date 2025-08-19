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

// [���] : PC -> ��ʾз� ���ַ�����
// [�Է�] : ��ʾз� ���ַ����� -> PC

/// \brief NI-DAQ�� �̿��� Chamber/Spindle ���� �����  
class AirControl {
public:
	struct Config
	{
		std::string deviceName = "Dev1";        // NI-DAQ ��ġ �̸�
		std::string aoTaskName = "AO_Task";     // AO �½�ũ �̸�
		std::string aiTaskName = "AI_Task";     // AI �½�ũ �̸�

		// ���� ä�� ��� (ao0, ao1 ��)
		std::vector<std::string> aoPhysicalChannels = { "ao0", "ao1" }; // Chamber, Spindle
		std::vector<std::string> aiPhysicalChannels = { "ai0", "ai2" }; // Chamber, Spindle �ǵ��

		// ���� ä�� �̸� ��� (�ڵ� ������ ����� �̸�)
		std::vector<std::string> aoChannelNames = { "ChamberChannel", "SpindleChannel" };
		std::vector<std::string> aiChannelNames = { "ChamberFeedback", "SpindleFeedback" };

		// �ϵ���� ���� ����
		double maxVoltage = 10.0;                // �ִ� ���� (V ����)
		double maxChamberPressureMPa = 0.4;      // �ִ� Chamber ���� (MPa ����)
		double maxSpindlePressureMPa = 0.6;      // �ִ� Spindle ���� (MPa ����)

		// �ϵ���� ����
		const double MFT_diameter_mm = 94.9;						// MFT ���� (mm)
		double MFT_radius_m = MFT_diameter_mm / 2.0 * 1e-3;		    // MFT ������ (m)					
		double A_m2 = M_PI * MFT_radius_m * MFT_radius_m;			// MFT �ܸ��� (m^2)

		double max_force_N = maxChamberPressureMPa * 1e6 * A_m2;    // �ִ� �� (N ����) => Chamber ���п� ���� �ִ� ��        
		double pid_limit = max_force_N * 1.2;


		// --- DAQ ���ø� �� �ݹ� ���� ---
		uInt64 sampleRateHz = 1000;
		int32 samplesPerCallback = 100; // �� ������ŭ ������ ���̸� �ݹ� ȣ��
	};

	// ������/�Ҹ���
	explicit AirControl(const Config& config = Config{});
	~AirControl() { releaseTasks(); }

	// MFT �ܸ��� getter �޼��� �߰�
	double getA_m2() const noexcept { return m_config.A_m2; }

	// ���� ��� ���� �½�ũ�� �ʱ�ȭ�ϰ� ����
	bool initTasks();

	// ���� ��� ���� �½�ũ�� �����ϰ� �ʱ�ȭ ���·� �ǵ���
	void releaseTasks() noexcept;

	// ���� �½�ũ�� �ʱ�ȭ�Ǿ����� ���� ��ȸ
	bool isInitialized() const noexcept;

	// [���] ���� ���� ������ �����Ͽ� �з°� ������ ������Ʈ
	void updateOutputs() noexcept;

	// Chamber ���� ���� �� �ϵ��� ����� �ִ� ��°� Ŭ����
	void setDesiredChamberPressure(double pMPa) noexcept {
		m_desiredChamber = std::clamp(pMPa, 0.0, m_config.maxChamberPressureMPa);
	}
	// Chamber ���� ��ȸ
	double desiredChamberPressure() const noexcept { return m_desiredChamber.load(); }

	// Spindle ���� ���� �� �ϵ��� ����� �ִ� ��°� Ŭ����
	void setDesiredSpindlePressure(double pMPa) noexcept {
		m_desiredSpindle = std::clamp(pMPa, 0.0, m_config.maxSpindlePressureMPa);
	}
	// Spindle ���� ��ȸ
	double desiredSpindlePressure() const noexcept { return m_desiredSpindle.load(); }

	// [���] Chamber & Spindle ���� �� ���� ��ȸ
	double sendChamberVoltage() const noexcept { return m_sendVoltChamber.load(); }
	double sendSpindleVoltage() const noexcept { return m_sendVoltSpindle.load(); }
	double sendChamberPressure() const noexcept { return m_sendPressChamber.load(); }
	double sendSpindlePressure() const noexcept { return m_sendPressSpindle.load(); }

	// [�Է�] Chamber & Spindle �ǵ�� ���� �� ���� ��ȸ
	double feedbackChamberVoltage() const noexcept { return m_feedbackVoltChamber.load(); }
	double feedbackSpindleVoltage() const noexcept { return m_feedbackVoltSpindle.load(); }
	double feedbackChamberPressure() const noexcept { return m_feedbackPressChamber.load(); }
	double feedbackSpindlePressure() const noexcept { return m_feedbackPressSpindle.load(); }


private:
	// AO �� AI �½�ũ�� �����ϰ� ä���� �߰��� �� ����
	bool setupAoTask();
	bool setupAiTask();
	bool checkDaqError(int32 error, const char* context) noexcept;
	static int32 CVICALLBACK EveryNCallback(TaskHandle taskHandle, int32 everyNsamplesEventType, uInt32 nSamples, void* callbackData);

	Config m_config;

	TaskHandle m_aoTask{ nullptr };                                                     // ���(AO) �½�ũ �ڵ� (����)
	TaskHandle m_aiTask{ nullptr };                                                     // �Է�(AI) �½�ũ �ڵ� (����)

	// --- ���� ---
	std::atomic<long long> m_totalSamplesRead{ 0 };                                     // �� ���� ���� ��
	std::chrono::steady_clock::time_point m_startTime;                                  // �½�ũ ���� �ð�
	std::atomic<double> m_desiredChamber{ 0.0 }, m_desiredSpindle{ 0.0 };               // ���� ��ǥġ (MPa ����)

	// ���(AO) �� ---
	std::atomic<double> m_sendVoltChamber{ 0.0 }, m_sendVoltSpindle{ 0.0 };             // �۽� ���а� (V ����)
	std::atomic<double> m_sendPressChamber{ 0.0 }, m_sendPressSpindle{ 0.0 };           // �۽� ���а� (MPa ����)

	// --- �Է�(AI) �� ---
	std::atomic<double> m_feedbackVoltChamber{ 0.0 }, m_feedbackVoltSpindle{ 0.0 };     // �ǵ�� ���а� (V ����)
	std::atomic<double> m_feedbackPressChamber{ 0.0 }, m_feedbackPressSpindle{ 0.0 };   // �ǵ�� ���а� (MPa ����)
};
