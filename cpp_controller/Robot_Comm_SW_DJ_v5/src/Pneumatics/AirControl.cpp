// AirControl.cpp  
#include "AirControl.h"  
#include <iostream>
#include <vector>
#include <cstring>  
#include <cstdio>

AirControl::AirControl(const Config& config)
    : m_config(config) // 멤버 이니셜라이저 리스트를 사용해 m_config를 초기화
{
}

bool AirControl::isInitialized() const noexcept
{
    // 태스크 핸들이 nullptr이 아니면 초기화된 것으로 간주
    return m_aoTask != nullptr && m_aiTask != nullptr;
}

bool AirControl::initTasks()
{

	if (!setupAoTask()) return false;                   // AO 태스크 설정
	if (!setupAiTask()) return false;                   // AI 태스크 설정
    // 모든 태스크가 성공적으로 시작되면 시작 시각 기록
    m_startTime = std::chrono::steady_clock::now();
    return true;
}

void AirControl::releaseTasks() noexcept
{
    // AO 태스크 해제
    if (m_aoTask) {
        DAQmxStopTask(m_aoTask);
        DAQmxClearTask(m_aoTask);
        m_aoTask = nullptr;
    }
    // AI 태스크 해제
    if (m_aiTask) {
        DAQmxStopTask(m_aiTask);
        DAQmxClearTask(m_aiTask);
        m_aiTask = nullptr;
    }
}

// =================================================================
// AO 태스크 설정 헬퍼 함수 구현
// // 태스크 생성 후 채널을 추가하고 시작합니다.
// =================================================================
bool AirControl::setupAoTask()
{
    // 1. 태스크 생성
    if (!checkDaqError(DAQmxCreateTask("AO_Task", &m_aoTask), "Create AO Task")) return false;

    // 설정된 채널 수만큼 반복하여 채널을 생성합니다.
    for (size_t i = 0; i < m_config.aoPhysicalChannels.size(); ++i) {
        std::string physicalChannel = m_config.deviceName + "/" + m_config.aoPhysicalChannels[i];
        if (!checkDaqError(DAQmxCreateAOVoltageChan(m_aoTask, physicalChannel.c_str(), m_config.aoChannelNames[i].c_str(), 0.0, m_config.maxVoltage, DAQmx_Val_Volts, nullptr), 
            ("Create " + m_config.aoChannelNames[i]).c_str())) return false;
    }

    if (!checkDaqError(DAQmxStartTask(m_aoTask), "Start AO Task")) return false;
    return true;
}

// =================================================================
//  AI 태스크 설정 헬퍼 함수 구현 (콜백 방식)
// // 태스크 생성 후 채널을 추가하고 샘플링 타이밍을 설정합니다.
// =================================================================
bool AirControl::setupAiTask()
{
    if (!checkDaqError(DAQmxCreateTask(m_config.aiTaskName.c_str(), &m_aiTask), "Create AI Task")) return false;

    for (size_t i = 0; i < m_config.aiPhysicalChannels.size(); ++i) {
        std::string physicalChannel = m_config.deviceName + "/" + m_config.aiPhysicalChannels[i];
        if (!checkDaqError(DAQmxCreateAIVoltageChan(m_aiTask, physicalChannel.c_str(), m_config.aiChannelNames[i].c_str(), DAQmx_Val_RSE, 0.0, m_config.maxVoltage, DAQmx_Val_Volts, nullptr), 
            ("Create " + m_config.aiChannelNames[i]).c_str())) return false;
    }

    // 샘플링 타이밍 설정
    const uInt64 bufferSize = m_config.sampleRateHz; // 1초 분량의 버퍼
    if (!checkDaqError(DAQmxCfgSampClkTiming(m_aiTask, "", static_cast<float64>(m_config.sampleRateHz), DAQmx_Val_Rising, DAQmx_Val_ContSamps, static_cast<uInt64>(bufferSize)), "Configure AI Sample Clock")) return false;

    // 콜백 함수 등록
    if (!checkDaqError(DAQmxRegisterEveryNSamplesEvent(m_aiTask, DAQmx_Val_Acquired_Into_Buffer, m_config.samplesPerCallback, 0, EveryNCallback, this), "Register AI Callback")) return false;

    if (!checkDaqError(DAQmxStartTask(m_aiTask), "Start AI Task")) return false;
    return true;
}

// =================================================================
// 에러 검사 헬퍼 함수 구현
// =================================================================
bool AirControl::checkDaqError(int32 error, const char* context) noexcept
{
    if (DAQmxFailed(error))
    {
        char errBuff[2048]{};
        DAQmxGetExtendedErrorInfo(errBuff, sizeof(errBuff));
        std::cerr << "DAQmx Error in " << context << ": " << errBuff << "\n";

        // 에러 발생 시 모든 태스크를 즉시 정리합니다.
        releaseTasks();
        return false;
    }
    return true;
}

void AirControl::updateOutputs() noexcept
{
    if (!isInitialized()) return;

    // 1) 목표 압력 읽고 전압 계산  
    double pChamber = m_desiredChamber.load();
    double pSpindle = m_desiredSpindle.load();

    // 각 레귤레이터의 최대 압력에 맞춰 전압을 개별적으로 계산
    double vChamber = (pChamber / m_config.maxChamberPressureMPa) * m_config.maxVoltage;
    double vSpindle = (pSpindle / m_config.maxSpindlePressureMPa) * m_config.maxVoltage;


    // 2) 전압 데이터를 배열에 담기 (채널 생성 순서: ao0, ao1)
    double data[2] = { vChamber, vSpindle };

    // 3) 단일 Write 함수 호출로 두 채널에 동시에 전압 출력
    int32 sampsWritten = 0;
    DAQmxWriteAnalogF64(
        m_aoTask,                   // 통합된 태스크 핸들
        1,                          // 채널 당 샘플 수
        TRUE,                       // 태스크 자동 시작
        5.0,                        // 타임아웃(초)
        DAQmx_Val_GroupByChannel,   // 데이터 레이아웃
        data,                       // 보낼 데이터 배열
        &sampsWritten,              // 실제 쓰여진 샘플 수 (반환)
        nullptr);

    // 4) 최종값 저장
    m_sendVoltChamber = vChamber;
    m_sendVoltSpindle = vSpindle;
    m_sendPressChamber = pChamber;
    m_sendPressSpindle = pSpindle;
}

// =================================================================
// 콜백 함수 구현
// =================================================================
int32 CVICALLBACK AirControl::EveryNCallback(TaskHandle taskHandle, int32 everyNsamplesEventType, uInt32 nSamples, void* callbackData)
{
    // 1. callbackData를 원래의 AirControl 객체 포인터로 변환합니다.
    AirControl* self = static_cast<AirControl*>(callbackData);

    // callbackData가 유효하지 않은 경우를 대비한 방어 코드
    if (self == nullptr) {
        return 0;
    }

    // 2. DAQmxReadAnalogF64를 호출하여 버퍼에 쌓인 데이터를 읽어옵니다.
    // 여기서는 nSamples 만큼 읽는 것이 일반적입니다.
    std::vector<double> readData(2 * nSamples);
    int32 sampsRead = 0;

    DAQmxReadAnalogF64(
        taskHandle,
        nSamples,                   // 콜백을 유발시킨 샘플 수 만큼 읽기
        10.0,                       // 타임아웃
        DAQmx_Val_GroupByChannel,
        readData.data(),
        readData.size(),
        &sampsRead,
        nullptr);

    // 3. 읽은 데이터 처리 (기존 updateInputs의 로직과 유사)
    if (sampsRead > 0)
    {
        // 여러 샘플을 읽었더라도, 가장 마지막 샘플을 대표값으로 사용하거나
        // 평균을 내서 사용할 수 있습니다. 여기서는 마지막 샘플을 사용합니다.
        double v_fb_chamber = readData[sampsRead - 1]; // Chamber 채널의 마지막 샘플
        double v_fb_spindle = readData[sampsRead + sampsRead - 1]; // Spindle 채널의 마지막 샘플

        double p_fb_chamber = (v_fb_chamber / self->m_config.maxVoltage) * self->m_config.maxChamberPressureMPa;
        self->m_feedbackVoltChamber = v_fb_chamber; // std::atomic 이므로 스레드 안전
        self->m_feedbackPressChamber = p_fb_chamber;

        double p_fb_spindle = (v_fb_spindle / self->m_config.maxVoltage) * self->m_config.maxSpindlePressureMPa;

        self->m_feedbackVoltSpindle = v_fb_spindle;
        self->m_feedbackPressSpindle = p_fb_spindle;

        self->m_totalSamplesRead += sampsRead;
    }

    return 0; // 0을 반환하여 계속 진행
}
