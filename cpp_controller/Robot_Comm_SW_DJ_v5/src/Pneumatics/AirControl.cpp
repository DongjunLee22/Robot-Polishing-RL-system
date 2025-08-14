// AirControl.cpp  
#include "AirControl.h"  
#include <iostream>
#include <vector>
#include <cstring>  
#include <cstdio>

AirControl::AirControl(const Config& config)
    : m_config(config) // ��� �̴ϼȶ����� ����Ʈ�� ����� m_config�� �ʱ�ȭ
{
}

bool AirControl::isInitialized() const noexcept
{
    // �½�ũ �ڵ��� nullptr�� �ƴϸ� �ʱ�ȭ�� ������ ����
    return m_aoTask != nullptr && m_aiTask != nullptr;
}

bool AirControl::initTasks()
{

	if (!setupAoTask()) return false;                   // AO �½�ũ ����
	if (!setupAiTask()) return false;                   // AI �½�ũ ����
    // ��� �½�ũ�� ���������� ���۵Ǹ� ���� �ð� ���
    m_startTime = std::chrono::steady_clock::now();
    return true;
}

void AirControl::releaseTasks() noexcept
{
    // AO �½�ũ ����
    if (m_aoTask) {
        DAQmxStopTask(m_aoTask);
        DAQmxClearTask(m_aoTask);
        m_aoTask = nullptr;
    }
    // AI �½�ũ ����
    if (m_aiTask) {
        DAQmxStopTask(m_aiTask);
        DAQmxClearTask(m_aiTask);
        m_aiTask = nullptr;
    }
}

// =================================================================
// AO �½�ũ ���� ���� �Լ� ����
// // �½�ũ ���� �� ä���� �߰��ϰ� �����մϴ�.
// =================================================================
bool AirControl::setupAoTask()
{
    // 1. �½�ũ ����
    if (!checkDaqError(DAQmxCreateTask("AO_Task", &m_aoTask), "Create AO Task")) return false;

    // ������ ä�� ����ŭ �ݺ��Ͽ� ä���� �����մϴ�.
    for (size_t i = 0; i < m_config.aoPhysicalChannels.size(); ++i) {
        std::string physicalChannel = m_config.deviceName + "/" + m_config.aoPhysicalChannels[i];
        if (!checkDaqError(DAQmxCreateAOVoltageChan(m_aoTask, physicalChannel.c_str(), m_config.aoChannelNames[i].c_str(), 0.0, m_config.maxVoltage, DAQmx_Val_Volts, nullptr), 
            ("Create " + m_config.aoChannelNames[i]).c_str())) return false;
    }

    if (!checkDaqError(DAQmxStartTask(m_aoTask), "Start AO Task")) return false;
    return true;
}

// =================================================================
//  AI �½�ũ ���� ���� �Լ� ���� (�ݹ� ���)
// // �½�ũ ���� �� ä���� �߰��ϰ� ���ø� Ÿ�̹��� �����մϴ�.
// =================================================================
bool AirControl::setupAiTask()
{
    if (!checkDaqError(DAQmxCreateTask(m_config.aiTaskName.c_str(), &m_aiTask), "Create AI Task")) return false;

    for (size_t i = 0; i < m_config.aiPhysicalChannels.size(); ++i) {
        std::string physicalChannel = m_config.deviceName + "/" + m_config.aiPhysicalChannels[i];
        if (!checkDaqError(DAQmxCreateAIVoltageChan(m_aiTask, physicalChannel.c_str(), m_config.aiChannelNames[i].c_str(), DAQmx_Val_RSE, 0.0, m_config.maxVoltage, DAQmx_Val_Volts, nullptr), 
            ("Create " + m_config.aiChannelNames[i]).c_str())) return false;
    }

    // ���ø� Ÿ�̹� ����
    const uInt64 bufferSize = m_config.sampleRateHz; // 1�� �з��� ����
    if (!checkDaqError(DAQmxCfgSampClkTiming(m_aiTask, "", static_cast<float64>(m_config.sampleRateHz), DAQmx_Val_Rising, DAQmx_Val_ContSamps, static_cast<uInt64>(bufferSize)), "Configure AI Sample Clock")) return false;

    // �ݹ� �Լ� ���
    if (!checkDaqError(DAQmxRegisterEveryNSamplesEvent(m_aiTask, DAQmx_Val_Acquired_Into_Buffer, m_config.samplesPerCallback, 0, EveryNCallback, this), "Register AI Callback")) return false;

    if (!checkDaqError(DAQmxStartTask(m_aiTask), "Start AI Task")) return false;
    return true;
}

// =================================================================
// ���� �˻� ���� �Լ� ����
// =================================================================
bool AirControl::checkDaqError(int32 error, const char* context) noexcept
{
    if (DAQmxFailed(error))
    {
        char errBuff[2048]{};
        DAQmxGetExtendedErrorInfo(errBuff, sizeof(errBuff));
        std::cerr << "DAQmx Error in " << context << ": " << errBuff << "\n";

        // ���� �߻� �� ��� �½�ũ�� ��� �����մϴ�.
        releaseTasks();
        return false;
    }
    return true;
}

void AirControl::updateOutputs() noexcept
{
    if (!isInitialized()) return;

    // 1) ��ǥ �з� �а� ���� ���  
    double pChamber = m_desiredChamber.load();
    double pSpindle = m_desiredSpindle.load();

    // �� ���ַ������� �ִ� �з¿� ���� ������ ���������� ���
    double vChamber = (pChamber / m_config.maxChamberPressureMPa) * m_config.maxVoltage;
    double vSpindle = (pSpindle / m_config.maxSpindlePressureMPa) * m_config.maxVoltage;


    // 2) ���� �����͸� �迭�� ��� (ä�� ���� ����: ao0, ao1)
    double data[2] = { vChamber, vSpindle };

    // 3) ���� Write �Լ� ȣ��� �� ä�ο� ���ÿ� ���� ���
    int32 sampsWritten = 0;
    DAQmxWriteAnalogF64(
        m_aoTask,                   // ���յ� �½�ũ �ڵ�
        1,                          // ä�� �� ���� ��
        TRUE,                       // �½�ũ �ڵ� ����
        5.0,                        // Ÿ�Ӿƿ�(��)
        DAQmx_Val_GroupByChannel,   // ������ ���̾ƿ�
        data,                       // ���� ������ �迭
        &sampsWritten,              // ���� ������ ���� �� (��ȯ)
        nullptr);

    // 4) ������ ����
    m_sendVoltChamber = vChamber;
    m_sendVoltSpindle = vSpindle;
    m_sendPressChamber = pChamber;
    m_sendPressSpindle = pSpindle;
}

// =================================================================
// �ݹ� �Լ� ����
// =================================================================
int32 CVICALLBACK AirControl::EveryNCallback(TaskHandle taskHandle, int32 everyNsamplesEventType, uInt32 nSamples, void* callbackData)
{
    // 1. callbackData�� ������ AirControl ��ü �����ͷ� ��ȯ�մϴ�.
    AirControl* self = static_cast<AirControl*>(callbackData);

    // callbackData�� ��ȿ���� ���� ��츦 ����� ��� �ڵ�
    if (self == nullptr) {
        return 0;
    }

    // 2. DAQmxReadAnalogF64�� ȣ���Ͽ� ���ۿ� ���� �����͸� �о�ɴϴ�.
    // ���⼭�� nSamples ��ŭ �д� ���� �Ϲ����Դϴ�.
    std::vector<double> readData(2 * nSamples);
    int32 sampsRead = 0;

    DAQmxReadAnalogF64(
        taskHandle,
        nSamples,                   // �ݹ��� ���߽�Ų ���� �� ��ŭ �б�
        10.0,                       // Ÿ�Ӿƿ�
        DAQmx_Val_GroupByChannel,
        readData.data(),
        readData.size(),
        &sampsRead,
        nullptr);

    // 3. ���� ������ ó�� (���� updateInputs�� ������ ����)
    if (sampsRead > 0)
    {
        // ���� ������ �о�����, ���� ������ ������ ��ǥ������ ����ϰų�
        // ����� ���� ����� �� �ֽ��ϴ�. ���⼭�� ������ ������ ����մϴ�.
        double v_fb_chamber = readData[sampsRead - 1]; // Chamber ä���� ������ ����
        double v_fb_spindle = readData[sampsRead + sampsRead - 1]; // Spindle ä���� ������ ����

        double p_fb_chamber = (v_fb_chamber / self->m_config.maxVoltage) * self->m_config.maxChamberPressureMPa;
        self->m_feedbackVoltChamber = v_fb_chamber; // std::atomic �̹Ƿ� ������ ����
        self->m_feedbackPressChamber = p_fb_chamber;

        double p_fb_spindle = (v_fb_spindle / self->m_config.maxVoltage) * self->m_config.maxSpindlePressureMPa;

        self->m_feedbackVoltSpindle = v_fb_spindle;
        self->m_feedbackPressSpindle = p_fb_spindle;

        self->m_totalSamplesRead += sampsRead;
    }

    return 0; // 0�� ��ȯ�Ͽ� ��� ����
}
