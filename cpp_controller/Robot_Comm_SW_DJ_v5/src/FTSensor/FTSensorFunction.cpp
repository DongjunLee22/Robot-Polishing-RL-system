#include "FTSensorFunction.h"
#include <cstring>      // std::memset ��
#include <algorithm>    // std::clamp

// --- ������/�Ҹ��� ---
FTSensorFunction::FTSensorFunction() {
    // �ʿ�� �ʱ�ȭ
}
FTSensorFunction::~FTSensorFunction() {
    Release();  // �Ҹ� �� �ڿ� ����
}

// =====================================
// FT ���� ���� �� �ʱ�ȭ �Լ�
HRESULT FTSensorFunction::SelectDevice(BOOL fUserSelect) {
    std::lock_guard<std::mutex> lock(mtx_);

    // FT ���� ���� �ڵ�
    if (!fUserSelect) return E_FAIL;

	IVciDeviceManager*  pDevMgr = nullptr;      // ����̽� �Ŵ���
	IVciEnumDevice*     pEnum = nullptr;        // enumerator �ڵ�
	VCIDEVICEINFO       sInfo;                  // ����̽� ����

	// ����̽� �Ŵ��� �ʱ�ȭ
    hResult_ = VciGetDeviceManager(&pDevMgr);
    if (hResult_ == VCI_OK) hResult_ = pDevMgr->EnumDevices(&pEnum);
    if (hResult_ == VCI_OK) hResult_ = pEnum->Next(1, &sInfo, NULL);
    if (pEnum) { pEnum->Release(); pEnum = nullptr; }

	// ����̽� ���� Ȯ��
    if (hResult_ == VCI_OK) {
        IVciDevice* pDevice;
        hResult_ = pDevMgr->OpenDevice(sInfo.VciObjectId, &pDevice);
		
        // ����̽� ���� ���� ��
        if (hResult_ == VCI_OK) {
            hResult_ = pDevice->OpenComponent(CLSID_VCIBAL, IID_IBalObject, (void**)&pBalObject_);
            pDevice->Release();
        }
        lBusCtrlNo_ = 0;
    }
    return hResult_;
}

// =====================================
// FT ���� ��� �ʱ�ȭ �� ���� ����
HRESULT FTSensorFunction::CheckBalFeatures(LONG lCtrlNo) {
    std::lock_guard<std::mutex> lock(mtx_);

    if (!pBalObject_) return E_FAIL;
    BALFEATURES features = { 0 };
    hResult_ = pBalObject_->GetFeatures(&features);
    if (hResult_ != VCI_OK) return hResult_;

    if (lCtrlNo >= features.BusSocketCount)
        return VCI_E_UNEXPECTED;
    if (VCI_BUS_TYPE(features.BusSocketType[lCtrlNo]) != VCI_BUS_CAN)
        return VCI_E_UNEXPECTED;

    return hResult_;
}

// =====================================
// FT ���� ���� �ʱ�ȭ �� �޽��� ä�� ����
HRESULT FTSensorFunction::InitSocket(LONG lCtrlNo) {
    std::lock_guard<std::mutex> lock(mtx_);

    if (!pBalObject_) return E_FAIL;
    ICanSocket* pCanSocket = nullptr;
    hResult_ = pBalObject_->OpenSocket(lCtrlNo, IID_ICanSocket, (void**)&pCanSocket);
    if (hResult_ != VCI_OK) return hResult_;

    // Check CAN capabilities (���� ����)
    CANCAPABILITIES capabilities = { 0 };
    hResult_ = pCanSocket->GetCapabilities(&capabilities);
    if (hResult_ == VCI_OK && !(capabilities.dwFeatures & CAN_FEATURE_STDANDEXT))
        hResult_ = VCI_E_NOT_SUPPORTED;

    // �޽��� ä�� ���� �� �ʱ�ȭ
    if (hResult_ == VCI_OK) hResult_ = pCanSocket->CreateChannel(FALSE, &pCanChn_);
    pCanSocket->Release();

    // ä�� �ʱ�ȭ �� ����/������ ����
    if (hResult_ == VCI_OK)
    {
        UINT16 wRxFifoSize = 1024;        
        UINT16 wTxFifoSize = 128;

        hResult_ = pCanChn_->Initialize(wRxFifoSize, wTxFifoSize);
    }
	// ������ ������ ���� �� �̺�Ʈ ����
    if (hResult_ == VCI_OK) {
        UINT16 wRxThreshold = 1;
        UINT16 wTxThreshold = 1;

        hResult_ = pCanChn_->GetReader(&pReader_);
        if (hResult_ == VCI_OK) {
            pReader_->SetThreshold(wRxThreshold);
            hEventReader_ = CreateEvent(NULL, FALSE, FALSE, NULL);
            pReader_->AssignEvent(hEventReader_);
        }
        hResult_ = pCanChn_->GetWriter(&pWriter_);
        if (hResult_ == VCI_OK)
            pWriter_->SetThreshold(wTxThreshold);
    }
	// ä�� Ȱ��ȭ �� CAN ��Ʈ�ѷ� ����
    if (hResult_ == VCI_OK) hResult_ = pCanChn_->Activate();
    if (hResult_ == VCI_OK) hResult_ = pBalObject_->OpenSocket(lCtrlNo, IID_ICanControl, (void**)&pCanControl_);
	
    // CAN ��Ʈ�ѷ� �ʱ�ȭ
    if (hResult_ == VCI_OK) {
        CANINITLINE init = { CAN_OPMODE_STANDARD | CAN_OPMODE_EXTENDED | CAN_OPMODE_ERRFRAME, 0, CAN_BT0_1000KB, CAN_BT1_1000KB };
        hResult_ = pCanControl_->InitLine(&init);
        // CAN ��Ʈ�ѷ��� ���� �׼��� ���� ����
        if (hResult_ == VCI_OK) {
            // ǥ�� ������ ���� ����
            hResult_ = pCanControl_->SetAccFilter(CAN_FILTER_STD, CAN_ACC_CODE_ALL, CAN_ACC_MASK_ALL);
            // Ȯ�� ������ ���� ����
            if (hResult_ == VCI_OK)
                hResult_ = pCanControl_->SetAccFilter(CAN_FILTER_EXT, CAN_ACC_CODE_ALL, CAN_ACC_MASK_ALL);
            // �̹� ���� ���Ͱ� ���� ��쿡�� ����
            if (VCI_E_INVALID_STATE == hResult_)
                hResult_ = VCI_OK;
        }
		// CAN ��Ʈ�ѷ� ����
        if (hResult_ == VCI_OK)
            hResult_ = pCanControl_->StartLine();
    }
    return hResult_;
}

// =====================================
// FT ���� ������ ���� �� ó��
HRESULT FTSensorFunction::ProcessMessage(WORD wLimit) {
    std::lock_guard<std::mutex> lock(mtx_);

	// ������ ��ȿ���� Ȯ��
    if (!pReader_) return E_UNEXPECTED;
    
    PCANMSG pCanMsg;
    UINT16 wCount = 0;

	// �޽��� �б� �õ�
    HRESULT hr = pReader_->AcquireRead((PVOID*)&pCanMsg, &wCount);
    if (hr == VCI_OK) {
        if (wCount > wLimit) wCount = wLimit;

        // �޼��� ó��
        for (UINT16 i = 0; i < wCount; ++i)
            PrintMessage(&pCanMsg[i]);          // FT ���� �޼��� �о���� �Լ�
        pReader_->ReleaseRead(wCount);
    }
    return hr;
}

// =====================================
// FT ���� �޽��� �ؼ�(���� Force/Torque ������Ʈ)
void FTSensorFunction::PrintMessage(PCANMSG pCanMsg) {
    // CAN id���� Force/Torque �ؼ�
    if (pCanMsg->dwMsgId == 1) {
        // Force ���ð� ��� (ADIN Robot�� Force�� ��ȯ ���� ����)
        std::array<float, 3> f;
        f[0] = ((pCanMsg->abData[0]) * 256 + pCanMsg->abData[1]) / 100.0f - 300.0f;
		f[1] = ((pCanMsg->abData[2]) * 256 + pCanMsg->abData[3]) / 100.0f - 300.0f;
		f[2] = ((pCanMsg->abData[4]) * 256 + pCanMsg->abData[5]) / 100.0f - 300.0f;
        ftState_.setRawForce(f);
    }
    else if (pCanMsg->dwMsgId == 2) {
        // Torque ���ð� ��� (ADIN Robot�� Torque�� ��ȯ ���� ����)
        std::array<float, 3> t;
		t[0] = ((pCanMsg->abData[0]) * 256 + pCanMsg->abData[1]) / 500.0f - 50.0f;
		t[1] = ((pCanMsg->abData[2]) * 256 + pCanMsg->abData[3]) / 500.0f - 50.0f;
		t[2] = ((pCanMsg->abData[4]) * 256 + pCanMsg->abData[5]) / 500.0f - 50.0f;
        ftState_.setRawTorque(t);
    }
}

// =====================================
// FTState(����) ���� ���� �Լ�
FTState& FTSensorFunction::getState() { return ftState_; }
const FTState& FTSensorFunction::getState() const { return ftState_; }

// =====================================
// Snapshot(���纻) ���� �Լ�
FTState::Snapshot FTSensorFunction::getSnapshot() const {
    // ���������� FTState���� �������� �����ϰ� �����ؼ� ��ȯ
    return ftState_.getSnapshot();
}

// =====================================
// ���� ���� �� �ڿ� ����(����)
void FTSensorFunction::Release() {
    std::lock_guard<std::mutex> lock(mtx_);
    if (pCanControl_) { pCanControl_->StopLine(); pCanControl_->ResetLine(); pCanControl_->Release(); pCanControl_ = nullptr; }
    if (pCanChn_) { pCanChn_->Release(); pCanChn_ = nullptr; }
    if (pReader_) { pReader_->Release(); pReader_ = nullptr; }
    if (pWriter_) { pWriter_->Release(); pWriter_ = nullptr; }
    if (pBalObject_) { pBalObject_->Release(); pBalObject_ = nullptr; }
    if (hEventReader_) { CloseHandle(hEventReader_); hEventReader_ = nullptr; }
}

// =====================================
// ���� Ȯ�� �Լ� (���Ῡ�� ��)
bool FTSensorFunction::isConnected() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return (pCanControl_ && pCanChn_);
}

// =====================================
// �� ��ȿ�� �˻� �Լ�
bool FTSensorFunction::isForceValid(float minForce, float maxForce) const {
    auto f = ftState_.getFilteredForce();
    for (int i = 0; i < 3; ++i)
        if (f[i] < minForce || f[i] > maxForce)
            return false;
    return true;
}

// =====================================
// ���� �ʱ�ȭ �� ���� �ʱ�ȭ
void FTSensorFunction::initializeSensor() {
    std::lock_guard<std::mutex> lock(mtx_);
    ftState_.clearAll();
}

// =====================================
// ���� ���� Ȯ�� �Լ�
bool FTSensorFunction::checkSensorError(float minForce, float maxForce) {
    auto f = ftState_.getFilteredForce();
    for (int i = 0; i < 3; ++i)
        if (f[i] < minForce || f[i] > maxForce)
            return true;
    return false;
}
