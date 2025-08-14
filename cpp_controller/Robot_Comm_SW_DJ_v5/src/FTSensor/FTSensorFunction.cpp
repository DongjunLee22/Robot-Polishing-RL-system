#include "FTSensorFunction.h"
#include <cstring>      // std::memset 등
#include <algorithm>    // std::clamp

// --- 생성자/소멸자 ---
FTSensorFunction::FTSensorFunction() {
    // 필요시 초기화
}
FTSensorFunction::~FTSensorFunction() {
    Release();  // 소멸 시 자원 해제
}

// =====================================
// FT 센서 연결 및 초기화 함수
HRESULT FTSensorFunction::SelectDevice(BOOL fUserSelect) {
    std::lock_guard<std::mutex> lock(mtx_);

    // FT 센서 연결 코드
    if (!fUserSelect) return E_FAIL;

	IVciDeviceManager*  pDevMgr = nullptr;      // 디바이스 매니저
	IVciEnumDevice*     pEnum = nullptr;        // enumerator 핸들
	VCIDEVICEINFO       sInfo;                  // 디바이스 정보

	// 디바이스 매니저 초기화
    hResult_ = VciGetDeviceManager(&pDevMgr);
    if (hResult_ == VCI_OK) hResult_ = pDevMgr->EnumDevices(&pEnum);
    if (hResult_ == VCI_OK) hResult_ = pEnum->Next(1, &sInfo, NULL);
    if (pEnum) { pEnum->Release(); pEnum = nullptr; }

	// 디바이스 정보 확인
    if (hResult_ == VCI_OK) {
        IVciDevice* pDevice;
        hResult_ = pDevMgr->OpenDevice(sInfo.VciObjectId, &pDevice);
		
        // 디바이스 열기 실패 시
        if (hResult_ == VCI_OK) {
            hResult_ = pDevice->OpenComponent(CLSID_VCIBAL, IID_IBalObject, (void**)&pBalObject_);
            pDevice->Release();
        }
        lBusCtrlNo_ = 0;
    }
    return hResult_;
}

// =====================================
// FT 센서 기능 초기화 및 소켓 설정
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
// FT 센서 소켓 초기화 및 메시지 채널 설정
HRESULT FTSensorFunction::InitSocket(LONG lCtrlNo) {
    std::lock_guard<std::mutex> lock(mtx_);

    if (!pBalObject_) return E_FAIL;
    ICanSocket* pCanSocket = nullptr;
    hResult_ = pBalObject_->OpenSocket(lCtrlNo, IID_ICanSocket, (void**)&pCanSocket);
    if (hResult_ != VCI_OK) return hResult_;

    // Check CAN capabilities (생략 가능)
    CANCAPABILITIES capabilities = { 0 };
    hResult_ = pCanSocket->GetCapabilities(&capabilities);
    if (hResult_ == VCI_OK && !(capabilities.dwFeatures & CAN_FEATURE_STDANDEXT))
        hResult_ = VCI_E_NOT_SUPPORTED;

    // 메시지 채널 생성 및 초기화
    if (hResult_ == VCI_OK) hResult_ = pCanSocket->CreateChannel(FALSE, &pCanChn_);
    pCanSocket->Release();

    // 채널 초기화 및 리더/라이터 생성
    if (hResult_ == VCI_OK)
    {
        UINT16 wRxFifoSize = 1024;        
        UINT16 wTxFifoSize = 128;

        hResult_ = pCanChn_->Initialize(wRxFifoSize, wTxFifoSize);
    }
	// 리더와 라이터 생성 및 이벤트 설정
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
	// 채널 활성화 및 CAN 컨트롤러 열기
    if (hResult_ == VCI_OK) hResult_ = pCanChn_->Activate();
    if (hResult_ == VCI_OK) hResult_ = pBalObject_->OpenSocket(lCtrlNo, IID_ICanControl, (void**)&pCanControl_);
	
    // CAN 컨트롤러 초기화
    if (hResult_ == VCI_OK) {
        CANINITLINE init = { CAN_OPMODE_STANDARD | CAN_OPMODE_EXTENDED | CAN_OPMODE_ERRFRAME, 0, CAN_BT0_1000KB, CAN_BT1_1000KB };
        hResult_ = pCanControl_->InitLine(&init);
        // CAN 컨트롤러에 대한 액세스 필터 설정
        if (hResult_ == VCI_OK) {
            // 표준 프레임 필터 설정
            hResult_ = pCanControl_->SetAccFilter(CAN_FILTER_STD, CAN_ACC_CODE_ALL, CAN_ACC_MASK_ALL);
            // 확장 프레임 필터 설정
            if (hResult_ == VCI_OK)
                hResult_ = pCanControl_->SetAccFilter(CAN_FILTER_EXT, CAN_ACC_CODE_ALL, CAN_ACC_MASK_ALL);
            // 이미 동일 필터가 있을 경우에는 무시
            if (VCI_E_INVALID_STATE == hResult_)
                hResult_ = VCI_OK;
        }
		// CAN 컨트롤러 시작
        if (hResult_ == VCI_OK)
            hResult_ = pCanControl_->StartLine();
    }
    return hResult_;
}

// =====================================
// FT 센서 데이터 수신 및 처리
HRESULT FTSensorFunction::ProcessMessage(WORD wLimit) {
    std::lock_guard<std::mutex> lock(mtx_);

	// 리더가 유효한지 확인
    if (!pReader_) return E_UNEXPECTED;
    
    PCANMSG pCanMsg;
    UINT16 wCount = 0;

	// 메시지 읽기 시도
    HRESULT hr = pReader_->AcquireRead((PVOID*)&pCanMsg, &wCount);
    if (hr == VCI_OK) {
        if (wCount > wLimit) wCount = wLimit;

        // 메세지 처리
        for (UINT16 i = 0; i < wCount; ++i)
            PrintMessage(&pCanMsg[i]);          // FT 센서 메세지 읽어오는 함수
        pReader_->ReleaseRead(wCount);
    }
    return hr;
}

// =====================================
// FT 센서 메시지 해석(실제 Force/Torque 업데이트)
void FTSensorFunction::PrintMessage(PCANMSG pCanMsg) {
    // CAN id별로 Force/Torque 해석
    if (pCanMsg->dwMsgId == 1) {
        // Force 원시값 계산 (ADIN Robot의 Force값 변환 공식 적용)
        std::array<float, 3> f;
        f[0] = ((pCanMsg->abData[0]) * 256 + pCanMsg->abData[1]) / 100.0f - 300.0f;
		f[1] = ((pCanMsg->abData[2]) * 256 + pCanMsg->abData[3]) / 100.0f - 300.0f;
		f[2] = ((pCanMsg->abData[4]) * 256 + pCanMsg->abData[5]) / 100.0f - 300.0f;
        ftState_.setRawForce(f);
    }
    else if (pCanMsg->dwMsgId == 2) {
        // Torque 원시값 계산 (ADIN Robot의 Torque값 변환 공식 적용)
        std::array<float, 3> t;
		t[0] = ((pCanMsg->abData[0]) * 256 + pCanMsg->abData[1]) / 500.0f - 50.0f;
		t[1] = ((pCanMsg->abData[2]) * 256 + pCanMsg->abData[3]) / 500.0f - 50.0f;
		t[2] = ((pCanMsg->abData[4]) * 256 + pCanMsg->abData[5]) / 500.0f - 50.0f;
        ftState_.setRawTorque(t);
    }
}

// =====================================
// FTState(상태) 원본 접근 함수
FTState& FTSensorFunction::getState() { return ftState_; }
const FTState& FTSensorFunction::getState() const { return ftState_; }

// =====================================
// Snapshot(복사본) 제공 함수
FTState::Snapshot FTSensorFunction::getSnapshot() const {
    // 내부적으로 FTState에서 스냅샷을 안전하게 복사해서 반환
    return ftState_.getSnapshot();
}

// =====================================
// 센서 연결 및 자원 해제(종료)
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
// 상태 확인 함수 (연결여부 등)
bool FTSensorFunction::isConnected() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return (pCanControl_ && pCanChn_);
}

// =====================================
// 힘 유효성 검사 함수
bool FTSensorFunction::isForceValid(float minForce, float maxForce) const {
    auto f = ftState_.getFilteredForce();
    for (int i = 0; i < 3; ++i)
        if (f[i] < minForce || f[i] > maxForce)
            return false;
    return true;
}

// =====================================
// 센서 초기화 및 상태 초기화
void FTSensorFunction::initializeSensor() {
    std::lock_guard<std::mutex> lock(mtx_);
    ftState_.clearAll();
}

// =====================================
// 센서 오류 확인 함수
bool FTSensorFunction::checkSensorError(float minForce, float maxForce) {
    auto f = ftState_.getFilteredForce();
    for (int i = 0; i < 3; ++i)
        if (f[i] < minForce || f[i] > maxForce)
            return true;
    return false;
}
