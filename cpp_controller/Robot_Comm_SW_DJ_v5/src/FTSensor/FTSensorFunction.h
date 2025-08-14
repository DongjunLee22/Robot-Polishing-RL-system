#pragma once

#include "FTState.h"
#include <vcisdk.h>  // FT센서 통신 SDK
#include <mutex>

// FT 센서 기능(통신/초기화/메시지 처리 등)을 담당하는 클래스
class FTSensorFunction {
public:
    FTSensorFunction();
    ~FTSensorFunction();

    // FT센서 디바이스 선택 및 초기화
    HRESULT SelectDevice(BOOL fUserSelect);
    HRESULT CheckBalFeatures(LONG lCtrlNo);
    HRESULT InitSocket(LONG lCtrlNo);

    // FT센서 데이터 수신/처리
    HRESULT ProcessMessage(WORD wLimit);
    void    PrintMessage(PCANMSG pCanMsg);

    // FTState(상태) 접근 (읽기/쓰기)
    FTState&        getState();
    const FTState&  getState() const;

    // 스냅샷(복사본) 제공 함수
    FTState::Snapshot getSnapshot() const;

    // FT센서 자원 해제 (종료/Disconnect)
    void Release();

    // 기타 상태 확인 함수
    bool isConnected() const;                                   // 연결상태 확인
	bool isForceValid(float minForce, float maxForce) const;    // 힘 유효성 검사

	void initializeSensor();                                    // 센서 초기화
	bool checkSensorError(float minForce, float maxForce);      // 센서 오류 확인

private:
    // FT 센서 상태 관리 객체 (센서의 Force/Torque 데이터, bias 등 모든 상태 정보 저장)
    FTState ftState_;

    // 센서 드라이버/통신에 필요한 핸들 및 포인터
    HRESULT         hResult_ = 0;
    IBalObject*     pBalObject_ = nullptr;
    LONG            lBusCtrlNo_ = 0;
    ICanControl*    pCanControl_ = nullptr;
    ICanChannel*    pCanChn_ = nullptr;
    HANDLE          hEventReader_ = nullptr;
    PFIFOREADER     pReader_ = nullptr;
    PFIFOWRITER     pWriter_ = nullptr;

    // CAN 메시지, 기타 동기화 멤버 등
    mutable std::mutex mtx_;  // 내부 동기화용
};

