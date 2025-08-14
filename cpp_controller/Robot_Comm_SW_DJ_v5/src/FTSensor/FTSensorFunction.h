#pragma once

#include "FTState.h"
#include <vcisdk.h>  // FT���� ��� SDK
#include <mutex>

// FT ���� ���(���/�ʱ�ȭ/�޽��� ó�� ��)�� ����ϴ� Ŭ����
class FTSensorFunction {
public:
    FTSensorFunction();
    ~FTSensorFunction();

    // FT���� ����̽� ���� �� �ʱ�ȭ
    HRESULT SelectDevice(BOOL fUserSelect);
    HRESULT CheckBalFeatures(LONG lCtrlNo);
    HRESULT InitSocket(LONG lCtrlNo);

    // FT���� ������ ����/ó��
    HRESULT ProcessMessage(WORD wLimit);
    void    PrintMessage(PCANMSG pCanMsg);

    // FTState(����) ���� (�б�/����)
    FTState&        getState();
    const FTState&  getState() const;

    // ������(���纻) ���� �Լ�
    FTState::Snapshot getSnapshot() const;

    // FT���� �ڿ� ���� (����/Disconnect)
    void Release();

    // ��Ÿ ���� Ȯ�� �Լ�
    bool isConnected() const;                                   // ������� Ȯ��
	bool isForceValid(float minForce, float maxForce) const;    // �� ��ȿ�� �˻�

	void initializeSensor();                                    // ���� �ʱ�ȭ
	bool checkSensorError(float minForce, float maxForce);      // ���� ���� Ȯ��

private:
    // FT ���� ���� ���� ��ü (������ Force/Torque ������, bias �� ��� ���� ���� ����)
    FTState ftState_;

    // ���� ����̹�/��ſ� �ʿ��� �ڵ� �� ������
    HRESULT         hResult_ = 0;
    IBalObject*     pBalObject_ = nullptr;
    LONG            lBusCtrlNo_ = 0;
    ICanControl*    pCanControl_ = nullptr;
    ICanChannel*    pCanChn_ = nullptr;
    HANDLE          hEventReader_ = nullptr;
    PFIFOREADER     pReader_ = nullptr;
    PFIFOWRITER     pWriter_ = nullptr;

    // CAN �޽���, ��Ÿ ����ȭ ��� ��
    mutable std::mutex mtx_;  // ���� ����ȭ��
};

