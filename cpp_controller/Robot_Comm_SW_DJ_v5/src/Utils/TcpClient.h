// TcpClient.h
#pragma once

#include <afxsock.h>
#include <functional> // std::function을 사용하기 위해 필요
#include <atomic>
#include <mutex>
#include <queue>
#include <condition_variable>
#include <vector>
#include "Protocol.h"

class TcpClient
{
public:
    TcpClient();
    ~TcpClient();

    // 서버에 연결하고 데이터 수신을 시작합니다.
    bool Connect(const CString& ipAddress, UINT port);

    // 연결을 종료하고 쓰레드를 정리합니다.
    void Disconnect();

    // 서버로 메시지를 전송합니다.
    bool Send(const char* data, int length);

    // 현재 연결 상태를 반환합니다.
    bool IsConnected() const;

    // 데이터 수신 시 호출될 콜백 함수를 설정합니다.
    // std::function은 모든 종류의 호출 가능한 것(함수 포인터, 람다 등)을 저장할 수 있는 유연한 래퍼입니다.
    //void SetReceiveCallback(std::function<void(const char*, int)> callback);
    void SetReceiveCallback(std::function<void(const RLAgentPacket&)> callback);

private:
    // 쓰레드에서 실행될 정적 멤버 함수
	static UINT ReceiveThreadFunc(LPVOID pParam);           // 데이터 수신 쓰레드 함수

    CWinThread* m_pReceiveThread;                           // 데이터 수신을 위한 쓰레드 포인터

    std::atomic<bool> m_isConnected;                        // 연결 상태 플래그
    std::atomic<bool> m_isThreadRunning;                    // 쓰레드 루프 제어 플래그

	std::queue<std::vector<char>> m_sendQueue;				// 전송할 메시지를 저장하는 큐

	std::condition_variable m_cv;                           // 큐에 메시지가 추가되었음을 워커 쓰레드에 알리기 위한 변수

    // 서버 정보를 워커 쓰레드에 전달하기 위한 멤버 변수
	CString m_serverIpAddress;                              // 서버 IP 주소
	UINT m_serverPort;  						            // 서버 포트 번호

    //std::function<void(const char*, int)> m_receiveCallback;// 외부(다이얼로그)에서 전달받은 콜백 함수를 저장할 멤버 변수
    std::function<void(const RLAgentPacket&)> m_receiveCallback;
    
    std::mutex m_queueMutex;                                // m_sendQueue를 보호하기 위한 뮤텍스
    std::mutex m_connectMutex;                              // 연결 상태를 보호하기 위한 뮤텍스
};