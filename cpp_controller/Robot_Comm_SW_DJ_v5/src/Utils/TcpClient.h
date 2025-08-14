// TcpClient.h
#pragma once

#include <afxsock.h>
#include <functional> // std::function�� ����ϱ� ���� �ʿ�
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

    // ������ �����ϰ� ������ ������ �����մϴ�.
    bool Connect(const CString& ipAddress, UINT port);

    // ������ �����ϰ� �����带 �����մϴ�.
    void Disconnect();

    // ������ �޽����� �����մϴ�.
    bool Send(const char* data, int length);

    // ���� ���� ���¸� ��ȯ�մϴ�.
    bool IsConnected() const;

    // ������ ���� �� ȣ��� �ݹ� �Լ��� �����մϴ�.
    // std::function�� ��� ������ ȣ�� ������ ��(�Լ� ������, ���� ��)�� ������ �� �ִ� ������ �����Դϴ�.
    //void SetReceiveCallback(std::function<void(const char*, int)> callback);
    void SetReceiveCallback(std::function<void(const RLAgentPacket&)> callback);

private:
    // �����忡�� ����� ���� ��� �Լ�
	static UINT ReceiveThreadFunc(LPVOID pParam);           // ������ ���� ������ �Լ�

    CWinThread* m_pReceiveThread;                           // ������ ������ ���� ������ ������

    std::atomic<bool> m_isConnected;                        // ���� ���� �÷���
    std::atomic<bool> m_isThreadRunning;                    // ������ ���� ���� �÷���

	std::queue<std::vector<char>> m_sendQueue;				// ������ �޽����� �����ϴ� ť

	std::condition_variable m_cv;                           // ť�� �޽����� �߰��Ǿ����� ��Ŀ �����忡 �˸��� ���� ����

    // ���� ������ ��Ŀ �����忡 �����ϱ� ���� ��� ����
	CString m_serverIpAddress;                              // ���� IP �ּ�
	UINT m_serverPort;  						            // ���� ��Ʈ ��ȣ

    //std::function<void(const char*, int)> m_receiveCallback;// �ܺ�(���̾�α�)���� ���޹��� �ݹ� �Լ��� ������ ��� ����
    std::function<void(const RLAgentPacket&)> m_receiveCallback;
    
    std::mutex m_queueMutex;                                // m_sendQueue�� ��ȣ�ϱ� ���� ���ؽ�
    std::mutex m_connectMutex;                              // ���� ���¸� ��ȣ�ϱ� ���� ���ؽ�
};