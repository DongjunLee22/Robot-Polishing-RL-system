// TcpClient.cpp
#include "TcpClient.h"
#include "Protocol.h"

TcpClient::TcpClient()
    : m_pReceiveThread(nullptr), m_isConnected(false), m_isThreadRunning(false), m_serverPort(0)
{
}

TcpClient::~TcpClient()
{
    // 소멸자에서 반드시 연결을 종료하여 리소스를 해제합니다.
    Disconnect();
}

bool TcpClient::Connect(const CString& ipAddress, UINT port)
{
	std::lock_guard<std::mutex> lock(m_connectMutex);
	if (m_isConnected.load() || m_isThreadRunning.load()) {
		return true; // 이미 연결 중이거나 쓰레드가 실행 중이면 아무것도 하지 않음
	}

	// 서버 정보를 멤버 변수에 저장
	m_serverIpAddress = ipAddress;
	m_serverPort = port;

	// 쓰레드 시작 플래그 설정 및 쓰레드 시작
	m_isThreadRunning.store(true);
	m_pReceiveThread = AfxBeginThread(ReceiveThreadFunc, this);
	if (!m_pReceiveThread) {
		m_isThreadRunning.store(false);
		return false;
	}

	return true;
}

void TcpClient::Disconnect()
{
    std::lock_guard<std::mutex> lock(m_connectMutex);

    if (!m_isConnected.load()) {
        return;
    }

    m_isThreadRunning.store(false);

    m_cv.notify_one();

	if (m_pReceiveThread != nullptr) {
		WaitForSingleObject(m_pReceiveThread->m_hThread, 5000);
		m_pReceiveThread = nullptr;
	}

    m_isConnected.store(false);
}

bool TcpClient::Send(const char* data, int length)
{
	if (!m_isConnected.load()) {
		return false;
	}

	{
		std::lock_guard<std::mutex> lock(m_queueMutex);
		// char* 데이터를 std::vector<char>로 변환하여 큐에 추가
		m_sendQueue.push(std::vector<char>(data, data + length));
	}
	m_cv.notify_one();

	return true;
}

bool TcpClient::IsConnected() const
{
    return m_isConnected.load();
}

void TcpClient::SetReceiveCallback(std::function<void(const RLAgentPacket&)> callback)
{
	m_receiveCallback = callback;
}

UINT TcpClient::ReceiveThreadFunc(LPVOID pParam)
{
	TcpClient* pClient = static_cast<TcpClient*>(pParam);

	// 1. 워커 쓰레드 내에서 소켓 객체 생성
	AfxSocketInit(NULL);
	CSocket clientSocket;

	if (!clientSocket.Create()) {
		// 오류 처리
		pClient->m_isThreadRunning.store(false);
		return 1;
	}

	// 2. 연결 시도
	if (!clientSocket.Connect(pClient->m_serverIpAddress, pClient->m_serverPort)) {
		// 오류 처리
		clientSocket.Close();
		pClient->m_isThreadRunning.store(false);
		return 1;
	}
	pClient->m_isConnected.store(true);

	// 네이글 알고리즘 비활성화 (지연 없는 전송을 위해)
	BOOL bNodelay = TRUE;
	clientSocket.SetSockOpt(TCP_NODELAY, &bNodelay, sizeof(bNodelay));

	std::vector<char> recvBuffer; // 가변 크기 수신 버퍼
	char tempBuffer[2048];        // 소켓에서 직접 읽어올 임시 버퍼

	// 3. 메인 루프 시작 (select 모델 사용)
	while (pClient->m_isThreadRunning.load())
	{
		fd_set read_fds, write_fds;
		FD_ZERO(&read_fds);
		FD_ZERO(&write_fds);

		// 항상 수신을 감시
		FD_SET(clientSocket.m_hSocket, &read_fds);

		// 보낼 메시지가 있을 때만 송신을 감시
		if (!pClient->m_sendQueue.empty()) {
			FD_SET(clientSocket.m_hSocket, &write_fds);
		}

		// 100ms 타임아웃 설정 (루프가 m_isThreadRunning 플래그를 주기적으로 확인할 수 있도록)
		timeval tv;
		tv.tv_sec = 0;
		tv.tv_usec = 100000;

		int sel_res = select(0, &read_fds, &write_fds, NULL, &tv);

		if (sel_res < 0) { // 오류 발생 시 루프 종료
			break;
		}

		if (sel_res == 0) { // 타임아웃 발생 시 다시 루프 시작
			continue;
		}

		// 4. 수신 처리
		if (FD_ISSET(clientSocket.m_hSocket, &read_fds)) {
			int recvLen = clientSocket.Receive(tempBuffer, sizeof(tempBuffer));
			if (recvLen <= 0) { // 연결 끊김 또는 오류
				break;
			}

			// 임시 버퍼에 받은 데이터를 메인 수신 버퍼(recvBuffer) 뒤에 추가
			recvBuffer.insert(recvBuffer.end(), tempBuffer, tempBuffer + recvLen);

			// 메인 수신 버퍼에 완전한 패킷이 있는지 반복해서 확인
			while (true) {
				// 최소 패킷 길이(9바이트)보다 데이터가 적으면 더 수신해야 함
				if (recvBuffer.size() < sizeof(RLAgentPacket)) {
					break;
				}

				// SOF (0xBBBB)를 찾음
				size_t sof_pos = std::string::npos;
				for (size_t i = 0; i <= recvBuffer.size() - 2; ++i) {
					// Big-Endian (네트워크 바이트 순서)로 0xBBBB를 확인
					if ((unsigned char)recvBuffer[i] == 0xBB && (unsigned char)recvBuffer[i + 1] == 0xBB) {
						sof_pos = i;
						break;
					}
				}

				if (sof_pos == std::string::npos) {
					// SOF를 못찾았으면 버퍼에 유효한 데이터가 거의 없는 것
					// (단, 마지막 1바이트는 다음 SOF의 일부일 수 있으므로 남김)
					if (recvBuffer.size() > 1) {
						recvBuffer.erase(recvBuffer.begin(), recvBuffer.end() - 1);
					}
					break;
				}

				// SOF 이전의 불필요한 데이터를 모두 삭제
				if (sof_pos > 0) {
					recvBuffer.erase(recvBuffer.begin(), recvBuffer.begin() + sof_pos);
				}

				// SOF부터 시작하는 완전한 패킷이 있는지 확인
				if (recvBuffer.size() < sizeof(RLAgentPacket)) {
					// SOF는 찾았지만 아직 패킷 전체가 도착하지 않음
					break;
				}

				RLAgentPacket parsedPacket;
				// 새로 만든 Unpack 함수로 데이터 해석 및 검증 시도
				if (UnpackRLAgentCommand(recvBuffer.data(), recvBuffer.size(), parsedPacket))
				{
					// 성공 시, 잘 정리된 구조체를 콜백으로 전달
					if (pClient->m_receiveCallback) {
						pClient->m_receiveCallback(parsedPacket);
					}
				}
				else
				{
					// Unpack 실패 (체크섬 오류 등) 시, 메시지를 출력하고 패킷을 폐기
					TRACE("Packet discard due to unpacking failure.\n");
				}

				// 성공이든 실패든, 처리한 패킷(9바이트)은 버퍼에서 "한 번만" 삭제
				recvBuffer.erase(recvBuffer.begin(), recvBuffer.begin() + sizeof(RLAgentPacket));
			}
		}

		// 5. 송신 처리
		if (FD_ISSET(clientSocket.m_hSocket, &write_fds)) {
			std::lock_guard<std::mutex> lock(pClient->m_queueMutex);
			if (!pClient->m_sendQueue.empty()) {
				// CString이 아닌 std::vector<char>를 큐에서 가져옴
				std::vector<char> dataToSend = pClient->m_sendQueue.front();
				pClient->m_sendQueue.pop();

				// CT2A 변환 없이 바로 데이터 전송
				if (!dataToSend.empty()) {
					clientSocket.Send(dataToSend.data(), dataToSend.size());
				}
			}
		}
	}

	// 6. 쓰레드 종료 전 리소스 정리
	pClient->m_isConnected.store(false);
	pClient->m_isThreadRunning.store(false);
	clientSocket.Close();

	return 0;
}
