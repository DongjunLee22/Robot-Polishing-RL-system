# TCP/IP 통신 프로토콜 명세서 (v1.0)

## 1. 기본 연결 정보

- **Server**: Python 강화학습 에이전트 (`python_rl_agent`)
- **Client**: C++(MFC) 로봇 제어기 (`cpp_controller`)
- **Server IP Address**: `192.168.0.17`
- **Port**: `8888`

---

## 2. 통신 프로토콜 개요

- **데이터 형식**: 고정 길이의 바이너리(Binary) 패킷을 사용합니다.
- **바이트 순서 (Endianness)**: **리틀 엔디안 (Little-Endian)**을 기준으로 통신합니다.
  - C++(Windows, x86/x64) 환경의 기본값이 리틀 엔디안이므로, Python 측에서 수신할 때 이를 명시적으로 처리해야 합니다.
- **데이터 패딩 (Padding)**: `#pragma pack(push, 1)` 지시어를 사용하여 구조체 내에 메모리 패딩을 비활성화합니다. 따라서 패킷의 총 크기는 각 멤버 변수의 크기의 합과 같습니다.

---

## 3. 데이터 패킷 구조

### 3.1. C++ Client → Python Server 전송 패킷

- **구조체명**: `PythonCommPacket`
- **총 크기**: **17 bytes**
- **시작 플래그 (SOF)**: `0xAAAA`

| 순서 | 필드명             | 데이터 타입        | 크기 (bytes) | 설명                                   |
| :--- | :----------------- | :----------------- | :----------- | :------------------------------------- |
| 1    | `sof`              | `unsigned short`   | 2            | Start of Frame, 항상 `0xAAAA`          |
| 2    | `contactForceZ`    | `float`            | 4            | Z축 방향 접촉력 (단위: N)              |
| 3    | `chamberPressure`  | `float`            | 4            | 공압 챔버 압력 (단위: bar 등)          |
| 4    | `chamberVoltage`   | `float`            | 4            | 공압 챔버 제어 전압 (단위: V)          |
| 5    | `pidControlValue`  | `float`            | 4            | PID 제어기 출력 값                     |
| 6    | `pidFlag`          | `unsigned char`    | 1            | PID 제어기 활성화 상태 (0: 비활성, 1: 활성) |
| 7    | `checksum`         | `unsigned short`   | 2            | SOF를 제외한 데이터 필드의 무결성 검증 값 |

### 3.2. Python Server → C++ Client 전송 패킷

- **구조체명**: `RLAgentPacket`
- **총 크기**: **9 bytes**
- **시작 플래그 (SOF)**: `0xBBBB`

| 순서 | 필드명           | 데이터 타입      | 크기 (bytes) | 설명                                      |
| :--- | :--------------- | :--------------- | :----------- | :---------------------------------------- |
| 1    | `sof`            | `unsigned short` | 2            | Start of Frame, 항상 `0xBBBB`             |
| 2    | `rlVoltageValue` | `float`          | 4            | 강화학습 에이전트가 결정한 제어 전압 (단위: V) |
| 3    | `confirmFlag`    | `unsigned char`  | 1            | C++로부터 메시지 수신 확인용 (0: 수신 못함, 1: 수신함) |
| 4    | `checksum`       | `unsigned short` | 2            | SOF를 제외한 데이터 필드의 무결성 검증 값 |

---

## 4. 변경 이력

- **v1.0 (2025-08-14)**: 문서 최초 작성 (JSON 기반)
