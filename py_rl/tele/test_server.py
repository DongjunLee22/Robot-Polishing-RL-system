import socket
import struct
import threading
import time
import signal
import sys

# --- 통신 프로토콜 설정 (C++의 Protocol.h와 동일해야 함) ---

# 1) C++ -> Python (수신용)
# Big-Endian: unsigned short(SOF), float*4, unsigned char, unsigned short(checksum)
CPP_TO_PY_PACKET_FORMAT = ">HffffBH"
CPP_TO_PY_PACKET_SIZE   = 29
CPP_TO_PY_SOF           = 0xAAAA

# 2) Python -> C++ (송신용)
# 주석에 'unsigned char' 이므로 'B' 사용 (이전 코드의 'b'는 signed였음)
PY_TO_CPP_PACKET_FORMAT = ">HfBH"
PY_TO_CPP_PACKET_SIZE   = 10
PY_TO_CPP_SOF           = 0xBBBB

HOST = '0.0.0.0'
PORT = 8888

shutdown_event = threading.Event()

def simple_xor_checksum(data: bytes) -> int:
    """간단한 XOR 체크섬 (0~255)"""
    checksum = 0
    for byte in data:
        checksum ^= byte
    return checksum & 0xFF

def recv_exact(conn: socket.socket, n: int) -> bytes:
    """정확히 n바이트를 받을 때까지 반복 수신 (EOF 시 빈 bytes 반환)"""
    buf = bytearray()
    while len(buf) < n and not shutdown_event.is_set():
        chunk = conn.recv(n - len(buf))
        if not chunk:
            # 연결 종료
            return b""
        buf.extend(chunk)
    return bytes(buf)

def receiver_thread(conn: socket.socket, addr):
    print(f"[수신 쓰레드] C++ 클라이언트({addr})로부터 데이터 수신 시작")
    try:
        while not shutdown_event.is_set():
            data = recv_exact(conn, CPP_TO_PY_PACKET_SIZE)
            if not data:
                print(f"[수신 쓰레드] C++ 클라이언트({addr}) 연결 종료(EOF)")
                break

            # 1) 길이 검증
            if len(data) != CPP_TO_PY_PACKET_SIZE:
                print(f"[경고] {CPP_TO_PY_PACKET_SIZE}B가 아닌 {len(data)}B 수신 (조립 실패)")
                continue

            # 2) 언패킹
            try:
                sof, force_z, pressure, voltage, pid_val, pid_flag, received_checksum = struct.unpack(
                    CPP_TO_PY_PACKET_FORMAT, data
                )
            except struct.error as e:
                print(f"[오류] struct.unpack 실패: {e}")
                continue

            # 3) SOF 검증
            if sof != CPP_TO_PY_SOF:
                print(f"[오류] SOF 불일치: {hex(sof)} (기대: {hex(CPP_TO_PY_SOF)})")
                continue

            # 4) 체크섬 검증 (마지막 2바이트는 checksum)
            calculated_checksum = simple_xor_checksum(data[:-2]) & 0xFF
            received_checksum_u8 = received_checksum & 0xFF  # C++에서 uint16_t로 올 수도 있어 마스킹

            if received_checksum_u8 == calculated_checksum:
                print(f"✅ [수신] Fz:{force_z:.3f}  P:{pressure:.3f}  V:{voltage:.3f}  PID:{pid_val:.3f}  FLAG:{pid_flag}")
            else:
                print(f"❌ [체크섬 오류] recv:{received_checksum_u8} calc:{calculated_checksum}")

    except (ConnectionResetError, OSError) as e:
        print(f"[수신 쓰레드] 예외: {e}")
    finally:
        shutdown_event.set()
        try:
            conn.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        conn.close()
        print("[수신 쓰레드] 종료")

def sender_thread(conn: socket.socket, addr):
    print(f"[송신 쓰레드] C++ 클라이언트({addr})로 데이터 송신 시작")
    try:
        while not shutdown_event.is_set():
            # 1) 페이로드 준비
            rl_voltage_value = 1.234 + (time.time() % 1.0)
            confirm_flag = 1  # unsigned char 의미 → 'B'

            # 2) 체크섬 계산용 데이터 (checksum 제외 부분과 동일하게)
            data_part = struct.pack(">HfB", PY_TO_CPP_SOF, rl_voltage_value, confirm_flag)
            checksum = simple_xor_checksum(data_part)  # 0~255

            # 3) 최종 패킷 (SOF, float, flag, checksum[uint16])
            final_packet = struct.pack(PY_TO_CPP_PACKET_FORMAT, PY_TO_CPP_SOF, rl_voltage_value, confirm_flag, checksum)

            # 4) 송신
            conn.sendall(final_packet)
            # print(f"[송신] V:{rl_voltage_value:.3f} FLAG:{confirm_flag} CHK:{checksum}")

            time.sleep(0.5)

    except (BrokenPipeError, ConnectionResetError, OSError) as e:
        print(f"[송신 쓰레드] 예외: {e}")
    finally:
        shutdown_event.set()
        try:
            conn.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        conn.close()
        print("[송신 쓰레드] 종료")

def main():
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    try:
        server_socket.bind((HOST, PORT))
        server_socket.listen(1)

        # 안내 문구: 바인딩은 0.0.0.0이며, 실제 접속은 동일 서브넷 NIC IP를 사용
        print(f"✅ 서버 시작: 모든 인터페이스 0.0.0.0:{PORT} (클라이언트는 서버 NIC의 실제 IP로 접속하세요)")
        print("Ctrl+C를 누르면 종료합니다.")

        conn, addr = server_socket.accept()
        print(f"🎉 C++ 클라이언트 {addr} 연결됨")

        # ✨ 여기! addr도 함께 넘깁니다 (이전 코드의 TypeError 수정 포인트)
        receiver = threading.Thread(target=receiver_thread, args=(conn, addr), daemon=True)
        sender   = threading.Thread(target=sender_thread,   args=(conn, addr), daemon=True)
        receiver.start()
        sender.start()

        # 둘 중 하나가 끝나면 종료 루프
        while receiver.is_alive() and sender.is_alive():
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n[서버] 종료 신호 수신")
    except Exception as e:
        print(f"[메인] 서버 오류: {e}")
    finally:
        shutdown_event.set()
        try:
            server_socket.close()
        except OSError:
            pass
        print("[서버] 소켓 닫힘, 프로그램 종료")

if __name__ == '__main__':
    main()
