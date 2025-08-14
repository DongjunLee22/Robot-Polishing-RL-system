import socket
import struct
import threading
import time
import signal
import sys

# --- 통신 프로토콜 설정 ---
# C++의 Protocol.h와 완벽하게 일치해야 합니다.
# 1. C++ -> Python (수신용)
CPP_TO_PY_PACKET_FORMAT = ">HffffBH"  # Big-Endian, unsigned short, 4 floats, unsigned char, unsigned short
CPP_TO_PY_PACKET_SIZE = 21
CPP_TO_PY_SOF = 0xAAAA

# 2. Python -> C++ (송신용)
PY_TO_CPP_PACKET_FORMAT = ">HfbH"  # Big-Endian, unsigned short, float, unsigned char, unsigned short
PY_TO_CPP_PACKET_SIZE = 9
PY_TO_CPP_SOF = 0xBBBB

# 서버 설정
HOST = '0.0.0.0'  # 모든 IP 주소에서 오는 연결을 허용
PORT = 8888       # C++ 코드에 설정된 포트 번호

# 서버 실행 상태를 제어하기 위한 이벤트
shutdown_event = threading.Event()

def simple_xor_checksum(data):
    """간단한 XOR 체크섬 계산 함수"""
    checksum = 0
    for byte in data:
        checksum ^= byte
    return checksum

def receiver_thread(conn, addr):
    """C++ 클라이언트로부터 데이터를 수신하는 쓰레드"""
    print(f"[수신 쓰레드] C++ 클라이언트({addr})로부터 데이터 수신을 시작합니다.")
    try:
        while not shutdown_event.is_set():
            # 정확히 21바이트의 데이터를 수신
            data = conn.recv(CPP_TO_PY_PACKET_SIZE)
            if not data:
                print(f"[수신 쓰레드] C++ 클라이언트({addr})와 연결이 끊어졌습니다.")
                break

            if len(data) == CPP_TO_PY_PACKET_SIZE:
                # 1. 데이터 언패킹
                unpacked_data = struct.unpack(CPP_TO_PY_PACKET_FORMAT, data)
                
                sof, force_z, pressure, voltage, pid_val, pid_flag, received_checksum = unpacked_data

                # 2. SOF 검증
                if sof != CPP_TO_PY_SOF:
                    print(f"[오류] SOF 불일치! 수신된 SOF: {hex(sof)}")
                    continue
                
                # 3. 체크섬 검증
                data_for_checksum = data[:-2] # 체크섬 필드를 제외한 데이터 부분
                calculated_checksum = simple_xor_checksum(data_for_checksum)

                if received_checksum == calculated_checksum:
                    print(f"✅ [수신 성공] ForceZ: {force_z:.2f}, Pressure: {pressure:.2f}, Voltage: {voltage:.2f}, PID_Val: {pid_val:.2f}, PID_Flag: {pid_flag}")
                else:
                    print(f"❌ [체크섬 오류] 수신된 체크섬: {received_checksum}, 계산된 체크섬: {calculated_checksum}")
            else:
                print(f"[경고] {CPP_TO_PY_PACKET_SIZE}바이트가 아닌 {len(data)}바이트의 데이터 수신")

    except ConnectionResetError:
        print(f"[수신 쓰레드] C++ 클라이언트({addr})에서 연결을 강제로 종료했습니다.")
    except Exception as e:
        print(f"[수신 쓰레드] 오류 발생: {e}")
    finally:
        print("[수신 쓰레드] 종료.")
        shutdown_event.set() # 한 쓰레드가 끝나면 다른 쓰레드도 종료하도록 신호

def sender_thread(conn, addr):
    """C++ 클라이언트로 데이터를 주기적으로 송신하는 쓰레드"""
    print(f"[송신 쓰레드] C++ 클라이언트({addr})로 데이터 송신을 시작합니다.")
    try:
        while not shutdown_event.is_set():
            # 1. 보낼 테스트 데이터 생성
            rl_voltage_value = 1.234 + time.time() % 1.0 # 시간이 지남에 따라 변하는 값
            confirm_flag = 1
            
            # 2. 체크섬 계산을 위해 데이터 부분만 먼저 패킹
            data_part = struct.pack('>Hfb', PY_TO_CPP_SOF, rl_voltage_value, confirm_flag)
            checksum = simple_xor_checksum(data_part)
            
            # 3. 최종 패킷 패킹 (SOF, 데이터, 체크섬)
            final_packet = struct.pack(PY_TO_CPP_PACKET_FORMAT, PY_TO_CPP_SOF, rl_voltage_value, confirm_flag, checksum)
            
            # 4. 데이터 전송
            conn.sendall(final_packet)
            # print(f"-> [송신] RL Voltage: {rl_voltage_value:.2f}")
            
            # 0.5초 대기
            time.sleep(0.5)

    except (BrokenPipeError, ConnectionResetError):
        print(f"[송신 쓰레드] C++ 클라이언트({addr})와 연결이 끊어졌습니다.")
    except Exception as e:
        print(f"[송신 쓰레드] 오류 발생: {e}")
    finally:
        print("[송신 쓰레드] 종료.")
        shutdown_event.set() # 한 쓰레드가 끝나면 다른 쓰레드도 종료하도록 신호

def main():
    """메인 서버 로직"""
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # SO_REUSEADDR 옵션을 설정하여 서버가 즉시 재시작될 수 있도록 함
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    
    try:
        server_socket.bind((HOST, PORT))
        server_socket.listen()
        
        my_ip = socket.gethostbyname(socket.gethostname())
        print(f"✅ 서버가 시작되었습니다. C++ 클라이언트에서 IP: {my_ip}, Port: {PORT} 로 연결하세요.")
        print("Ctrl+C를 누르면 서버를 종료합니다.")

        conn, addr = server_socket.accept()
        print(f"🎉 C++ 클라이언트 {addr}가 연결되었습니다.")
        
        # 수신 및 송신 쓰레드 생성 및 시작
        receiver = threading.Thread(target=receiver_thread, args=(conn,))
        sender = threading.Thread(target=sender_thread, args=(conn,))
        
        receiver.start()
        sender.start()
        
        # 쓰레드가 종료될 때까지 대기
        receiver.join()
        sender.join()

    except KeyboardInterrupt:
        print("\n[서버] 종료 신호를 받았습니다. 모든 연결을 정리합니다.")
    except Exception as e:
        print(f"[메인] 서버 오류 발생: {e}")
    finally:
        shutdown_event.set()
        server_socket.close()
        print("[서버] 소켓을 닫고 프로그램을 종료합니다.")

if __name__ == '__main__':
    main()