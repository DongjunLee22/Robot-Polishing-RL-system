# residual_rl_method1_final.py - TEST VERSION
import os
import random
import time
import json
import socket
import struct
import threading
from collections import deque

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim

# =========================
# CONFIG - TEST VERSION
# =========================
CONFIG = {
    "STATE_DIM": 6,              # 6개 상태 변수로 수정
    "ACTION_DIM": 1,
    "HIDDEN": 256,
    "LR": 3e-4,
    "GAMMA": 0.99,
    "TAU": 0.005,
    "AUTO_ENTROPY": True,

    # Residual limits (MPa)
    "R_MIN": -0.1,
    "R_MAX":  0.1,
    "R_SLEW_PER_20MS": 0.02,     # |Δresidual| limit per 20ms tick

    # Scheduling
    "TICK_SEC": 0.020,
    "TICK_TOL": 0.003,

    # Training - TEST VERSION
    "BATCH_SIZE": 64,             # 테스트용으로 줄임
    "REPLAY_WARMUP": 100,         # 테스트용으로 줄임

    # Networking
    "HOST": "0.0.0.0",
    "PORT": 8888,
    "RECV_TIMEOUT_SEC": 0.5,
    "COMM_FAIL_MAX": 3,

    # Episode - TEST VERSION
    "EPISODES": 10,               # 테스트용으로 10에피소드
    "MAX_EPISODE_STEPS": 2000,    # 테스트용: 40초 (2000 스텝)

    # Safety / Reward shaping
    "MAX_FORCE_ERR": 15.0,
    "MAX_PRESS_DELTA": 0.05,

    # Logging
    "LOG_EVERY_CTRL": 25,         # 테스트용으로 더 자주 로깅
    "SAVE_THRESH_FREQ": 5,        # 테스트용으로 더 자주 저장
    "SAVE_THRESH_PCT": 80,        # 테스트용으로 낮춤
    
    # Model saving
    "MODEL_SAVE_DIR": "saved_agents",  # 모델 저장 전용 폴더
}

# =========================
# Utils: seed
# =========================
def set_seed(seed=42):
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(seed)

# =========================
# SAC Models
# =========================
class Actor(nn.Module):
    def __init__(self, state_dim, action_dim, hidden_dim=256, log_std_min=-20, log_std_max=2):
        super().__init__()
        self.log_std_min = log_std_min
        self.log_std_max = log_std_max
        self.fc1 = nn.Linear(state_dim, hidden_dim)
        self.fc2 = nn.Linear(hidden_dim, hidden_dim)
        self.mean_head = nn.Linear(hidden_dim, action_dim)
        self.log_std_head = nn.Linear(hidden_dim, action_dim)

    def forward(self, state):
        x = F.relu(self.fc1(state))
        x = F.relu(self.fc2(x))
        mean = self.mean_head(x)
        log_std = torch.clamp(self.log_std_head(x), self.log_std_min, self.log_std_max)
        return mean, log_std

    def sample(self, state):
        mean, log_std = self.forward(state)
        std = log_std.exp()
        normal = torch.distributions.Normal(mean, std)
        x_t = normal.rsample()
        action = torch.tanh(x_t)
        
        # log_prob 계산 - 원본 코드와 동일하게 수정
        log_prob = normal.log_prob(x_t) - torch.log(1 - action.pow(2) + 1e-6)
        log_prob_sum = log_prob.sum(1, keepdim=True)  # (batch_size, 1) 차원
        
        return action, log_prob_sum

class Critic(nn.Module):
    def __init__(self, state_dim, action_dim, hidden_dim=256):
        super().__init__()
        self.q1_fc1 = nn.Linear(state_dim + action_dim, hidden_dim)
        self.q1_fc2 = nn.Linear(hidden_dim, hidden_dim)
        self.q1_fc3 = nn.Linear(hidden_dim, 1)
        self.q2_fc1 = nn.Linear(state_dim + action_dim, hidden_dim)
        self.q2_fc2 = nn.Linear(hidden_dim, hidden_dim)
        self.q2_fc3 = nn.Linear(hidden_dim, 1)

    def forward(self, state, action):
        sa = torch.cat([state, action], 1)
        q1 = F.relu(self.q1_fc1(sa))
        q1 = F.relu(self.q1_fc2(q1))
        q1 = self.q1_fc3(q1)
        q2 = F.relu(self.q2_fc1(sa))
        q2 = F.relu(self.q2_fc2(q2))
        q2 = self.q2_fc3(q2)
        return q1, q2

class ReplayBuffer:
    def __init__(self, capacity=100000):
        self.buffer = deque(maxlen=capacity)
    def push(self, state, action, reward, next_state, done):
        self.buffer.append((state, action, reward, next_state, done))
    def sample(self, batch_size):
        batch = random.sample(self.buffer, batch_size)
        state, action, reward, next_state, done = map(np.stack, zip(*batch))
        
        # action을 (batch_size, 1) 차원으로 변환
        if action.ndim == 1:
            action = action.reshape(-1, 1)
        
        return state, action, reward, next_state, done
    def __len__(self):
        return len(self.buffer)

class ResidualSACAgent:
    def __init__(self, cfg=CONFIG):
        self.cfg = cfg
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        s_dim, a_dim, hidden = cfg["STATE_DIM"], cfg["ACTION_DIM"], cfg["HIDDEN"]
        self.gamma, self.tau = cfg["GAMMA"], cfg["TAU"]
        self.alpha = 0.2
        self.auto_entropy_tuning = cfg["AUTO_ENTROPY"]

        self.actor = Actor(s_dim, a_dim, hidden).to(self.device)
        self.critic = Critic(s_dim, a_dim, hidden).to(self.device)
        self.critic_target = Critic(s_dim, a_dim, hidden).to(self.device)
        self.critic_target.load_state_dict(self.critic.state_dict())

        self.actor_opt = optim.Adam(self.actor.parameters(), lr=cfg["LR"])
        self.critic_opt = optim.Adam(self.critic.parameters(), lr=cfg["LR"])

        if self.auto_entropy_tuning:
            self.target_entropy = -torch.prod(torch.tensor([a_dim], device=self.device)).item()
            self.log_alpha = torch.zeros(1, requires_grad=True, device=self.device)
            self.alpha_opt = optim.Adam([self.log_alpha], lr=cfg["LR"])

        self.replay = ReplayBuffer()
        self.total_steps = 0
        self.episode_rewards = []

    def select_action(self, state, evaluate=False):
        state = torch.FloatTensor(state.reshape(1, -1)).to(self.device)
        
        with torch.no_grad():
            if evaluate:
                mean, _ = self.actor(state)
                action = torch.tanh(mean)
            else:
                action, log_prob = self.actor.sample(state)
        
        action = action.cpu().numpy().flatten()
        
        # scale to residual MPa range [-0.1, 0.1]
        return float(action[0] * (self.cfg["R_MAX"]))

    def store_transition(self, state, action, reward, next_state, done):
        # normalize back to [-1,1] for critic input consistency
        norm_action = action / self.cfg["R_MAX"]
        
        # 리플레이버퍼 저장 모니터링
        self.replay.push(state, norm_action, reward, next_state, done)
        
        # 저장된 데이터 개수 추적 (디버깅용)
        if len(self.replay) % 100 == 0:  # 100개마다 로깅
            print(f"📊 ReplayBuffer: {len(self.replay)} transitions stored")

    def update_parameters(self, batch_size=None):
        bs = batch_size or self.cfg["BATCH_SIZE"]
        if len(self.replay) < bs: return
        s, a, r, ns, d = self.replay.sample(bs)
        
        s = torch.FloatTensor(s).to(self.device)
        a = torch.FloatTensor(a).to(self.device)
        r = torch.FloatTensor(r).unsqueeze(1).to(self.device)
        ns = torch.FloatTensor(ns).to(self.device)
        d = torch.FloatTensor(d).unsqueeze(1).to(self.device)

        with torch.no_grad():
            na, nlogp = self.actor.sample(ns)
            # nlogp는 이미 (batch_size, 1) 차원
            q1n, q2n = self.critic_target(ns, na)
            min_qn = torch.min(q1n, q2n) - self.alpha * nlogp
            y = r + (1 - d) * self.gamma * min_qn

        q1, q2 = self.critic(s, a)
        q_loss = F.mse_loss(q1, y) + F.mse_loss(q2, y)
        self.critic_opt.zero_grad()
        q_loss.backward()
        nn.utils.clip_grad_norm_(self.critic.parameters(), 1.0)
        self.critic_opt.step()

        pi, logp = self.actor.sample(s)
        # logp는 이미 (batch_size, 1) 차원
        q1_pi, q2_pi = self.critic(s, pi)
        min_q_pi = torch.min(q1_pi, q2_pi)
        pi_loss = ((self.alpha * logp) - min_q_pi).mean()
        self.actor_opt.zero_grad()
        pi_loss.backward()
        nn.utils.clip_grad_norm_(self.actor.parameters(), 1.0)
        self.actor_opt.step()

        if self.auto_entropy_tuning:
            # logp는 (batch_size, 1)이므로 squeeze(1)로 (batch_size,)로 변환
            logp_entropy = logp.squeeze(1)
            a_loss = -(self.log_alpha * (logp_entropy + self.target_entropy).detach()).mean()
            self.alpha_opt.zero_grad(); a_loss.backward(); self.alpha_opt.step()
            self.alpha = self.log_alpha.exp()

        # soft update
        with torch.no_grad():
            for tp, lp in zip(self.critic_target.parameters(), self.critic.parameters()):
                tp.data.copy_(self.tau * lp.data + (1 - self.tau) * tp.data)

    def save_model(self, path):
        torch.save({
            "actor": self.actor.state_dict(),
            "critic": self.critic.state_dict(),
            "critic_target": self.critic_target.state_dict(),
            "actor_opt": self.actor_opt.state_dict(),
            "critic_opt": self.critic_opt.state_dict(),
            "total_steps": self.total_steps,
            "episode_rewards": self.episode_rewards,
        }, path)
        print(f"💾 Saved: {path}")

# =========================
# TCP Communicator (residual only)
# =========================
class ResidualRLCommunicator:
    def __init__(self, host, port, recv_timeout):
        self.host, self.port = host, port
        self.recv_timeout = recv_timeout
        self.socket = None
        self.conn = None
        self.connected = False
        
        # 패킷 구조 정의 - 6개 float + 1개 bool로 수정
        self.CPP_TO_PY_PACKET_FORMAT = ">HffffffBH"  # Big-Endian, 6개 float + 1개 bool
        self.CPP_TO_PY_PACKET_SIZE = 29  # 2+4×6+1+2 = 29 bytes
        self.CPP_TO_PY_SOF = 0xAAAA
        
        self.PY_TO_CPP_PACKET_FORMAT = ">HfBBH"  # Big-Endian, 2+4+1+1+2 = 10 bytes
        self.PY_TO_CPP_PACKET_SIZE = 10  # 2+4+1+1+2 = 10 bytes
        self.PY_TO_CPP_SOF = 0xBBBB
        
        # 수신 쓰레드 관련
        self.latest_state = None
        self.latest_rl_flag = False
        self.receive_thread = None
        self.is_receiving = False
        self.state_lock = threading.Lock()  # 스레드 안전성
        
        # 모니터링 통계 추가
        self.packets_received = 0
        self.packets_sent = 0
        self.checksum_errors = 0
        self.sof_errors = 0
        self.packet_size_errors = 0
        self.last_packet_time = None
        self.connection_start_time = None
        
        # 🎯 29바이트 데이터 수신 품질 모니터링 추가
        self.expected_packet_interval = 0.001  # 1ms (1000Hz)
        self.packet_receive_times = []  # 최근 100개 패킷 수신 시간
        self.packet_sequence_numbers = []  # 패킷 순서 번호 (최근 100개)
        self.missed_packets = 0  # 누락된 패킷 수
        self.late_packets = 0  # 지연된 패킷 수
        self.max_packet_history = 100  # 최대 기록 개수

    def connect(self):
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.socket.bind((self.host, self.port))
            self.socket.listen(1)
            print(f"🔗 Waiting for Robot PC on {self.host}:{self.port} ...")
            conn, addr = self.socket.accept()
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            conn.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
            conn.settimeout(self.recv_timeout)
            print(f"✅ Connected: {addr}")
            self.conn = conn
            self.connected = True
            
            # 연결 시간 기록
            self.connection_start_time = time.time()
            
            # 연결 후 수신 쓰레드 시작
            self.start_receiving()
            return True
        except Exception as e:
            print(f"❌ Connection error: {e}")
            return False

    def start_receiving(self):
        """1000Hz로 계속 수신하는 별도 쓰레드 시작"""
        self.is_receiving = True
        self.receive_thread = threading.Thread(target=self._receive_loop, daemon=True)
        self.receive_thread.start()
        print("📡 Started 1000Hz receiving thread")

    def _receive_loop(self):
        """1000Hz로 계속 수신하는 메인 루프"""
        while self.is_receiving:
            try:
                # non-blocking 수신 (1ms timeout)
                self.conn.settimeout(0.001)
                data = self._recv_exact(self.CPP_TO_PY_PACKET_SIZE)
                if data:
                    state, rl_flag = self._process_packet(data)
                    if state is not None:
                        with self.state_lock:
                            self.latest_state = state
                            self.latest_rl_flag = rl_flag
                            self.last_packet_time = time.time()
            except socket.timeout:
                continue
            except Exception as e:
                print(f"⚠️ Receive loop error: {e}")
                break
        print("📡 Receive loop terminated")

    def _recv_exact(self, nbytes):
        """정확히 n바이트를 받을 때까지 반복 수신"""
        data = b''
        while len(data) < nbytes:
            chunk = self.conn.recv(nbytes - len(data))
            if not chunk:
                return None
            data += chunk
        return data

    def _process_packet(self, data):
        """수신된 패킷을 처리하여 상태와 플래그 반환"""
        try:
            # 1. 길이 검증
            if len(data) != self.CPP_TO_PY_PACKET_SIZE:
                self.packet_size_errors += 1
                print(f"⚠️ [경고] {self.CPP_TO_PY_PACKET_SIZE}B가 아닌 {len(data)}B 수신 (총 {self.packet_size_errors}회)")
                return None, False
            
            # 2. 언패킹 - 포맷 수정
            try:
                (sof, current_force, target_force, force_error, force_error_dot, 
                 force_error_int, pi_output, sander_active, 
                 received_checksum) = struct.unpack(">HffffffBH", data)
            except struct.error as e:
                print(f"⚠️ [오류] struct.unpack 실패: {e}")
                return None, False
            
            # 3. SOF 검증
            if sof != self.CPP_TO_PY_SOF:
                self.sof_errors += 1
                print(f"⚠️ [오류] SOF 불일치: {hex(sof)} (기대: {hex(self.CPP_TO_PY_SOF)}) (총 {self.sof_errors}회)")
                return None, False
            
            # 4. 체크섬 검증 (CRC-16)
            calculated_checksum = self.calculate_crc16(data[:-2])
            
            if received_checksum != calculated_checksum:
                self.checksum_errors += 1
                print(f"❌ [체크섬 오류] recv:{received_checksum} calc:{calculated_checksum} (총 {self.checksum_errors}회)")
                return None, False
            
            # 5. 상태 배열 구성 - 6개 변수로 수정
            state = np.array([
                current_force,      # 0: RL_currentForceZ
                target_force,       # 1: RL_targetForceZ  
                force_error,        # 2: RL_forceZError
                force_error_dot,    # 3: RL_forceZErrordot
                force_error_int,    # 4: RL_forceZErrorintegral
                pi_output,          # 5: RL_pidFlag (float)
            ], dtype=np.float32)
            
            rl_flag = bool(sander_active)
            
            # 6. 수신 성공 통계 업데이트
            self.packets_received += 1
            
            # 🎯 29바이트 데이터 수신 품질 모니터링
            current_time = time.time()
            
            # 패킷 수신 시간 기록
            self.packet_receive_times.append(current_time)
            if len(self.packet_receive_times) > self.max_packet_history:
                self.packet_receive_times.pop(0)
            
            # 패킷 순서 번호 기록
            self.packet_sequence_numbers.append(self.packets_received)
            if len(self.packet_sequence_numbers) > self.max_packet_history:
                self.packet_sequence_numbers.pop(0)
            
            # 패킷 간격 및 누락 검사 (최소 2개 패킷이 있을 때)
            if len(self.packet_receive_times) >= 2:
                last_interval = self.packet_receive_times[-1] - self.packet_receive_times[-2]
                
                # 지연된 패킷 검사 (1ms보다 늦게 도착)
                if last_interval > self.expected_packet_interval * 1.5:  # 1.5ms 이상
                    self.late_packets += 1
                
                # 누락된 패킷 추정 (간격이 너무 클 때)
                expected_packets = int(last_interval / self.expected_packet_interval)
                if expected_packets > 1:
                    self.missed_packets += (expected_packets - 1)
            
            return state, rl_flag
            
        except Exception as e:
            print(f"⚠️ Packet processing error: {e}")
            return None, False

    def calculate_crc16(self, data: bytes) -> int:
        """CRC-16/MODBUS 체크섬 계산 (C++와 동일)"""
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc = crc >> 1
        return crc



    def get_latest_state(self):
        """메인 루프에서 호출하여 최신 상태 반환 (non-blocking)"""
        with self.state_lock:
            if self.latest_state is not None:
                # sander_active 상태 디버깅을 위한 로깅 추가
                if hasattr(self, 'last_logged_rl_flag') and self.last_logged_rl_flag != self.latest_rl_flag:
                    print(f"🔄 RL Flag Changed: {self.last_logged_rl_flag} -> {self.latest_rl_flag}")
                    self.last_logged_rl_flag = self.latest_rl_flag
                elif not hasattr(self, 'last_logged_rl_flag'):
                    self.last_logged_rl_flag = self.latest_rl_flag
                    print(f"🔄 Initial RL Flag: {self.latest_rl_flag}")
                
                return self.latest_state.copy(), self.latest_rl_flag
        return None, False

    def send_residual(self, rl_residual, timing_accurate, episode_done):
        """50Hz로 residual 전송"""
        try:
            # 1. 10바이트 패킷 구성
            # SOF(2) + rl_residual(4) + timing_accurate(1) + episode_done(1) + checksum(2)
            
            # 2. 체크섬 계산용 데이터 (checksum 제외 부분)
            data_part = struct.pack(">HfBB", 
                                  self.PY_TO_CPP_SOF, 
                                  float(rl_residual), 
                                  int(timing_accurate), 
                                  int(episode_done))
            checksum = self.calculate_crc16(data_part)
            
            # 3. 최종 패킷 (SOF, float, unsigned char, unsigned char, checksum[uint16])
            final_packet = struct.pack(self.PY_TO_CPP_PACKET_FORMAT, 
                                     self.PY_TO_CPP_SOF, 
                                     float(rl_residual), 
                                     int(timing_accurate), 
                                     int(episode_done), 
                                     checksum)
            
            # 4. 송신
            self.conn.sendall(final_packet)
            
            # 5. 송신 성공 통계 업데이트
            self.packets_sent += 1
            
            return True
            
        except Exception as e:
            print(f"⚠️ Error sending residual: {e}")
            return False

    def send_reset(self):
        """에피소드 리셋 명령 전송 (binary protocol)"""
        try:
            # 🎯 binary protocol로 reset 신호 전송
            # SOF(2) + reset_flag(1) + padding(3) + checksum(2) = 8 bytes
            reset_data = struct.pack(">HBxxxH", 0xCCCC, 1, 0)  # 0xCCCC = reset SOF
            checksum = self.calculate_crc16(reset_data[:-2])
            reset_packet = struct.pack(">HBxxxH", 0xCCCC, 1, checksum)
            
            self.conn.sendall(reset_packet)
            return True
        except Exception as e:
            print(f"⚠️ Error sending reset: {e}")
            return False

    def get_communication_stats(self):
        """통신 통계 반환"""
        uptime = time.time() - self.connection_start_time if self.connection_start_time else 0
        
        # 🎯 29바이트 데이터 수신 품질 통계 계산
        packet_quality_stats = {}
        if len(self.packet_receive_times) >= 2:
            # 평균 패킷 간격 계산
            intervals = [self.packet_receive_times[i] - self.packet_receive_times[i-1] 
                        for i in range(1, len(self.packet_receive_times))]
            avg_interval = sum(intervals) / len(intervals)
            packet_quality_stats["avg_packet_interval_ms"] = avg_interval * 1000
            
            # 패킷 간격 표준편차 계산
            interval_variance = sum((x - avg_interval) ** 2 for x in intervals) / len(intervals)
            packet_quality_stats["packet_interval_std_ms"] = (interval_variance ** 0.5) * 1000
            
            # 누락된 패킷 비율 계산
            expected_packets = uptime / self.expected_packet_interval
            packet_quality_stats["packet_loss_rate_percent"] = (self.missed_packets / expected_packets * 100) if expected_packets > 0 else 0
        
        return {
            "uptime_seconds": uptime,
            "packets_received": self.packets_received,
            "packets_sent": self.packets_sent,
            "checksum_errors": self.checksum_errors,
            "sof_errors": self.sof_errors,
            "packet_size_errors": self.packet_size_errors,
            "last_packet_time": self.last_packet_time,
            "receive_rate_hz": self.packets_received / uptime if uptime > 0 else 0,
            "send_rate_hz": self.packets_sent / uptime if uptime > 0 else 0,
            # 🎯 29바이트 데이터 품질 통계 추가
            "missed_packets": self.missed_packets,
            "late_packets": self.late_packets,
            **packet_quality_stats
        }

    def print_communication_stats(self):
        """통신 통계 출력"""
        stats = self.get_communication_stats()
        print("\n📊 === COMMUNICATION STATISTICS ===")
        print(f"⏱️  Uptime: {stats['uptime_seconds']:.1f}s")
        print(f"📥 Packets Received: {stats['packets_received']}")
        print(f"📤 Packets Sent: {stats['packets_sent']}")
        print(f"📥 Receive Rate: {stats['receive_rate_hz']:.1f} Hz")
        print(f"📤 Send Rate: {stats['send_rate_hz']:.1f} Hz")
        print(f"❌ Checksum Errors: {stats['checksum_errors']}")
        print(f"⚠️  SOF Errors: {stats['sof_errors']}")
        print(f"📏 Packet Size Errors: {stats['packet_size_errors']}")
        
        # 🎯 29바이트 데이터 수신 품질 상세 정보
        if 'avg_packet_interval_ms' in stats:
            print(f"\n🎯 === 29-BYTE DATA QUALITY ===")
            print(f"📊 Avg Packet Interval: {stats['avg_packet_interval_ms']:.3f}ms (Target: 1.000ms)")
            print(f"📊 Packet Interval Std: {stats['packet_interval_std_ms']:.3f}ms")
            print(f"📊 Missed Packets: {stats['missed_packets']}")
            print(f"📊 Late Packets: {stats['late_packets']}")
            print(f"📊 Packet Loss Rate: {stats['packet_loss_rate_percent']:.2f}%")
            
            # 품질 평가
            if stats['packet_loss_rate_percent'] < 0.1:
                quality = "🟢 EXCELLENT"
            elif stats['packet_loss_rate_percent'] < 1.0:
                quality = "🟡 GOOD"
            elif stats['packet_loss_rate_percent'] < 5.0:
                quality = "🟠 FAIR"
            else:
                quality = "🔴 POOR"
            print(f"📊 Overall Quality: {quality}")
        
        if stats['last_packet_time']:
            time_since_last = time.time() - stats['last_packet_time']
            print(f"\n🕐 Last Packet: {time_since_last:.3f}s ago")
        print("=" * 40)

    def close(self):
        """연결 종료 및 리소스 정리"""
        try:
            # 수신 쓰레드 종료
            self.is_receiving = False
            if self.receive_thread and self.receive_thread.is_alive():
                self.receive_thread.join(timeout=1.0)
            
            # 소켓 종료
            if self.conn: 
                self.conn.close()
            if self.socket: 
                self.socket.close()
        finally:
            self.connected = False
            print("🔌 Communication closed")

# =========================
# Environment (Method 1) - TEST VERSION
# =========================
class PneumaticPolishingEnvironment:
    def __init__(self, cfg=CONFIG):
        self.cfg = cfg
        self.agent = ResidualSACAgent(cfg)
        self.comm = ResidualRLCommunicator(cfg["HOST"], cfg["PORT"], cfg["RECV_TIMEOUT_SEC"])

        # scheduler
        self.last_tick = None

        # residual limiter
        self.prev_residual = 0.0

        # episode stats
        self.episode_step = 0
        self.max_episode_steps = cfg["MAX_EPISODE_STEPS"]
        self.current_episode_reward = 0.0
        self.best_episode_reward = -float("inf")
        self.best_agent_episode = -1

        # RL activity monitor
        self.rl_inactive_count = 0
        self.max_rl_inactive_steps = 250
        self.rl_active_in_episode = False
        self.total_rl_active_steps = 0

        # comm fail
        self.fail_count = 0
        self.FAIL_MAX = cfg["COMM_FAIL_MAX"]

        # logging
        self.control_updates = 0
        self.last_log_time = None  # 시간 기반 로깅을 위한 변수 추가

    # ---- scheduler ----
    def should_send_now(self):
        now = time.time()
        if self.last_tick is None:
            self.last_tick = now
            return True, True
        
        dt = now - self.last_tick
        
        # 50Hz = 20ms 간격
        if dt >= self.cfg["TICK_SEC"]:
            is_exact = abs(dt - self.cfg["TICK_SEC"]) <= self.cfg["TICK_TOL"]
            self.last_tick = now
            
            # 50Hz 타이밍 디버깅 (1초마다)
            if int(now) % 1 == 0 and int(self.last_tick) % 1 == 0:
                print(f"⏱️ Scheduler: dt={dt*1000:.1f}ms, exact={is_exact}")
            
            return True, is_exact
        
        return False, False

    # ---- residual limiter ----
    def limit_residual(self, r):
        r = float(np.clip(r, self.cfg["R_MIN"], self.cfg["R_MAX"]))
        delta = np.clip(r - self.prev_residual, -self.cfg["R_SLEW_PER_20MS"], self.cfg["R_SLEW_PER_20MS"])
        r_limited = self.prev_residual + delta
        self.prev_residual = r_limited
        return r_limited

    # ---- reward / done ----
    def calculate_reward(self, state, action_residual, rl_flag):
        current_force, target_force = state[0], state[1]
        force_err = abs(current_force - target_force)
        residual_change = abs(action_residual - self.prev_residual)
        # 1) tracking
        reward = -(force_err / self.cfg["MAX_FORCE_ERR"])
        if force_err < 1.0: reward += 0.5
        # 2) smoothness (rl_flag에 따라 가중치 조정)
        smooth_w = 0.2 if rl_flag else 0.3
        reward += -smooth_w * (residual_change / self.cfg["MAX_PRESS_DELTA"])
        # 3) safety
        if current_force > 80.0: reward += -5.0
        # 4) residual magnitude penalty
        reward += -0.1 * abs(action_residual)
        return float(reward)

    def is_done(self, state):
        # 🎯 오직 최대 스텝에 도달했을 때만 True (에피소드 끝까지 진행)
        if self.episode_step >= self.max_episode_steps: 
            return True
        
        # 🚨 안전장치: 접촉력이 과도하게 높을 때만 종료
        if state[0] > 100.0: 
            print(f"⚠️ Safety: Force too high ({state[0]:.1f}N > 100N) - Episode terminated")
            return True
        
        # ✅ 다른 모든 경우: 계속 진행 (목표 접촉력 달성해도 계속)
        return False

    # ---- RL activity monitor ----
    def check_rl_status(self, rl_flag):
        if rl_flag:
            self.rl_inactive_count = 0
            self.rl_active_in_episode = True
            self.total_rl_active_steps += 1
            return "active"
        else:
            self.rl_inactive_count += 1
            if self.rl_inactive_count >= self.max_rl_inactive_steps:
                print("❌ RL inactive too long → terminate episode")
                return "terminate"
            return "inactive"

    # ---- episode helpers ----
    def reset_episode(self):
        self.prev_residual = 0.0
        self.episode_step = 0
        self.current_episode_reward = 0.0
        self.control_updates = 0
        self.rl_inactive_count = 0
        self.rl_active_in_episode = False
        self.last_tick = None
        self.last_log_time = None  # 로깅 시간 리셋
        ok = self.comm.send_reset()
        if ok:
            print("\n--- Episode Reset ---")
            print("Robot PC: 1kHz PI running, will add RL residual (held) each tick.")
        else:
            print("⚠️ Reset signal failed (continuing).")
        return ok

    # ---- main loop ----
    def run_training(self, episodes=None):
        episodes = episodes or self.cfg["EPISODES"]
        if not self.comm.connect():
            print("Failed to connect to Robot PC")
            return

        # saved_agents 폴더 생성
        model_save_dir = self.cfg["MODEL_SAVE_DIR"]
        os.makedirs(model_save_dir, exist_ok=True)
        print(f"📁 Model save directory: {model_save_dir}")

        print("🚀 TEST VERSION: TRUE Residual RL - 10 episodes, 100 seconds each")
        print("📡 RL sends residual at 50Hz, Robot sums PI(1kHz)+Residual.")
        print("⏱️  Each episode: 2,000 steps (40 seconds) - 목표 접촉력 달성해도 계속 진행")
        print("🎯 Episode completion: Only when max steps (2,000) reached")
        
        # 🎯 RL Flag가 True가 될 때까지 대기 (로봇이 Control Step 2에 도달할 때까지)
        print("\n🔄 Waiting for Robot to reach Control Step 2 (RL activation)...")
        print("📋 Robot Control Sequence:")
        print("   Step 0: Robot descending to mold (position control)")
        print("   Step 1: Contact force stabilization (5 seconds)")
        print("   Step 2: Sander ON + PI control + RL activation")
        
        wait_start_time = time.time()
        while True:
            state, rl_flag = self.comm.get_latest_state()
            if rl_flag:  # sander_active = True
                wait_duration = time.time() - wait_start_time
                print(f"🎯 RL Activated! (waited {wait_duration:.1f}s)")
                print("🚀 Starting RL episodes...")
                break
            
            # 대기 중 상태 표시
            if state is not None:
                current_force = state[0]
                target_force = state[1]
                print(f"⏳ Waiting... Current Force: {current_force:.1f}N, Target: {target_force:.1f}N", end='\r')
            
            time.sleep(0.1)  # 100ms 대기
            
            # 무한 대기 방지 (5분 후 타임아웃)
            if time.time() - wait_start_time > 300:
                print(f"\n⚠️ Timeout! Robot didn't reach Control Step 2 in 5 minutes")
                print("🔍 Check robot control settings and physical contact")
                return
        
        # 에피소드별 통계
        episode_stats = []
        
        for ep in range(episodes):
            print(f"\n🎬 === EPISODE {ep+1}/{episodes} START ===")
            
            # 🎯 에피소드 시작 시 RL Flag 상태 확인
            episode_start_state, episode_start_rl_flag = self.comm.get_latest_state()
            if not episode_start_rl_flag:
                print(f"⚠️ Warning: RL Flag is False at episode {ep+1} start")
                print("🔄 Waiting for RL activation...")
                wait_start = time.time()
                while not episode_start_rl_flag:
                    episode_start_state, episode_start_rl_flag = self.comm.get_latest_state()
                    if time.time() - wait_start > 60:  # 1분 타임아웃
                        print(f"⚠️ Timeout waiting for RL activation in episode {ep+1}")
                        break
                    time.sleep(0.1)
            
            episode_start_time = time.time()
            self.reset_episode()

            # 새 에피소드 시작 신호 전송
            print(f"📡 Starting episode {ep+1} - sending episode_done=False")
            self.comm.send_residual(0.0, True, False)

            prev_state = None
            prev_action = None
            prev_rl_flag = False
            
            # 에피소드별 통계
            episode_packets_received = 0
            episode_packets_sent = 0
            episode_rl_active_steps = 0

            while True:
                # 수신된 최신 상태 가져오기 (non-blocking)
                res = self.comm.get_latest_state()
                if res[0] is None:
                    # 아직 수신된 상태가 없음
                    send_now, timing_ok = self.should_send_now()
                    if not send_now:
                        continue
                    # 상태가 없으면 대기
                    time.sleep(0.001)  # 1ms 대기
                    continue

                state, rl_flag = res
                episode_packets_received += 1

                send_now, timing_ok = self.should_send_now()
                if not send_now:
                    continue
                
                # 🎯 50Hz 기준으로만 스텝 증가 (1000Hz가 아닌)
                self.episode_step += 1

                # (1) store prev transition & learn on 50Hz boundary
                if prev_state is not None and prev_rl_flag:
                    reward = self.calculate_reward(prev_state, prev_action, prev_rl_flag)
                    done = self.is_done(state)  # next_state-based done
                    self.agent.store_transition(prev_state, prev_action, reward, state, done)
                    self.current_episode_reward += reward
                    if len(self.agent.replay) > self.cfg["REPLAY_WARMUP"]:
                        self.agent.update_parameters(self.cfg["BATCH_SIZE"])
                    
                    # 🎯 done 체크는 아래에서 통합 처리 (중복 제거)

                # (2) compute residual (or zero if inactive)
                rl_status = self.check_rl_status(rl_flag)
                if rl_status == "terminate":
                    print("Episode terminated due to prolonged RL inactivity")
                    break

                # 🎯 에피소드 종료 시에만 episode_done = True 전송
                if self.episode_step >= self.max_episode_steps:
                    episode_done = True
                    rl_residual = 0.0  # 종료 시 residual은 0
                    print(f"🎯 Episode {ep+1} ending at step {self.episode_step} - sending episode_done=True")
                else:
                    episode_done = False
                    if rl_flag:
                        # 🎯 50Hz로만 select_action 호출 (1000Hz가 아닌)
                        if send_now:  # send_now가 True일 때만
                            raw_res = self.agent.select_action(state, evaluate=False)
                            rl_residual = self.limit_residual(raw_res)
                            episode_rl_active_steps += 1
                        else:
                            # send_now가 False면 이전 residual 유지
                            rl_residual = prev_action if prev_action is not None else 0.0
                    else:
                        rl_residual = 0.0

                # (3) send residual (50Hz)
                ok = self.comm.send_residual(rl_residual, timing_ok, episode_done)
                if not ok:
                    self.fail_count += 1
                    print(f"⚠️ Send failed ({self.fail_count}/{self.FAIL_MAX})")
                    if self.fail_count >= self.FAIL_MAX:
                        # advise PI-only fallback by ending episode with residual=0
                        self.comm.send_residual(0.0, False, True)
                        print("Comms degraded → advise PI-only fallback on Robot PC; ending episode.")
                        break
                else:
                    self.fail_count = 0
                    episode_packets_sent += 1
                    
                    # 50Hz 전송 모니터링 (1초마다)
                    current_time = time.time()
                    if int(current_time) % 1 == 0 and int(current_time - episode_start_time) % 1 == 0:
                        print(f"📡 50Hz Send: residual={rl_residual:.3f}MPa, episode_done={episode_done}, timing_ok={timing_ok}")

                # 🎯 에피소드 종료 시 즉시 break
                if episode_done:
                    print(f"🎯 Episode {ep+1} completed at step {self.episode_step:,}")
                    
                    # 에피소드 종료 후 명확한 신호 전송
                    print(f"📡 Sending final episode_done=True for episode {ep+1}")
                    self.comm.send_residual(0.0, True, True)
                    
                    break

                # (4) logging (light) - 2.5초마다 로깅
                current_time = time.time()
                if (self.last_log_time is None or 
                    current_time - self.last_log_time >= 2.5):  # 2.5초마다 로깅
                    mode = "RESIDUAL" if rl_flag else "PI-ONLY"
                    
                    # 🎯 목표 접촉력 달성 시 특별 표시
                    force_achieved = ""
                    if abs(state[0] - state[1]) < 0.5:  # 목표 ±0.5N 달성
                        force_achieved = " 🎯 TARGET ACHIEVED!"
                    
                    print(f"[Ep {ep+1}/10] Step {self.episode_step} | {mode} | "
                          f"F {state[0]:.1f}/{state[1]:.1f}N | "
                          f"PI {state[5]:.3f}MPa | RL {rl_residual:.3f}MPa | "
                          f"{'EXACT' if timing_ok else 'DELAY'} | "
                          f"Time: {current_time - episode_start_time:.1f}s | "
                          f"RL_Flag: {rl_flag}{force_achieved}")
                    self.last_log_time = current_time
                self.control_updates += 1

                # (5) keep for next transition
                prev_state = state.copy()
                prev_action = rl_residual
                prev_rl_flag = rl_flag

                # 🎯 에피소드 종료는 위에서 처리됨 (episode_done 체크)
                # 중복된 break 로직 제거

            # ---- episode end ----
            episode_duration = time.time() - episode_start_time
            
            # 에피소드 통계 저장
            episode_stat = {
                "episode": ep + 1,
                "duration": episode_duration,
                "steps": self.episode_step,
                "reward": self.current_episode_reward,
                "packets_received": episode_packets_received,
                "packets_sent": episode_packets_sent,
                "rl_active_steps": episode_rl_active_steps,
                "rl_active_ratio": episode_rl_active_steps / self.episode_step if self.episode_step > 0 else 0
            }
            episode_stats.append(episode_stat)
            
            self.agent.episode_rewards.append(self.current_episode_reward)
            if self.current_episode_reward > self.best_episode_reward:
                self.best_episode_reward = self.current_episode_reward
                self.best_agent_episode = ep
                self.agent.save_model(f"{self.cfg['MODEL_SAVE_DIR']}/test_best_agent_episode_{ep+1}_reward_{self.best_episode_reward:.2f}.pth")

            # dynamic threshold saving (테스트용으로 더 자주)
            if ep % self.cfg["SAVE_THRESH_FREQ"] == 0 and ep > 0:
                recent = self.agent.episode_rewards[-self.cfg["SAVE_THRESH_FREQ"]:]
                th = np.percentile(recent, self.cfg["SAVE_THRESH_PCT"])
                if self.current_episode_reward >= th:
                    self.agent.save_model(f"{self.cfg['MODEL_SAVE_DIR']}/test_high_perf_ep_{ep+1}_reward_{self.current_episode_reward:.2f}.pth")

            # 에피소드 완료 요약
            print(f"\n🎯 === EPISODE {ep+1}/10 COMPLETED ===")
            print(f"⏱️  Duration: {episode_duration:.1f}s")
            print(f"📊 Steps: {self.episode_step:,}/{self.max_episode_steps:,} ({self.episode_step/self.max_episode_steps*100:.1f}%)")
            print(f"🏆 Reward: {self.current_episode_reward:.2f}")
            print(f"📥 Packets Received: {episode_packets_received}")
            print(f"📤 Packets Sent: {episode_packets_sent}")
            print(f"🤖 RL Active Steps: {episode_rl_active_steps} ({episode_stat['rl_active_ratio']*100:.1f}%)")
            print(f"📈 Best So Far: {self.best_episode_reward:.2f}")
            
            # 🎯 에피소드 완료 이유 표시
            if self.episode_step >= self.max_episode_steps:
                print(f"✅ Completed: Reached maximum episode steps ({self.max_episode_steps:,})")
            else:
                print("⚠️  Completed: Episode terminated early (safety or error)")
            print("=" * 40)
            
            # 통신 상태 모니터링 (5에피소드마다)
            if (ep + 1) % 5 == 0:
                self.comm.print_communication_stats()

        # ---- 전체 학습 완료 ----
        print("\n🎯 TEST Training finished!")
        print(f"✅ Completed {episodes} episodes successfully")
        print(f"🏆 Best Episode: {self.best_agent_episode+1}, Best Reward: {self.best_episode_reward:.2f}")
        print(f"📈 Total RL active steps: {self.total_rl_active_steps}")
        
        # 전체 통계 요약
        print("\n📊 === FINAL TRAINING SUMMARY ===")
        total_duration = sum(ep["duration"] for ep in episode_stats)
        total_packets_received = sum(ep["packets_received"] for ep in episode_stats)
        total_packets_sent = sum(ep["packets_sent"] for ep in episode_stats)
        avg_reward = np.mean([ep["reward"] for ep in episode_stats])
        
        print(f"⏱️  Total Duration: {total_duration:.1f}s")
        print(f"📥 Total Packets Received: {total_packets_received}")
        print(f"📤 Total Packets Sent: {total_packets_sent}")
        print(f"📊 Average Reward: {avg_reward:.2f}")
        print(f"📈 Best Reward: {self.best_episode_reward:.2f}")
        print(f"🤖 Total RL Active Steps: {self.total_rl_active_steps}")
        
        # 에피소드별 상세 통계
        print("\n📋 === EPISODE DETAILS ===")
        for ep_stat in episode_stats:
            print(f"Ep {ep_stat['episode']:2d}: "
                  f"Reward {ep_stat['reward']:6.2f}, "
                  f"Steps {ep_stat['steps']:4d}, "
                  f"RL Active {ep_stat['rl_active_steps']:4d} "
                  f"({ep_stat['rl_active_ratio']*100:4.1f}%)")
        
        # 최종 통신 통계
        self.comm.print_communication_stats()
        
        self.comm.close()

# =========================
# Main - TEST VERSION
# =========================
if __name__ == "__main__":
    print("🧪 TEST VERSION: JY_Pneumatic_SAC_Pre_only_test.py")
    print("=" * 60)
    
    set_seed(42)
    env = PneumaticPolishingEnvironment(CONFIG)
    
    try:
        print("🚀 Starting test training...")
        env.run_training(CONFIG["EPISODES"])
        print("✅ Test completed successfully!")
        
    except KeyboardInterrupt:
        print("\n⚠️ Interrupted by user.")
        env.comm.close()
        
    except Exception as e:
        print(f"\n❌ Test failed with error: {e}")
        env.comm.close()
        
    finally:
        print("🔚 Test program terminated.")
