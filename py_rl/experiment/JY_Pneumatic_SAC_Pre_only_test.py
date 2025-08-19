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
    "STATE_DIM": 7,              # current_pressure 제거하여 7개로 변경
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
    "MAX_EPISODE_STEPS": 1000,    # 테스트용으로 줄임 (~20초)

    # Safety / Reward shaping
    "MAX_FORCE_ERR": 15.0,
    "MAX_PRESS_DELTA": 0.05,

    # Logging
    "LOG_EVERY_CTRL": 25,         # 테스트용으로 더 자주 로깅
    "SAVE_THRESH_FREQ": 5,        # 테스트용으로 더 자주 저장
    "SAVE_THRESH_PCT": 80,        # 테스트용으로 낮춤
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
        log_prob = normal.log_prob(x_t) - torch.log(1 - action.pow(2) + 1e-6)
        return action, log_prob.sum(1, keepdim=True)

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
                action, _ = self.actor.sample(state)
        action = action.cpu().numpy().flatten()
        # scale to residual MPa range [-0.1, 0.1]
        return float(action[0] * (self.cfg["R_MAX"]))

    def store_transition(self, state, action, reward, next_state, done):
        # normalize back to [-1,1] for critic input consistency
        norm_action = action / self.cfg["R_MAX"]
        self.replay.push(state, norm_action, reward, next_state, done)

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
        q1_pi, q2_pi = self.critic(s, pi)
        min_q_pi = torch.min(q1_pi, q2_pi)
        pi_loss = ((self.alpha * logp) - min_q_pi).mean()
        self.actor_opt.zero_grad()
        pi_loss.backward()
        nn.utils.clip_grad_norm_(self.actor.parameters(), 1.0)
        self.actor_opt.step()

        if self.auto_entropy_tuning:
            a_loss = -(self.log_alpha * (logp + self.target_entropy).detach()).mean()
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
        
        # 패킷 구조 정의 - current_pressure 제거하여 7개 상태 변수
        self.CPP_TO_PY_PACKET_FORMAT = ">HfffffffBH"  # Big-Endian
        self.CPP_TO_PY_PACKET_SIZE = 29  # 2+4+4+4+4+4+4+1+2 = 29 bytes
        self.CPP_TO_PY_SOF = 0xAAAA
        
        self.PY_TO_CPP_PACKET_FORMAT = ">HffBH"  # Big-Endian
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
            
            # 2. 언패킹
            try:
                (sof, current_force, target_force, force_error, force_error_dot, 
                 force_error_int, pi_output, sander_active, 
                 received_checksum) = struct.unpack(self.CPP_TO_PY_PACKET_FORMAT, data)
            except struct.error as e:
                print(f"⚠️ [오류] struct.unpack 실패: {e}")
                return None, False
            
            # 3. SOF 검증
            if sof != self.CPP_TO_PY_SOF:
                self.sof_errors += 1
                print(f"⚠️ [오류] SOF 불일치: {hex(sof)} (기대: {hex(self.CPP_TO_PY_SOF)}) (총 {self.sof_errors}회)")
                return None, False
            
            # 4. 체크섬 검증
            calculated_checksum = self.simple_xor_checksum(data[:-2]) & 0xFF
            received_checksum_u8 = received_checksum & 0xFF
            
            if received_checksum_u8 != calculated_checksum:
                self.checksum_errors += 1
                print(f"❌ [체크섬 오류] recv:{received_checksum_u8} calc:{calculated_checksum} (총 {self.checksum_errors}회)")
                return None, False
            
            # 5. 상태 배열 구성
            state = np.array([
                current_force,      # 0: RL_currentForceZ
                target_force,       # 1: RL_targetForceZ  
                force_error,        # 2: RL_forceZError
                force_error_dot,    # 3: RL_forceZErrordot
                force_error_int,    # 4: RL_forceZErrorintegral
                pi_output,          # 5: RL_pidFlag (float)
                sander_active       # 6: RL_sanderactiveFlag
            ], dtype=np.float32)
            
            rl_flag = bool(sander_active)
            
            # 6. 수신 성공 통계 업데이트
            self.packets_received += 1
            
            return state, rl_flag
            
        except Exception as e:
            print(f"⚠️ Packet processing error: {e}")
            return None, False

    def simple_xor_checksum(self, data: bytes) -> int:
        """간단한 XOR 체크섬 (0~255)"""
        checksum = 0
        for byte in data:
            checksum ^= byte
        return checksum & 0xFF

    def get_latest_state(self):
        """메인 루프에서 호출하여 최신 상태 반환 (non-blocking)"""
        with self.state_lock:
            if self.latest_state is not None:
                return self.latest_state.copy(), self.latest_rl_flag
        return None, False

    def send_residual(self, rl_residual, timing_accurate, episode_done):
        """50Hz로 residual 전송"""
        try:
            # 1. 10바이트 패킷 구성
            # SOF(2) + rl_residual(4) + timing_accurate(1) + episode_done(1) + checksum(2)
            
            # 2. 체크섬 계산용 데이터 (checksum 제외 부분)
            data_part = struct.pack(">HffB", 
                                  self.PY_TO_CPP_SOF, 
                                  float(rl_residual), 
                                  float(timing_accurate), 
                                  int(episode_done))
            checksum = self.simple_xor_checksum(data_part) & 0xFF
            
            # 3. 최종 패킷 (SOF, float, float, unsigned char, checksum[uint16])
            final_packet = struct.pack(self.PY_TO_CPP_PACKET_FORMAT, 
                                     self.PY_TO_CPP_SOF, 
                                     float(rl_residual), 
                                     float(timing_accurate), 
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
        """에피소드 리셋 명령 전송 (JSON 유지)"""
        try:
            payload = {"command": "reset_episode"}
            data = json.dumps(payload).encode("utf-8")
            self.conn.send(len(data).to_bytes(4, "little"))
            self.conn.send(data)
            return True
        except Exception as e:
            print(f"⚠️ Error sending reset: {e}")
            return False

    def get_communication_stats(self):
        """통신 통계 반환"""
        uptime = time.time() - self.connection_start_time if self.connection_start_time else 0
        return {
            "uptime_seconds": uptime,
            "packets_received": self.packets_received,
            "packets_sent": self.packets_sent,
            "checksum_errors": self.checksum_errors,
            "sof_errors": self.sof_errors,
            "packet_size_errors": self.packet_size_errors,
            "last_packet_time": self.last_packet_time,
            "receive_rate_hz": self.packets_received / uptime if uptime > 0 else 0,
            "send_rate_hz": self.packets_sent / uptime if uptime > 0 else 0
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
        if stats['last_packet_time']:
            time_since_last = time.time() - stats['last_packet_time']
            print(f"🕐 Last Packet: {time_since_last:.3f}s ago")
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
        if dt >= self.cfg["TICK_SEC"] - 1e-3:
            is_exact = abs(dt - self.cfg["TICK_SEC"]) <= self.cfg["TICK_TOL"]
            self.last_tick = now
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
    def calculate_reward(self, state, action_residual):
        current_force, target_force = state[0], state[1]
        force_err = abs(current_force - target_force)
        sander_active = bool(state[6])
        residual_change = abs(action_residual - self.prev_residual)
        # 1) tracking
        reward = -(force_err / self.cfg["MAX_FORCE_ERR"])
        if force_err < 1.0: reward += 0.5
        # 2) smoothness
        smooth_w = 0.2 if sander_active else 0.3
        reward += -smooth_w * (residual_change / self.cfg["MAX_PRESS_DELTA"])
        # 3) safety
        if current_force > 80.0: reward += -5.0
        # 4) residual magnitude penalty
        reward += -0.1 * abs(action_residual)
        return float(reward)

    def is_done(self, state):
        if self.episode_step >= self.max_episode_steps: return True
        if state[0] > 100.0: return True  # force safety
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

        print("🚀 TEST VERSION: TRUE Residual RL - 10 episodes, shorter steps")
        print("📡 RL sends residual at 50Hz, Robot sums PI(1kHz)+Residual.")
        print("⏱️  Each episode: ~20 seconds (1000 steps)")
        
        # 에피소드별 통계
        episode_stats = []
        
        for ep in range(episodes):
            print(f"\n🎬 === EPISODE {ep+1}/{episodes} START ===")
            episode_start_time = time.time()
            
            self.reset_episode()

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
                self.episode_step += 1
                episode_packets_received += 1

                send_now, timing_ok = self.should_send_now()
                if not send_now:
                    continue

                # (1) store prev transition & learn on 50Hz boundary
                if prev_state is not None and prev_rl_flag:
                    reward = self.calculate_reward(prev_state, prev_action)
                    done = self.is_done(state)  # next_state-based done
                    self.agent.store_transition(prev_state, prev_action, reward, state, done)
                    self.current_episode_reward += reward
                    if len(self.agent.replay) > self.cfg["REPLAY_WARMUP"]:
                        self.agent.update_parameters(self.cfg["BATCH_SIZE"])
                    if done:
                        self.comm.send_residual(0.0, timing_ok, True)
                        episode_packets_sent += 1
                        break

                # (2) compute residual (or zero if inactive)
                rl_status = self.check_rl_status(rl_flag)
                if rl_status == "terminate":
                    print("Episode terminated due to prolonged RL inactivity")
                    break

                if rl_flag:
                    raw_res = self.agent.select_action(state, evaluate=False)
                    rl_residual = self.limit_residual(raw_res)
                    episode_done = self.is_done(state)
                    episode_rl_active_steps += 1
                else:
                    rl_residual = 0.0
                    episode_done = False

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

                # (4) logging (light) - 2.5초마다 로깅
                current_time = time.time()
                if (self.last_log_time is None or 
                    current_time - self.last_log_time >= 2.5):  # 2.5초마다 로깅
                    mode = "RESIDUAL" if rl_flag else "PI-ONLY"
                    print(f"[Ep {ep+1}/10] Step {self.episode_step} | {mode} | "
                          f"F {state[0]:.1f}/{state[1]:.1f}N | "
                          f"PI {state[5]:.3f}MPa | RL {rl_residual:.3f}MPa | "
                          f"{'EXACT' if timing_ok else 'DELAY'} | "
                          f"Time: {current_time - episode_start_time:.1f}s")
                    self.last_log_time = current_time
                self.control_updates += 1

                # (5) keep for next transition
                prev_state = state.copy()
                prev_action = rl_residual
                prev_rl_flag = rl_flag

                if episode_done and rl_flag:
                    break

                if self.episode_step >= self.max_episode_steps:
                    break

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
                self.agent.save_model(f"test_best_agent_episode_{ep+1}_reward_{self.best_episode_reward:.2f}.pth")

            # dynamic threshold saving (테스트용으로 더 자주)
            if ep % self.cfg["SAVE_THRESH_FREQ"] == 0 and ep > 0:
                recent = self.agent.episode_rewards[-self.cfg["SAVE_THRESH_FREQ"]:]
                th = np.percentile(recent, self.cfg["SAVE_THRESH_PCT"])
                if self.current_episode_reward >= th:
                    self.agent.save_model(f"test_high_perf_ep_{ep+1}_reward_{self.current_episode_reward:.2f}.pth")

            # 에피소드 완료 요약
            print(f"\n🎯 === EPISODE {ep+1}/10 COMPLETED ===")
            print(f"⏱️  Duration: {episode_duration:.1f}s")
            print(f"📊 Steps: {self.episode_step}")
            print(f"🏆 Reward: {self.current_episode_reward:.2f}")
            print(f"📥 Packets Received: {episode_packets_received}")
            print(f"📤 Packets Sent: {episode_packets_sent}")
            print(f"🤖 RL Active Steps: {episode_rl_active_steps} ({episode_stat['rl_active_ratio']*100:.1f}%)")
            print(f"📈 Best So Far: {self.best_episode_reward:.2f}")
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
