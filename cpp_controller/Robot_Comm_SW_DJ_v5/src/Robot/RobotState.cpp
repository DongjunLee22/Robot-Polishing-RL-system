#include "RobotState.h"
#include <cstring>  // for std::memcpy

using namespace RobotComm;

void RobotState::reset() noexcept {
    std::lock_guard<std::mutex> lk(m_mutex);
    m_tcpPos.fill(0.0f);
	m_flangePos.fill(0.0f);
    m_jointAng.fill(0.0f);
    m_tcpVel.fill(0.0f);
    m_flangeVel.fill(0.0f);
    m_jointVel.fill(0.0f);
	m_initialtcpPos.fill(0.0f);
    m_initialflangePos.fill(0.0f);
	m_initialjointAng.fill(0.0f);
    m_relaxationtcpPos.fill(0.0f);
	m_relaxationflangePos.fill(0.0f);
    m_relaxationjoint.fill(0.0f);
    m_snapshot = {};
}

RobotState::RobotState() noexcept {
    // 생성자 내용
    reset();  // 기존 reset() 함수 호출
}

RobotState::Snapshot RobotState::updateSnapshot() noexcept {
    std::lock_guard<std::mutex> lk(m_mutex);
    m_snapshot = {
        m_tcpPos,
        m_flangePos,
        m_jointAng,
        m_tcpVel,
        m_flangeVel,
        m_jointVel,
        m_desVel
    };
    return m_snapshot;
}

RobotState::Snapshot RobotState::getSnapshot() const noexcept {
    return m_snapshot;
}

void RobotState::withLock(std::function<void(const RobotState&)> cb) const noexcept {
    std::lock_guard<std::mutex> lk(m_mutex);
    cb(*this);
}

// Initial position saving functions
void RobotState::saveInitialtcpPosition(const float* data, size_t sz) noexcept {
    if (!data || sz < 6) return;
    std::lock_guard<std::mutex> lk(m_mutex);
    std::memcpy(m_initialtcpPos.data(), data, 6 * sizeof(float));
}
void RobotState::saveInitialflangePosition(const float* data, size_t sz) noexcept {
    if (!data || sz < 6) return;
    std::lock_guard<std::mutex> lk(m_mutex);
    std::memcpy(m_initialflangePos.data(), data, 6 * sizeof(float));
}
void RobotState::saveInitialjointPosition(const float* data, size_t sz) noexcept {
    if (!data || sz < 6) return;
    std::lock_guard<std::mutex> lk(m_mutex);
    std::memcpy(m_initialjointAng.data(), data, 6 * sizeof(float));
}

// Relaxation position saving functions
void RobotState::saveRelaxationtcpPosition(const float* tcpPos, size_t sz) noexcept {
    if (sz < 6) return;
    std::lock_guard<std::mutex> lk(m_mutex);
    std::memcpy(m_relaxationtcpPos.data(), tcpPos, 6 * sizeof(float));
}
void RobotState::saveRelaxationflangePosition(const float* flangePos, size_t sz) noexcept {
    if (sz < 6) return;
    std::lock_guard<std::mutex> lk(m_mutex);
    std::memcpy(m_relaxationflangePos.data(), flangePos, 6 * sizeof(float));
}
void RobotState::saveRelaxationjointPosition(const float* jointAng, size_t sz) noexcept {
    if (sz < 6) return;
    std::lock_guard<std::mutex> lk(m_mutex);
    std::memcpy(m_relaxationjoint.data(), jointAng, 6 * sizeof(float));
}

// Update functions for raw data
void RobotState::updateAllRawData(const float* tP, const float* fP, const float* jA,
    const float* tV, const float* fV, const float* jV, const float* desV,
    size_t sz) noexcept {
    if (sz < 6) return;
    std::lock_guard<std::mutex> lk(m_mutex);
    std::memcpy(m_tcpPos.data(), tP, 6 * sizeof(float));
    std::memcpy(m_flangePos.data(), fP, 6 * sizeof(float));
    std::memcpy(m_jointAng.data(), jA, 6 * sizeof(float));
    std::memcpy(m_tcpVel.data(), tV, 6 * sizeof(float));
	std::memcpy(m_flangeVel.data(), fV, 6 * sizeof(float));
    std::memcpy(m_jointVel.data(), jV, 6 * sizeof(float));
    std::memcpy(m_desVel.data(), desV, 3 * sizeof(float));
}

void RobotState::updatetcpPosRaw(const float* d, size_t sz) noexcept {
    if (sz < 6) return;
    std::lock_guard<std::mutex> lk(m_mutex);
    std::memcpy(m_tcpPos.data(), d, 6 * sizeof(float));
}
void RobotState::updateflangePosRaw(const float* d, size_t sz) noexcept {
    if (sz < 6) return;
    std::lock_guard<std::mutex> lk(m_mutex);
    std::memcpy(m_flangePos.data(), d, 6 * sizeof(float));
}
void RobotState::updateJointAngRaw(const float* d, size_t sz) noexcept {
    if (sz < 6) return;
    std::lock_guard<std::mutex> lk(m_mutex);
    std::memcpy(m_jointAng.data(), d, 6 * sizeof(float));
}
void RobotState::updatetcpVelRaw(const float* d, size_t sz) noexcept {
    if (sz < 6) return;
    std::lock_guard<std::mutex> lk(m_mutex);
    std::memcpy(m_tcpVel.data(), d, 6 * sizeof(float));
}
void RobotState::updateflangeVelRaw(const float* d, size_t sz) noexcept {
    if (sz < 6) return;
    std::lock_guard<std::mutex> lk(m_mutex);
    std::memcpy(m_flangeVel.data(), d, 6 * sizeof(float));
}
void RobotState::updateJointVelRaw(const float* d, size_t sz) noexcept {
    if (sz < 6) return;
    std::lock_guard<std::mutex> lk(m_mutex);
    std::memcpy(m_jointVel.data(), d, 6 * sizeof(float));
}

void RobotState::updatetcpPosition(const std::array<float, 6>& np) noexcept {
    std::lock_guard<std::mutex> lk(m_mutex);
    m_tcpPos = np;
}
void RobotState::updateflangePosition(const std::array<float, 6>& np) noexcept {
    std::lock_guard<std::mutex> lk(m_mutex);
    m_flangePos = np;
}
void RobotState::updateJointAngles(const std::array<float, 6>& na) noexcept {
    std::lock_guard<std::mutex> lk(m_mutex);
    m_jointAng = na;
}
void RobotState::updatetcpVelocity(const std::array<float, 6>& nv) noexcept {
    std::lock_guard<std::mutex> lk(m_mutex);
    m_tcpVel = nv;
}
void RobotState::updateflangeVelocity(const std::array<float, 6>& nv) noexcept {
    std::lock_guard<std::mutex> lk(m_mutex);
    m_flangeVel = nv;
}
void RobotState::updateJointVelocity(const std::array<float, 6>& nv) noexcept {
    std::lock_guard<std::mutex> lk(m_mutex);
    m_jointVel = nv;
}

#define COPY_ARRAY(member) \
    { std::lock_guard<std::mutex> lk(m_mutex); return member; }

std::array<float, 6> RobotState::gettcpPositionArray()              const noexcept { COPY_ARRAY(m_tcpPos) }
std::array<float, 6> RobotState::getflangePositionArray()           const noexcept { COPY_ARRAY(m_flangePos) }
std::array<float, 6> RobotState::getJointAngleArray()               const noexcept { COPY_ARRAY(m_jointAng) }
std::array<float, 6> RobotState::gettcpVelocityArray()              const noexcept { COPY_ARRAY(m_tcpVel) }
std::array<float, 6> RobotState::getflangeVelocityArray()           const noexcept { COPY_ARRAY(m_flangeVel) }
std::array<float, 6> RobotState::getJointVelocityArray()            const noexcept { COPY_ARRAY(m_jointVel) }
std::array<float, 6> RobotState::getInitialtcpPositionArray()       const noexcept { COPY_ARRAY(m_initialtcpPos) }
std::array<float, 6> RobotState::getInitialflangePositionArray()    const noexcept { COPY_ARRAY(m_initialflangePos) }
std::array<float, 6> RobotState::getInitialJointAngleArray()       const noexcept { COPY_ARRAY(m_initialjointAng) }
std::array<float, 6> RobotState::getRelaxationtcpPositionArray()    const noexcept { COPY_ARRAY(m_relaxationtcpPos) }
std::array<float, 6> RobotState::getRelaxationflangePositionArray() const noexcept { COPY_ARRAY(m_relaxationflangePos) }
std::array<float, 6> RobotState::getRelaxationAngleArray()          const noexcept { COPY_ARRAY(m_relaxationjoint) }
#undef COPY_ARRAY

#define COPY_TO_OUT(member)      \
    if (!out || size<6) return; \
    std::lock_guard<std::mutex> lk(m_mutex); \
    std::memcpy(out, member.data(), 6*sizeof(float));

void RobotState::copytcpPositionData(float* out, size_t size)   const noexcept { COPY_TO_OUT(m_tcpPos) }
void RobotState::copyflangePositionData(float* out, size_t size)   const noexcept { COPY_TO_OUT(m_flangePos) }
void RobotState::copyJointAngleData(float* out, size_t size)   const noexcept { COPY_TO_OUT(m_jointAng) }
void RobotState::copytcpVelocityData(float* out, size_t size)  const noexcept { COPY_TO_OUT(m_tcpVel) }
void RobotState::copyflangeVelocityData(float* out, size_t size) const noexcept { COPY_TO_OUT(m_flangeVel) }
void RobotState::copyJointVelocityData(float* out, size_t size)  const noexcept { COPY_TO_OUT(m_jointVel) }
void RobotState::copyInitialtcpPositionData(float* out, size_t size) const noexcept { COPY_TO_OUT(m_initialtcpPos) }
void RobotState::copyInitialflangePositionData(float* out, size_t size) const noexcept { COPY_TO_OUT(m_initialflangePos) }
void RobotState::copyInitialJointAngleData(float* out, size_t size) const noexcept { COPY_TO_OUT(m_initialjointAng) }
void RobotState::copyRelaxationtcpPositionData(float* out, size_t size) noexcept { COPY_TO_OUT(m_relaxationtcpPos) }
void RobotState::copyRelaxationflangePositionData(float* out, size_t size) noexcept { COPY_TO_OUT(m_relaxationflangePos) }
void RobotState::copyRelaxationAngleData(float* out, size_t size) noexcept { COPY_TO_OUT(m_relaxationjoint) }
#undef COPY_TO_OUT

float RobotState::gettcpPosition(size_t i) const noexcept {
    if (i >= 6) return 0.0f;
    std::lock_guard<std::mutex> lk(m_mutex);
    return m_tcpPos[i];
}
float RobotState::getflangePosition(size_t i) const noexcept {
    if (i >= 6) return 0.0f;
    std::lock_guard<std::mutex> lk(m_mutex);
    return m_flangePos[i];
}
float RobotState::getJointAngle(size_t i) const noexcept {
    if (i >= 6) return 0.0f;
    std::lock_guard<std::mutex> lk(m_mutex);
    return m_jointAng[i];
}
float RobotState::gettcpVelocity(size_t i) const noexcept {
    if (i >= 6) return 0.0f;
    std::lock_guard<std::mutex> lk(m_mutex);
    return m_tcpVel[i];
}
float RobotState::getflangeVelocity(size_t i) const noexcept {
    if (i >= 6) return 0.0f;
    std::lock_guard<std::mutex> lk(m_mutex);
    return m_flangeVel[i];
}
float RobotState::getJointVelocity(size_t i) const noexcept {
    if (i >= 6) return 0.0f;
    std::lock_guard<std::mutex> lk(m_mutex);
    return m_jointVel[i];
}
