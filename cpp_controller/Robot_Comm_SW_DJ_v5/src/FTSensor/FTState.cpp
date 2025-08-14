#include "FTState.h"
#include <algorithm>

//FTState::FTState() noexcept {
//    
//}

// 데이터 전체 초기화
void FTState::clearAll() noexcept {
    sensorForce_ = { 0.0f, 0.0f, 0.0f };
    sensorTorque_ = { 0.0f, 0.0f, 0.0f };
    rawForce_ = { 0.0f, 0.0f, 0.0f };
    rawTorque_ = { 0.0f, 0.0f, 0.0f };
    biasForce_ = { 0.0f, 0.0f, 0.0f };
    biasTorque_ = { 0.0f, 0.0f, 0.0f };
    biasSumForce_ = { 0.0f, 0.0f, 0.0f };
    biasSumTorque_ = { 0.0f, 0.0f, 0.0f };
    biasedForce_ = { 0.0f, 0.0f, 0.0f };
    biasedTorque_ = { 0.0f, 0.0f, 0.0f };
    forceBase_ = { 0.0f, 0.0f, 0.0f };
    torqueBase_ = { 0.0f, 0.0f, 0.0f };
    filteredForce_ = { 0.0f, 0.0f, 0.0f };
    filteredTorque_ = { 0.0f, 0.0f, 0.0f };
}

// Getter 구현
std::array<float, 3> FTState::getSensorForce() const noexcept { 
    std::lock_guard<std::mutex> lock(mtx_); 
    return sensorForce_; 
}
std::array<float, 3> FTState::getSensorTorque() const noexcept { 
    std::lock_guard<std::mutex> lock(mtx_); 
    return sensorTorque_; 
}
std::array<float, 3> FTState::getRawForce() const noexcept { 
    std::lock_guard<std::mutex> lock(mtx_); 
    return rawForce_; 
}
std::array<float, 3> FTState::getRawTorque() const noexcept { 
    std::lock_guard<std::mutex> lock(mtx_); 
    return rawTorque_; 
}
std::array<float, 3> FTState::getBiasForce() const noexcept {
    std::lock_guard<std::mutex> lock(mtx_); 
    return biasForce_; 
}
std::array<float, 3> FTState::getBiasTorque() const noexcept { 
    std::lock_guard<std::mutex> lock(mtx_); 
    return biasTorque_; 
}
std::array<float, 3> FTState::getBiasSumForce() const noexcept { 
    std::lock_guard<std::mutex> lock(mtx_); 
    return biasSumForce_; 
}
std::array<float, 3> FTState::getBiasSumTorque() const noexcept { 
    std::lock_guard<std::mutex> lock(mtx_); 
    return biasSumTorque_; 
}
std::array<float, 3> FTState::getBiasedForce() const noexcept {
    std::lock_guard<std::mutex> lock(mtx_);
    std::array<float, 3> result;
    for (int i = 0; i < 3; ++i) {
        result[i] = rawForce_[i] - biasForce_[i];
    }
    return result;
}
std::array<float, 3> FTState::getBiasedTorque() const noexcept {
    std::lock_guard<std::mutex> lock(mtx_);
    std::array<float, 3> result;
    for (int i = 0; i < 3; ++i) {
        result[i] = rawTorque_[i] - biasTorque_[i];
    }
    return result;
}
std::array<float, 3> FTState::getForceBase() const noexcept {
    std::lock_guard<std::mutex> lock(mtx_); 
    return forceBase_; 
}
std::array<float, 3> FTState::getTorqueBase() const noexcept { 
    std::lock_guard<std::mutex> lock(mtx_); 
    return torqueBase_; 
}
std::array<float, 3> FTState::getFilteredForce() const noexcept { 
    std::lock_guard<std::mutex> lock(mtx_); 
    return filteredForce_; 
}
std::array<float, 3> FTState::getFilteredTorque() const noexcept { 
    std::lock_guard<std::mutex> lock(mtx_); 
    return filteredTorque_; 
}


// Setter 구현
void FTState::setAll_FT_cal(
    const std::array<float, 3>& biasedForce,
    const std::array<float, 3>& biasedTorque,
    const std::array<float, 3>& forceBase,
    const std::array<float, 3>& torqueBase,
    const std::array<float, 3>& filteredForce,
    const std::array<float, 3>& filteredTorque) noexcept {
    std::lock_guard<std::mutex> lock(mtx_);
    biasedForce_ = biasedForce;
    biasedTorque_ = biasedTorque;
    forceBase_ = forceBase;
    torqueBase_ = torqueBase;
    filteredForce_ = filteredForce;
    filteredTorque_ = filteredTorque;
}

void FTState::setSensorForce(const std::array<float, 3>& val) noexcept { 
    std::lock_guard<std::mutex> lock(mtx_); 
    sensorForce_ = val; 
}
void FTState::setSensorTorque(const std::array<float, 3>& val) noexcept { 
    std::lock_guard<std::mutex> lock(mtx_); 
    sensorTorque_ = val; 
}
void FTState::setRawForce(const std::array<float, 3>& val) noexcept { 
    std::lock_guard<std::mutex> lock(mtx_); 
    rawForce_ = val; 
}
void FTState::setRawTorque(const std::array<float, 3>& val) noexcept { 
    std::lock_guard<std::mutex> lock(mtx_); 
    rawTorque_ = val; 
}
void FTState::setBiasForce(const std::array<float, 3>& val) noexcept { 
    std::lock_guard<std::mutex> lock(mtx_); 
    biasForce_ = val; 
}
void FTState::setBiasTorque(const std::array<float, 3>& val) noexcept {
    std::lock_guard<std::mutex> lock(mtx_); 
    biasTorque_ = val; 
}
void FTState::setBiasSumForce(const std::array<float, 3>& val) noexcept { 
    std::lock_guard<std::mutex> lock(mtx_);
    biasSumForce_ = val; 
}
void FTState::setBiasSumTorque(const std::array<float, 3>& val) noexcept { 
    std::lock_guard<std::mutex> lock(mtx_);
    biasSumTorque_ = val; 
}
void FTState::setBiasedForce(const std::array<float, 3>& val) noexcept { 
    std::lock_guard<std::mutex> lock(mtx_); 
    biasedForce_ = val; 
}
void FTState::setBiasedTorque(const std::array<float, 3>& val) noexcept { 
    std::lock_guard<std::mutex> lock(mtx_); 
    biasedTorque_ = val; 
}
void FTState::setForceBase(const std::array<float, 3>& val) noexcept { 
    std::lock_guard<std::mutex> lock(mtx_); 
    forceBase_ = val; 
}
void FTState::setTorqueBase(const std::array<float, 3>& val) noexcept { 
    std::lock_guard<std::mutex> lock(mtx_); 
    torqueBase_ = val; 
}
void FTState::setFilteredForce(const std::array<float, 3>& val) noexcept { 
    std::lock_guard<std::mutex> lock(mtx_); 
    filteredForce_ = val; 
}
void FTState::setFilteredTorque(const std::array<float, 3>& val) noexcept { 
    std::lock_guard<std::mutex> lock(mtx_); 
    filteredTorque_ = val; 
}


// Bias 누적 함수
void FTState::addBiasSumForce(const std::array<float, 3>& val) noexcept {
    std::lock_guard<std::mutex> lock(mtx_);
    for (size_t i = 0; i < 3; ++i) biasSumForce_[i] += val[i];
}
void FTState::addBiasSumTorque(const std::array<float, 3>& val) noexcept {
    std::lock_guard<std::mutex> lock(mtx_);
    for (size_t i = 0; i < 3; ++i) biasSumTorque_[i] += val[i];
}
void FTState::resetBiasSum() noexcept {
    std::lock_guard<std::mutex> lock(mtx_);
    biasSumForce_ = { 0.0f, 0.0f, 0.0f };
    biasSumTorque_ = { 0.0f, 0.0f, 0.0f };
}

void FTState::updateBiasAverage() noexcept {    // Bias 평균값 계산
    std::lock_guard<std::mutex> lock(mtx_);
    if (biasCount_ == 0) {
        // 0으로 나누기 방지: 평균값을 0으로 설정
        biasForce_.fill(0.0f);
        biasTorque_.fill(0.0f);
        return;
    }
    for (int j = 0; j < 3; ++j) {
        biasForce_[j] = biasSumForce_[j] / static_cast<float>(biasCount_);
        biasTorque_[j] = biasSumTorque_[j] / static_cast<float>(biasCount_);
    }
}

FTState::Snapshot FTState::updateSnapshot() noexcept {
    std::lock_guard<std::mutex> lock(mtx_);
    m_snapshot_ = {
        biasedForce_,   biasedTorque_,
        forceBase_,     torqueBase_,
        filteredForce_, filteredTorque_
    };
    return m_snapshot_;
}

FTState::Snapshot FTState::getSnapshot() const noexcept {
    return m_snapshot_;
}
