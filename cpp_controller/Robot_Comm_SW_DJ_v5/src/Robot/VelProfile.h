#include <algorithm>

struct ProfileConfig
{
    double ramp_duration_sec = 2.0;       // 가속/감속에 걸리는 시간
    float max_velocity = 5.0f;            // 최대 속도
    float final_velocity = 0.0f;          // 목표 위치 도달 시 최종 속도
    float target_z = 500.0f;              // 최종 목표 Z 위치
    float move_distance = 100.0f;         // 총 이동 거리 (감속 시작 지점 계산용)
    int direction = 1;                    // 이동 방향 (1: 상승, -1: 하강)
};

class VelocityProfiler
{
public:
    VelocityProfiler() = default;

    // ProfileConfig를 인자로 받아 속도를 계산
    float calculate(const ProfileConfig& config, double elapsed_sec, float current_pos_z, float start_pos_z) const
    {
        // 1. 시간 기반 가속/등속 속도 계산
        float time_based_vz = 0.0f;
        if (elapsed_sec < config.ramp_duration_sec) {
            float t_norm = static_cast<float>(elapsed_sec / config.ramp_duration_sec);
            float smooth_norm = t_norm * t_norm * (3.0f - 2.0f * t_norm);
            time_based_vz = config.max_velocity * smooth_norm;
        }
        else {
            time_based_vz = config.max_velocity;
        }

        // 2. 위치 기반 감속 속도 계산 (방향에 따라 일반화)
        // 감속 시작 거리 = 총 이동거리 - 감속 시간동안 이동할 거리 (근사치)
        float deceleration_distance = (float)config.max_velocity * (float)config.ramp_duration_sec * 0.5f;
        float deceleration_start_dist = config.move_distance - deceleration_distance;
        float moved_dist = std::abs(current_pos_z - start_pos_z);

        float position_based_vz = config.max_velocity;
        if (moved_dist >= deceleration_start_dist)
        {
            float t_norm = (moved_dist - deceleration_start_dist) / deceleration_distance;
            t_norm = std::clamp(t_norm, 0.0f, 1.0f);
            float smooth_norm = t_norm * t_norm * (3.0f - 2.0f * t_norm);
            position_based_vz = config.max_velocity - smooth_norm * (config.max_velocity - config.final_velocity);
        }

        // 3. 두 속도 중 더 느린 속도와 방향을 적용하여 최종 반환
        return (std::min)(time_based_vz, position_based_vz) * config.direction;
    }
};