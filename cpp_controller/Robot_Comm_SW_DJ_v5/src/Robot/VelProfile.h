#include <algorithm>

struct ProfileConfig
{
    double ramp_duration_sec = 2.0;       // ����/���ӿ� �ɸ��� �ð�
    float max_velocity = 5.0f;            // �ִ� �ӵ�
    float final_velocity = 0.0f;          // ��ǥ ��ġ ���� �� ���� �ӵ�
    float target_z = 500.0f;              // ���� ��ǥ Z ��ġ
    float move_distance = 100.0f;         // �� �̵� �Ÿ� (���� ���� ���� ����)
    int direction = 1;                    // �̵� ���� (1: ���, -1: �ϰ�)
};

class VelocityProfiler
{
public:
    VelocityProfiler() = default;

    // ProfileConfig�� ���ڷ� �޾� �ӵ��� ���
    float calculate(const ProfileConfig& config, double elapsed_sec, float current_pos_z, float start_pos_z) const
    {
        // 1. �ð� ��� ����/��� �ӵ� ���
        float time_based_vz = 0.0f;
        if (elapsed_sec < config.ramp_duration_sec) {
            float t_norm = static_cast<float>(elapsed_sec / config.ramp_duration_sec);
            float smooth_norm = t_norm * t_norm * (3.0f - 2.0f * t_norm);
            time_based_vz = config.max_velocity * smooth_norm;
        }
        else {
            time_based_vz = config.max_velocity;
        }

        // 2. ��ġ ��� ���� �ӵ� ��� (���⿡ ���� �Ϲ�ȭ)
        // ���� ���� �Ÿ� = �� �̵��Ÿ� - ���� �ð����� �̵��� �Ÿ� (�ٻ�ġ)
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

        // 3. �� �ӵ� �� �� ���� �ӵ��� ������ �����Ͽ� ���� ��ȯ
        return (std::min)(time_based_vz, position_based_vz) * config.direction;
    }
};