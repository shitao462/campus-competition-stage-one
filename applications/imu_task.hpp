#ifndef IMU_TASK_HPP
#define IMU_TASK_HPP

#include <cstdint>

namespace app
{
struct ImuStatus
{
  bool ready;
  float yaw;
  float yaw_rate;
  uint32_t stamp_ms;
};

ImuStatus get_imu_status();
}  // namespace app

#endif  // IMU_TASK_HPP
