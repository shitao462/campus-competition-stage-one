#include "cmsis_os.h"
#include "io/buzzer/buzzer.hpp"

namespace
{
constexpr float BUZZER_TIMER_CLOCK_HZ = 84e6f;
constexpr float BEEP_FREQUENCY_HZ = 5000.0f;
constexpr float BEEP_DUTY_CYCLE = 0.1f;
constexpr uint32_t BEEP_DURATION_MS = 100;
constexpr uint32_t BEEP_INTERVAL_MS = 100;
constexpr uint8_t BEEP_COUNT = 3;

sp::Buzzer buzzer(&htim4, TIM_CHANNEL_3, BUZZER_TIMER_CLOCK_HZ);
}  // namespace

extern "C" void buzzer_task(void const * argument)
{
  (void)argument;
  buzzer.set(BEEP_FREQUENCY_HZ, BEEP_DUTY_CYCLE);

  for (uint8_t beep_index = 0; beep_index < BEEP_COUNT; ++beep_index) {
    buzzer.start();
    osDelay(BEEP_DURATION_MS);
    buzzer.stop();

    if (beep_index + 1 < BEEP_COUNT) {
      osDelay(BEEP_INTERVAL_MS);
    }
  }

  // 启动提示音只播放一次；保留任务，避免任务函数返回。
  while (true) {
    osDelay(1000);
  }
}
