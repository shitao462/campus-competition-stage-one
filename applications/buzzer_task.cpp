#include "cmsis_os.h"
#include "io/buzzer/buzzer.hpp"

namespace
{
constexpr float BUZZER_TIMER_CLOCK_HZ = 84e6f;
constexpr float BEEP_DUTY_CYCLE = 0.1f;
constexpr uint32_t NOTE_GAP_MS = 25;
constexpr float NOTE_C_HZ = 2093.0f;
constexpr float NOTE_D_HZ = 2349.32f;
constexpr float NOTE_E_HZ = 2637.02f;
constexpr float NOTE_F_HZ = 2793.83f;
constexpr float NOTE_G_HZ = 3135.96f;

struct Note
{
  float frequency_hz;
  uint32_t duration_ms;
};

// 《两只老虎》开头：1 2 3 1 | 1 2 3 1 | 3 4 5 - | 3 4 5 -。
constexpr Note STARTUP_MELODY[] = {
  {NOTE_C_HZ, 250}, {NOTE_D_HZ, 250}, {NOTE_E_HZ, 250}, {NOTE_C_HZ, 250},
  {NOTE_C_HZ, 250}, {NOTE_D_HZ, 250}, {NOTE_E_HZ, 250}, {NOTE_C_HZ, 250},
  {NOTE_E_HZ, 250}, {NOTE_F_HZ, 250}, {NOTE_G_HZ, 500},
  {NOTE_E_HZ, 250}, {NOTE_F_HZ, 250}, {NOTE_G_HZ, 500}};

sp::Buzzer buzzer(&htim4, TIM_CHANNEL_3, BUZZER_TIMER_CLOCK_HZ);
}  // namespace

extern "C" void buzzer_task(void const * argument)
{
  (void)argument;
  for (const Note & note : STARTUP_MELODY) {
    buzzer.set(note.frequency_hz, BEEP_DUTY_CYCLE);
    buzzer.start();
    osDelay(note.duration_ms - NOTE_GAP_MS);
    buzzer.stop();
    osDelay(NOTE_GAP_MS);
  }

  // 启动提示音只播放一次；保留任务，避免任务函数返回。
  while (true) {
    osDelay(1000);
  }
}
