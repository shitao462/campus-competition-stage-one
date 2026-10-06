#ifndef REMOTE_TASK_HPP
#define REMOTE_TASK_HPP

#include <cstdint>

namespace app
{
struct RemoteStatus
{
  bool alive;
  bool right_switch_down;
};

RemoteStatus get_remote_status();
}  // namespace app

#endif  // REMOTE_TASK_HPP
