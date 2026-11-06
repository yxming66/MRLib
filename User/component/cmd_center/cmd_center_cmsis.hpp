#pragma once

// CMSIS-RTOS 消息队列适配器。
// 使用方式：先创建 osMessageQueue，再构造 CmsisQueueSink(queue)，
// 将 sink 注册到命令中心；默认队列满时返回失败，不丢弃已有命令。
// reset_before_put=true 仅用于独占生产者的任务上下文：会丢弃整个队列。
// 队列由应用创建/销毁并须比 sink 活得更久；ISR 不允许 reset 模式。

#include <type_traits>

#include "cmsis_os2.h"

namespace cmd {

class CmsisQueueSink {
 public:
  explicit CmsisQueueSink(osMessageQueueId_t queue, bool reset_before_put = false)
      : queue_(queue), reset_before_put_(reset_before_put) {}

  template <typename Cmd>
  bool operator()(const Cmd& cmd) const {
    static_assert(std::is_trivially_copyable_v<Cmd>,
                  "CMSIS queue commands must be trivially copyable");
    if (queue_ == nullptr || osMessageQueueGetMsgSize(queue_) != sizeof(Cmd)) {
      return false;
    }
    if (reset_before_put_) {
      if (osMessageQueueReset(queue_) != osOK) return false;
    }
    return osMessageQueuePut(queue_, &cmd, 0, 0) == osOK;
  }

 private:
  osMessageQueueId_t queue_ = nullptr;
  bool reset_before_put_ = false;
};

inline CmsisQueueSink queue(osMessageQueueId_t queue, bool reset_before_put = false) {
  return CmsisQueueSink(queue, reset_before_put);
}

}  // namespace cmd
