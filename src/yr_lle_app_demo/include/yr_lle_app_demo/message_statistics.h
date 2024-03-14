#ifndef MESSAGE_STATISTICS_H
#define MESSAGE_STATISTICS_H

#include <cstdint>
#include <chrono>

class MessageStatistics {
public:
  MessageStatistics();

  void AddBytes(uint32_t bytes);
  uint32_t GetByteCount() const;
  uint32_t GetMessageCount() const;
  double GetDataRate() const;

  void Reset() {
    byte_count_ = 0;
    message_count_ = 0;
    start_time_ = std::chrono::high_resolution_clock::now();
  }

private:
  uint32_t byte_count_ = 0;
  uint32_t message_count_ = 0;
  std::chrono::high_resolution_clock::time_point start_time_;
};

#endif  // MESSAGE_STATISTICS_H
