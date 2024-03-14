#include "message_statistics.h"

MessageStatistics::MessageStatistics()
  : start_time_(std::chrono::high_resolution_clock::now()) {
}

void MessageStatistics::AddBytes(uint32_t bytes) {
  byte_count_ += bytes;
  message_count_++;
}

uint32_t MessageStatistics::GetByteCount() const {
  return byte_count_;
}

uint32_t MessageStatistics::GetMessageCount() const {
  return message_count_;
}

double MessageStatistics::GetDataRate() const {
  auto current_time = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = current_time - start_time_;
  return elapsed.count() > 0 ? byte_count_ / elapsed.count() : 0;
}
