#pragma once

#include <mutex>

namespace deepbreak {

template <typename T>
class Buffer {
 public:
  Buffer() = default;
  ~Buffer() = default;

  void push(T data) {
    const std::lock_guard<std::mutex> guard(lock_);
    data_ = data;
  }

  T get() const {
    const std::lock_guard<std::mutex> guard(lock_);
    return data_;
  }

  T clear() {
    const std::lock_guard<std::mutex> guard(lock_);
    data_ = T();
  }

 private:
  T data_;
  mutable std::mutex lock_;
};

}  // namespace deepbreak