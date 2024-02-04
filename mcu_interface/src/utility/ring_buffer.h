#pragma once

#include <cstdint>
using std::size_t;
#include <cstring>
#include <utility/const_functions.h>

namespace MCUI::util {

template <typename T, size_t S> class RingBuffer {
public:
  static_assert(S && ((S & (S - 1)) == 0) && S > 2); // implementation requires power of 2 size, and can only store size - 1 elements
  RingBuffer() {index_read = index_write = 0;}

  size_t available() const {return mask(index_write - index_read);}
  size_t free() const {return size() - available();}
  bool empty() const {return index_read == index_write;}
  bool full() const {return next(index_write) == index_read;}
  void clear() {index_read = index_write = 0;}

  bool peek(T *const value) const {
    if (value == nullptr || empty()) return false;
    *value = buffer[index_read];
    return true;
  }

  inline size_t read(T *const dst, size_t length) {
    if (dst == nullptr || length == 0) return 0;
    length = MCUI::util::min(length, available());
    const size_t length1 = MCUI::util::min(length, buffer_size - index_read);
    memcpy(dst, (char*)buffer + index_read, length1);
    memcpy(dst + length1, (char*)buffer, length - length1);
    index_read = mask(index_read + length);
    return length;
  }

  inline size_t write(T const* src, size_t length) {
    if (src == nullptr || length == 0) return 0;
    length = MCUI::util::min(length, free());
    const size_t length1 = MCUI::util::min(length, buffer_size - index_write);
    memcpy((char*)buffer + index_write, src, length1);
    memcpy((char*)buffer, src + length1, length - length1);
    index_write = mask(index_write + length);
    return length;
  }

  size_t read(T *const value) {
    if (value == nullptr || empty()) return 0;
    *value = buffer[index_read];
    index_read = next(index_read);
    return 1;
  }

  size_t write(const T value) {
    size_t next_head = next(index_write);
    if (next_head == index_read) return 0;     // buffer full
    buffer[index_write] = value;
    index_write = next_head;
    return 1;
  }

  static constexpr size_t size() {
    return buffer_size - 1;
  }

private:
  inline size_t mask(size_t val) const {
    return val & buffer_mask;
  }

  inline size_t next(size_t val) const {
    return mask(val + 1);
  }

  static const size_t buffer_size = S;
  static const size_t buffer_mask = buffer_size - 1;
  volatile T buffer[buffer_size];
  volatile size_t index_write;
  volatile size_t index_read;
};

template <typename T, size_t S> class InOutRingBuffer {
public:
  static constexpr size_t size() {
    return S - 1;
  }
  RingBuffer<T, S>& in() { return m_in; }
  RingBuffer<T, S>& out() {return m_out; }
private:
  RingBuffer<T, S> m_in;
  RingBuffer<T, S> m_out;
};

} // namespace util
