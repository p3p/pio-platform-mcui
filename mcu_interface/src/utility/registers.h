#pragma once

#include <cstdint>
using std::size_t;

#include "bit_manipulation.h"

namespace MCUI::util::registers {

// standard register, reads are masked and shifted, writes use read/modify/write
template <typename T, size_t Position, size_t Count> struct reg32_t {
  [[gnu::always_inline]] constexpr operator T() const volatile { return MCUI::util::bitset_get_value(m_value, Position, Count); };
  template <typename O> [[gnu::always_inline]] constexpr void operator = (const O& other) volatile { MCUI::util::bitset_set_value(m_value, other, Position, Count);}
  template <typename O> [[gnu::always_inline]] constexpr T operator + (const O& other) volatile { return MCUI::util::bitset_get_value(m_value, Position, Count) + other; }
  [[gnu::always_inline, nodiscard]] uint32_t masked_underlying_value() volatile { return MCUI::util::bitset_mask(m_value, mask()); }

  [[gnu::always_inline, nodiscard]] static constexpr uint32_t mask() { return MCUI::util::bitset_build_mask(Position, Count); }
  [[gnu::always_inline, nodiscard]] static constexpr size_t position() { return Position; }
  [[gnu::always_inline, nodiscard]] static constexpr size_t bits() { return Count; }
  [[gnu::always_inline, nodiscard]] static constexpr uint32_t shift_and_mask(T value) { uint32_t result = value; result <<= position(); return result & mask(); }

  [[gnu::always_inline, nodiscard]] volatile uint32_t& underlying_value_ref() volatile { return m_value; }
  [[gnu::always_inline, nodiscard]] uint32_t underlying_value() volatile { return m_value; }
  [[gnu::always_inline]] void underlying_value(uint32_t value) volatile { m_value = value; }

protected:
  uint32_t m_value;
};

// FIFO Register, read and written values are separate registers, do not read/modify/write
template <typename T, size_t Position, size_t Count> struct reg32_fifo_t : public reg32_t<T, Position, Count> {
  using reg32_t<T, Position, Count>::m_value;
  template <typename O> [[gnu::always_inline]] constexpr void operator = (const O& other) volatile { MCUI::util::bitset_set_write_only_value(m_value, other, Position, Count); }
};

// Read Only Register, writing is disabled
template <typename T, size_t Position, size_t Count> struct reg32_ro_t : public reg32_t<T, Position, Count> {
  template <typename O> [[gnu::always_inline]] constexpr void operator = (const O& other) volatile { static_assert(sizeof(O) == -1, "reg32_ro_t is Read Only!"); }
  void underlying_value(uint32_t value) { static_assert(sizeof(T) == -1, "reg32_ro_t is Read Only!"); }
};

// Write Only Register, reading is disabled
template <typename T, size_t Position, size_t Count> struct reg32_wo_t : public reg32_t<T, Position, Count> {
  using reg32_t<T, Position, Count>::m_value;
  [[gnu::always_inline]] constexpr operator T() const volatile { static_assert(sizeof(T) == -1, "reg32_wo_t is Write Only!"); return 0; };
  template <typename O> [[gnu::always_inline]] constexpr void operator = (const O& other) volatile { MCUI::util::bitset_set_write_only_value(m_value, other, Position, Count); }
  template <typename O> [[gnu::always_inline]] constexpr T operator + (const O& other) volatile { static_assert(sizeof(O) == -1, "reg32_wo_t is Write Only!"); return 0; }
  [[gnu::always_inline, nodiscard]] constexpr uint32_t masked_underlying_value() { static_assert(sizeof(T) == -1, "reg32_wo_t is Write Only!"); return 0; }
  uint32_t underlying_value() const { static_assert(sizeof(T) == -1, "reg32_wo_t is Write Only!"); return 0;}
};

} // namespace util
