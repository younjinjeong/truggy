#pragma once

#include <atomic>
#include <cstddef>
#include <type_traits>

namespace truggy {

// Lock-free single-producer single-consumer ring buffer (Lamport queue).
// N must be a power of 2.
template<typename T, size_t N>
struct ring_buffer_t {
    static_assert((N & (N - 1)) == 0, "N must be a power of 2");
    static_assert(std::is_trivially_copyable_v<T>, "T must be trivially copyable");

    T data[N];
    alignas(64) std::atomic<size_t> head{0};  // written by producer
    alignas(64) std::atomic<size_t> tail{0};  // written by consumer

    bool push(const T& item) {
        size_t h = head.load(std::memory_order_relaxed);
        size_t next = (h + 1) & (N - 1);
        if (next == tail.load(std::memory_order_acquire)) {
            return false;  // full
        }
        data[h] = item;
        head.store(next, std::memory_order_release);
        return true;
    }

    bool pop(T& item) {
        size_t t = tail.load(std::memory_order_relaxed);
        if (t == head.load(std::memory_order_acquire)) {
            return false;  // empty
        }
        item = data[t];
        tail.store((t + 1) & (N - 1), std::memory_order_release);
        return true;
    }

    size_t size() const {
        size_t h = head.load(std::memory_order_acquire);
        size_t t = tail.load(std::memory_order_acquire);
        return (h - t) & (N - 1);
    }

    bool empty() const {
        return head.load(std::memory_order_acquire) ==
               tail.load(std::memory_order_acquire);
    }
};

} // namespace truggy
