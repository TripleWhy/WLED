#pragma once
#ifndef CIRCULAR_BUFFER_ALLOCATOR_H
#define CIRCULAR_BUFFER_ALLOCATOR_H

#include <cstddef>
#include <cstdint>
#include <memory>
#include <utility>
#include <limits>

// Memory block header - kept minimal
struct BlockHeader {
    std::size_t size;  // Size of the allocation including this header
    bool free;         // Whether this block is free
};

// Base circular buffer memory manager
template<typename Tag>
class CircularBufferMemoryManager {
private:
    static constexpr std::size_t bufferSize = Tag::size;
    static inline std::array<uint8_t, bufferSize> buffer{};
    static inline auto head = buffer.begin();
    static inline auto tail = buffer.begin();

    static constexpr std::size_t alignment = alignof(std::max_align_t);

    // Align size up to the required alignment
    static std::size_t alignUp(std::size_t size) {
        return (size + alignment - 1) & ~(alignment - 1);
    }

    // Get header from pointer to user data
    static BlockHeader* getHeader(void* ptr) {
        return static_cast<BlockHeader*>(ptr) - 1;
    }

    // Get pointer to user data from header
    static void* getUserPtr(BlockHeader* header) {
        return header + 1;
    }

    // Advance head as far as possible
    static void advanceHead() {
        while (head < tail) {
            BlockHeader* header = reinterpret_cast<BlockHeader*>(head);
            if (!header->free) {
                break;  // Stop at first non-free block
            }
            // Advance head past this free block
            head += header->size;

            // Check if we've wrapped around or reached the end
            if (head >= buffer.end()) {
                head = buffer.begin();
                tail = buffer.begin();  // Reset tail as well when we wrap
                break;
            }
        }
    }

public:
    // Allocate memory of specified size
    static void* allocate(std::size_t reqSize) {
        if (reqSize == 0) {
            return nullptr;
        }

        // Calculate total size needed with header and alignment
        std::size_t totalSize = alignUp(sizeof(BlockHeader) + reqSize);

        // First try: allocate at current tail position
        if (tail + totalSize <= buffer.end()) {
            // We have space at the end
            BlockHeader* header = reinterpret_cast<BlockHeader*>(tail);
            header->size = totalSize;
            header->free = false;

            tail += totalSize;  // Move tail forward
            return getUserPtr(header);
        }

        // Second try: If we don't have space at the end, try from beginning
        // But only if head is not at the beginning
        if (head != buffer.begin()) {
            // Reset tail to beginning
            uint8_t* oldTail = tail;
            tail = buffer.begin();

            // Check if we have space from beginning to head
            if (tail + totalSize <= head) {
                BlockHeader* header = reinterpret_cast<BlockHeader*>(tail);
                header->size = totalSize;
                header->free = false;

                tail += totalSize;  // Move tail forward
                return getUserPtr(header);
            }

            // If allocation at beginning failed, restore tail
            tail = oldTail;
        }

        // Out of memory - can't allocate
        return nullptr;
    }

    // Deallocate previously allocated memory
    static void deallocate(void* ptr) {
        if (!ptr) return;

        // Get the block header from the pointer
        BlockHeader* header = getHeader(ptr);

        // Mark the block as free
        header->free = true;

        // If this block is at the head, advance the head as far as possible
        if (reinterpret_cast<uint8_t*>(header) == head) {
            advanceHead();
        }
    }

    // Get the free space (approximate - doesn't account for fragmentation)
    static std::size_t getFreeSpace() {
        if (head >= tail) {
            return bufferSize - (head - tail);
        }
        return tail - head;
    }

    static std::size_t getUsedSpace() {
        return (head - tail + bufferSize) % bufferSize;
    }
};

// STL-compatible allocator that uses CircularBufferMemoryManager
template<typename T, typename Tag>
class CircularAllocator {
public:
    struct Deleter {
        void operator()(T* ptr) const {
            CircularAllocator::deleter(ptr);
        }
    };

    using value_type = T;
    using pointer = T*;
    using const_pointer = const T*;
    using reference = T&;
    using const_reference = const T&;
    using size_type = std::size_t;
    using difference_type = std::ptrdiff_t;
    using unique_ptr = std::unique_ptr<T, Deleter>;
    using vector = std::vector<T, CircularAllocator>;

    // Required by C++11 for allocator compatibility
    template<typename U>
    struct rebind {
        using other = CircularAllocator<U, Tag>;
    };

    // Constructors
    CircularAllocator() noexcept = default;

    template<typename U>
    CircularAllocator(const CircularAllocator<U, Tag>&) noexcept {}

    // Allocate memory for n objects of type T
    static T* allocate(std::size_t n) {
        if (n > std::numeric_limits<std::size_t>::max() / sizeof(T)) {
            return nullptr;  // No exceptions, return null on overflow
        }

        void* ptr = CircularBufferMemoryManager<Tag>::allocate(n * sizeof(T));
        return static_cast<T*>(ptr);
    }

    // Deallocate memory previously allocated
    static void deallocate(T* p, std::size_t) noexcept {
        CircularBufferMemoryManager<Tag>::deallocate(p);
    }

    // Construct an object at the given address
    template<typename U, typename... Args>
    static void construct(U* p, Args&&... args) {
        ::new ((void*)p) U(std::forward<Args>(args)...);
    }

    // Destroy an object without deallocating memory
    template<typename U>
    static void destroy(U* p) {
        p->~U();
    }

    // Required for allocator compatibility
    static constexpr size_type max_size() noexcept {
        return std::numeric_limits<size_type>::max() / sizeof(T);
    }

    static void deleter(T* ptr) {
        destroy(ptr);
        deallocate(ptr, 1);
    }

    template<typename... Args>
    static CircularAllocator::unique_ptr make_unique(Args&&... args) {
        T* rawPtr = allocate(1);
        construct(rawPtr, std::forward<Args>(args)...);
        return CircularAllocator::unique_ptr{rawPtr};
    }
};

// Equality operators for allocator comparison - required by STL
template<typename T, typename U, typename Tag1, typename Tag2>
constexpr bool operator==(const CircularAllocator<T, Tag1>&, const CircularAllocator<U, Tag2>&) noexcept {
    return std::is_same_v<Tag1, Tag2>;  // Equal if same tag type
}

template<typename T, typename U, typename Tag1, typename Tag2>
constexpr bool operator!=(const CircularAllocator<T, Tag1>&, const CircularAllocator<U, Tag2>&) noexcept {
    return !std::is_same_v<Tag1, Tag2>;  // Equal if same tag type
}


/* each segment uses 82 bytes of SRAM memory, so if you're application fails because of
  insufficient memory, decreasing MAX_NUM_SEGMENTS may help */
#ifdef ESP8266
  constexpr std::size_t MAX_NUM_SEGMENTS = 16;
  /* How much data bytes all segments combined may allocate */
  constexpr std::size_t MAX_SEGMENT_DATA = 5120;
#elif defined(CONFIG_IDF_TARGET_ESP32S2)
  constexpr std::size_t MAX_NUM_SEGMENTS = 20;
  constexpr std::size_t MAX_SEGMENT_DATA = (MAX_NUM_SEGMENTS*512);  // 10k by default (S2 is short on free RAM)
#else
  constexpr std::size_t MAX_NUM_SEGMENTS = 32;  // warning: going beyond 32 may consume too much RAM for stable operation
  constexpr std::size_t MAX_SEGMENT_DATA = (MAX_NUM_SEGMENTS*1280); // 40k by default
#endif

struct SegmentMemoryTag { static constexpr size_t size = MAX_SEGMENT_DATA; };
template<typename T>
using SegmentAllocator = CircularAllocator<T, SegmentMemoryTag>;
using SegmentMemoryManager = CircularBufferMemoryManager<SegmentMemoryTag>;

#endif // CIRCULAR_BUFFER_ALLOCATOR_H
