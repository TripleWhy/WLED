#pragma once
#ifndef CIRCULAR_BUFFER_ALLOCATOR_H
#define CIRCULAR_BUFFER_ALLOCATOR_H

#include <cstddef>
#include <cstdint>
#include <memory>
#include <utility>
#include <limits>

#include <HardwareSerial.h>

// Base circular buffer memory manager
template<typename Tag>
class CircularBufferMemoryManager {
private:
    struct BlockHeader {
        std::uint16_t size; // Size of the allocation including this header
        bool free;
    };

    static constexpr bool debugPrintFails             = false;
    static constexpr bool debugPrintFailDetails       = false;
    static constexpr bool debugPrintAllocaitons       = false;
    static constexpr bool debugPrintAllocaitonDetails = false;

    static constexpr std::size_t bufferSize = Tag::size;
    static_assert(bufferSize < std::numeric_limits<decltype(BlockHeader::size)>::max(), "BlockHeader is too short for this size.");
    using Buffer = std::array<uint8_t, bufferSize>;
    using BufferIterator = typename Buffer::iterator;
    static inline Buffer buffer{};
    static inline BufferIterator start{buffer.begin()};
    static inline BufferIterator end  {buffer.begin()};

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

    // Advance start as far as possible
    static void advance() {
        while (start != end) {
            BlockHeader& header = reinterpret_cast<BlockHeader&>(*start);
            if (!header.free) {
                break;
            }
            start += header.size;

            if (start >= buffer.end()) {
                start = buffer.begin();
                break;
            }
        }
        // if the buffer is now empty, reset it to the beginning to get a larger continuous area
        if (start == end) {
            start = buffer.begin();
            end = buffer.begin();
        }
    }

public:
    static inline nullptr_t allocateFailed(std::size_t reqSize) {
        if constexpr (debugPrintFails) {
            Serial.printf("CircularBufferMemoryManager::allocate %u failed. free: %u, used: %u \n", reqSize, getFreeSpace(), getUsedSpace());
            if constexpr (debugPrintFailDetails) {
                printAllocationDetails();
            }
        }
        return nullptr;
    }

    // Allocate memory of specified size
    static void* allocate(std::size_t reqSize) {
        if constexpr (debugPrintAllocaitons) {
            Serial.printf("CircularBufferMemoryManager::allocate %u\n", reqSize);
        }
        if (reqSize == 0) {
            return nullptr;
        }

        // Calculate total size needed with header and alignment
        //TODO I believe this aligns the block header, but not the actual returned pointer.
        std::size_t totalSize = alignUp(sizeof(BlockHeader) + reqSize);

        BufferIterator reqStart;
        BufferIterator reqEnd;
        BufferIterator limit;

        if (end >= start) {
            limit = buffer.end();
            reqStart = end;
            reqEnd = (reqStart + totalSize);
            if (reqEnd > limit) {
                limit = start;
                reqStart = buffer.begin();
                reqEnd = (reqStart + totalSize);
                if (reqEnd >= limit) {
                    return allocateFailed(reqSize);
                }
            }
        } else {
            limit = start;
            reqStart = end;
            reqEnd = (reqStart + totalSize);
            if (reqEnd >= limit) {
                return allocateFailed(reqSize);
            }
        }

        end = reqEnd;
        BlockHeader* header = reinterpret_cast<BlockHeader*>(&*reqStart);
        header->free = false;
        if (buffer.end() - end < static_cast<int>(sizeof(BlockHeader))) {
            header->size = buffer.end() - end;
            end = buffer.end();
        } else {
            header->size = totalSize;

            // insert dummy header to make block iteration work
            // end != start at this point, and due to the alignment contraints, there *shuold* be engouh space for a full BlockHeader in the unused region
            BlockHeader& dummy = reinterpret_cast<BlockHeader&>(*end);
            dummy.size = buffer.end() - end;
            dummy.free = true;
        }

        if constexpr (debugPrintAllocaitonDetails) {
            printAllocationDetails();
        }
        return getUserPtr(header);
    }

    // Deallocate previously allocated memory
    static void deallocate(void* ptr) {
        if constexpr (debugPrintAllocaitons) {
            Serial.printf("CircularBufferMemoryManager::deallocate %p\n", ptr);
        }
        if (ptr == nullptr) {
            return;
        }

        BlockHeader* header = getHeader(ptr);
        header->free = true;

        if (reinterpret_cast<uint8_t*>(header) == &*start) {
            advance();
        }
        if constexpr (debugPrintAllocaitonDetails) {
            printAllocationDetails();
        }
    }

    static std::size_t getFreeSpace() {
        if (start > end) {
            return start - end;
        } else {
            return bufferSize - (end - start);
        }
    }

    static std::size_t getContinuousFreeSpace() {
        if (start > end) {
            return start - end;
        } else {
            return std::max(buffer.end() - end, start - buffer.begin());
        }
    }

    static std::size_t getUsedSpace() {
        return (end - start + bufferSize) % bufferSize;
    }

    static void printAllocationDetails() {
        Serial.printf("buffer: %p - %p (%u)\n", &*buffer.cbegin(), &*buffer.cend(), buffer.cend() - buffer.cbegin());
        Serial.printf("start: %p (%u)\n", &*start, start - buffer.begin());
        Serial.printf("end:   %p (%u)\n", &*end, end - buffer.begin());

        Serial.println();
        if (end > start) {
            Serial.printf("unused: %u\n", start - buffer.begin());
        }
        for (auto it = start; it != end; ) {
            BlockHeader& header = reinterpret_cast<BlockHeader&>(*it);
            Serial.printf("header: %p (%u), pointer: %p, size: %u, free: %u\n", &header, it - buffer.begin(), getUserPtr(&header), header.size, header.free);
            if (it + header.size >= buffer.end()) {
                // Serial.printf("unused: %u\n", buffer.end() - it - header.size);
                it = buffer.begin();
            } else {
                it += header.size;
            }
        }
        if (end > start) {
            Serial.printf("unused: %u\n\n", buffer.end() - end);
        } else {
            Serial.printf("unused: %u\n\n", start - end);
        }
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
        // Serial.printf("%s: %u\n", __PRETTY_FUNCTION__, n);
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
        if (getContinuousFreeSpace() < 1) {
            return nullptr;
        }
        T* rawPtr = allocate(1);
        construct(rawPtr, std::forward<Args>(args)...);
        return CircularAllocator::unique_ptr{rawPtr};
    }

    static inline constexpr std::size_t getContinuousFreeSpace() {
        return CircularBufferMemoryManager<Tag>::getContinuousFreeSpace() / sizeof(T);
    }

    class vector : public std::vector<T, CircularAllocator> {
    private:
        using Base = std::vector<T, CircularAllocator>;

    public:
        bool resize(size_type count, bool preserveContent = false) {
            if (Base::size() == count) {
                return true;
            }
            if (!preserveContent) {
                Base::clear();
                Base::shrink_to_fit();
            }
            if (getContinuousFreeSpace() < count) {
                return false;
            }
            Base::reserve(count);
            Base::resize(count);
            return true;
        }
    };
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
