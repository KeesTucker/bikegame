#ifndef YDS_ALLOCATOR_H
#define YDS_ALLOCATOR_H

#include <malloc.h>

class ysAllocator {
public:
    template <int Alignment>
    static void *BlockAllocate(int size) {
        return ::_aligned_malloc(size, Alignment);
    }

    template <int Alignment, typename T_Create>
    static T_Create *TypeAllocate(int n = 1, bool construct = true) {
        void *block = BlockAllocate<Alignment>(sizeof(T_Create) * n);
        T_Create *typedArray = reinterpret_cast<T_Create *>(block);

        if (construct) {
            for (int i = 0; i < n; i++) {
                new (typedArray + i) T_Create();
            }
        }

        return typedArray;
    }

    template <typename T_Free>
    static void TypeFree(T_Free *data, int n, bool destroy = true, int alignment = 16) {
        if (destroy) {
            for (int i = 0; i < n; i++) {
                data[i].~T_Free();
            }
        }

        BlockFree(reinterpret_cast<void *>(data), alignment);
    }

    static void BlockFree(void *block, int alignment) {
        _aligned_free(block);
    }
};

// Explicit specialization moved outside class without 'static'
template <>
inline void *ysAllocator::BlockAllocate<1>(int size) {
    return malloc(size); // Alignment of 1 is normal malloc
}

#endif /* YDS_ALLOCATOR_H */