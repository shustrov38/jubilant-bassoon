#pragma once

#include <array>
#include <cstddef>

template <typename T, size_t N>
class CyclicBuffer {
public:
    inline void Push(T const& t)
    {
        mItems[mIndex] = t;
        mIndex = (mIndex + 1) % N;
        mCount = std::min(mCount + 1, N);
    }

    inline size_t Size() const
    {
        return mCount;
    }

    inline bool IsFilled() const
    {
        return Size() == N;
    }

    inline T *Data()
    {
        return mItems.data();
    }

    inline const T *Data() const
    {
        return mItems.data();
    }
    
    inline T &Get(size_t i)
    {
        size_t tailLength = N - mIndex;
        
        if (i <= tailLength) {
            return mItems[(mIndex + i) % N];
        } else {
            return mItems[i - tailLength];
        }
    }

    T &operator [](size_t i)
    {
        return Get(i);
    }

private:
    size_t mCount = 0;
    size_t mIndex = 0;
    std::array<T, N> mItems;
}; 