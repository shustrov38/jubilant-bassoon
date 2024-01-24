#pragma once 

#include <cstdlib>
#include <iomanip>
#include <unistd.h>
#include <iostream>
#include <sys/time.h>

class TQDM
{
public:
    TQDM(size_t total, size_t size)
        : mTotal(total)
        , mCount(0)
        , mSize(size)
        , mAverageTime(0)
        , mLastTime(GetTime())
        , mTotalTime(0)
    {
    }

    void Update(size_t x)
    {
        auto currTime = GetTime();

        ++mCount;
        mTotalTime += ((currTime - mLastTime) / 1000);
        mAverageTime = mTotalTime / mCount;
        mLastTime = currTime;

        double temp = static_cast<double>(x + 1) / static_cast<double>(mTotal);
        std::cout << '\r' << static_cast<size_t>(temp * 100) << "% | ";

        size_t filledSize = temp * mSize;
        for (size_t i = 0; i < filledSize; ++i) {
            std::cout << "█";
        }
        
        for (size_t i = 0; i < mSize - filledSize; ++i) {
            std::cout << ' ';
        }
        
        auto flags = std::cout.flags();
        std::cout << " | " << x + 1 << " / " << mTotal;
        std::cout << std::fixed << std::setprecision(2);
        std::cout << " [" << mTotalTime << "s<" << mTotal * mAverageTime << "s, " << 1 / mAverageTime << "it/s]";
        std::cout.flush();
        std::cout.setf(flags);
    }

private:
    static int64_t GetTime()
    {
        struct timeval time;
        if (gettimeofday(&time, NULL) == -1) {
            return 0LL;
        }
        return (int64_t)time.tv_sec * 1000LL + (int64_t)time.tv_usec / 1000LL;
    }

    size_t mTotal;
    size_t mCount;
    size_t mSize;

    double mAverageTime;
    double mLastTime;
    double mTotalTime;
};