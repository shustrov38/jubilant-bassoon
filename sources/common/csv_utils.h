#pragma once

#include <fstream>
#include <iostream>
#include <string_view>

namespace csv {
template <size_t Period = 0ul>
class Writer {
public:
    Writer(std::string_view const& filename, char delim = ';')
        : mStream(filename.data())
        , mDelim(delim)
        , mStep(Period)
        , mIsDerived(false)
    {
        std::cout << filename << std::endl;
    }

    void WriteRow()
    {
    }

    template <typename Head>
    void WriteRow(Head&& head)
    {
        if (!mStream.is_open()) {
            return;
        }
        if (mStep != Period) {
            mStep += (1 - mIsDerived);
            return;
        }
        mStream << head << '\n';
        mStep = (1 - mIsDerived);
        mIsDerived = false;
    }

    template <typename Head, typename... Tail>
    void WriteRow(Head&& head, Tail&&... tail)
    {
        if (!mStream.is_open()) {
            mStep = Period;
            return;
        }
        if (mStep != Period) {
            mStep += (1 - mIsDerived);
            return;
        }
        mStream << head << mDelim;
        mIsDerived = true;
        WriteRow(tail...);
    }

private:
    bool CanWrite()
    {
        if (mStep == Period) {
            mStep = 0;
            return mStream.is_open();
        }
        return false;
    }

private:
    bool mIsDerived;
    size_t mStep;
    std::ofstream mStream;
    char mDelim;
};
} // namespace csv
