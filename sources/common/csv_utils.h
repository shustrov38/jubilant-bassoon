#pragma once

#include <span>
#include <sstream>
#include <cassert>
#include <fstream>
#include <iostream>
#include <string_view>

namespace csv {
template <size_t Period = 0ul>
class PeriodWriter {
public:
    PeriodWriter(std::string_view const& filename, char delim = ';')
        : mStream(filename.data())
        , mDelim(delim)
        , mStep(Period)
        , mIsDerived(false)
    {
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

class Writer {
public:
    Writer(std::string_view const& filename)
        : mStream(filename.data())
    {
    }

    ~Writer()
    {
        mStream.flush();
    }

    Writer &Delimiter(char ch)
    {
        mDelim = ch;
        return *this;
    }

    Writer &Header(std::span<std::string> header)
    {
        assert(!mHeaderIsSet && !header.empty());
        mHeaderIsSet = true;
        mColumnsCount = header.size();
        mStream << header.front();
        for (size_t i = 1; i < header.size(); ++i) {
            mStream << mDelim << header[i];
        }
        mStream << '\n';
        return *this;
    }

    template <typename Head, typename... Tail>
    Writer &WriteRow(Head&& head, Tail&&... tail)
    {
        assert(mHeaderIsSet);
        [[maybe_unused]] size_t constexpr n = 1 + sizeof...(Tail);
        assert(n == mColumnsCount);

        mStream << head;
        ((mStream << mDelim << tail), ...);
        mStream << '\n';
        return *this;
    }

private:
    std::ofstream mStream;
    char mDelim {';'};
    bool mHeaderIsSet {false};
    size_t mColumnsCount;
};
} // namespace csv
