#pragma once

#include <span>
#include <sstream>
#include <cassert>
#include <fstream>
#include <iostream>
#include <string_view>

namespace csv {
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
