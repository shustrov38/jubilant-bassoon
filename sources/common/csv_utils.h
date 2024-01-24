#pragma once

#include <fstream>
#include <iostream>
#include <string_view>

namespace csv {
class Writer {
public:
    Writer(std::string_view const& filename, char delim = ';')
        : mStream(filename.data())
        , mDelim(delim)
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
        mStream << head << '\n';
    }

    template <typename Head, typename... Tail>
    void WriteRow(Head&& head, Tail&&... tail)
    {
        if (!mStream.is_open()) {
            return;
        }
        mStream << head << mDelim;
        WriteRow(tail...);
    }

private:
    std::ofstream mStream;
    char mDelim;
};
} // namespace csv
