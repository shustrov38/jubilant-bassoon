#pragma once

#include <string>
#include <cstdlib>

#include <iostream>

class EvnironmentVariable {
public:
    EvnironmentVariable(std::string const& name)
        : mName(name)
    {
        char const* value = std::getenv(mName.data());
        if (value) {
            mValue = std::string(value);
        }
    }

    operator bool() const
    {
        return !mValue.empty();
    }

    const std::string_view GetName() const
    {
        return mName;
    }

    const std::string_view GetValue() const
    {
        return mValue;
    }

private:
    std::string mName;
    std::string mValue;
};

namespace options {
#define DEFINE_OPTION(name)                                     \
    static EvnironmentVariable name("ACV_"#name"_FILE_NAME");   \
    EvnironmentVariable& Get##name()                            \
    {                                                           \
        return name;                                            \
    }

#define GET_OPTION(name) Get##name()
} // namespace options