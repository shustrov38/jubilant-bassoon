#pragma once

#if defined(__cpp_char8_t)
   template<typename T>
   const char* u8Cpp20(T&& t) noexcept 
   { 
#pragma warning (disable: 26490)
      return reinterpret_cast<const char*>(t);
#pragma warning (default: 26490)
   }
   #define U8(x) u8Cpp20(u8##x)
#else
   #define U8(x) u8##x
#endif