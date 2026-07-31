#pragma once

// `debug(...)` is a function that is a wrapper around `printf` in debug mode,
// and a no-op in release mode.  It is a function, not a macro.
//
// The `NDEBUG` preprocessor macro determines the mode.
//
// Example usage:
//
//     #include <picoro/debug.h>
//
//     int main() {
//       picoro::debug("This only prints in %s mode.\n", "release");
//     }

#ifndef NDEBUG
#include <stdarg.h>
#include <stdio.h>
#endif

namespace picoro {

#ifdef NDEBUG
__attribute__((format(printf, 1, 2)))
inline int debug(const char*, ...) { return 0; }
#else
__attribute__((format(printf, 1, 2)))
inline int debug(const char* format, ...) {
  va_list args;
  va_start(args, format);
  const int rc = vprintf(format, args);
  va_end(args);

  fflush(stdout);
  return rc;
}
#endif

}  // namespace picoro
