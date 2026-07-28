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
#include <stdio.h>
#endif

namespace picoro {

#ifdef NDEBUG
inline int debug(const char*, ...) { return 0; }
#else
// The use of a template will cause overloads to proliferate, but it's cleaner
// than using <cstdarg> and will probably get inlined away.
template <typename... Parameters>
int debug(const char* format, Parameters... parameters) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat-security"
  const int rc = printf(format, parameters...);
#pragma GCC diagnostic pop

  fflush(stdout);
  return rc;
}
#endif

}  // namespace picoro
