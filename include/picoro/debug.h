#pragma once

// `debug(...)` is a function-like macro that is `printf` in debug mode, and a
// no-op in release mode.
//
// The `NDEBUG` preprocessor macro determines the mode.
//
// Example usage:
//
//     #include <picoro/debug.h>
//
//     int main() {
//       debug("All %d arguments are not evaluated in %s mode.\n", 3, "release");
//     }

#ifdef NDEBUG
#define debug(...) \
  do {             \
  } while (false)
#else
#include <stdio.h>
#define debug(...)       \
  do {                   \
    printf(__VA_ARGS__); \
    fflush(stdout);      \
  } while (false)
#endif
