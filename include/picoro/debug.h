#pragma once

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
