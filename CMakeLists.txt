project(picoro CXX C ASM)

add_library(picoro_headers INTERFACE)
target_include_directories(picoro_headers INTERFACE include)
# Enable coroutines (GCC 10 requires a flag).

add_library(picoro_coroutine INTERFACE)
target_compile_options(picoro_coroutine INTERFACE $<$<COMPILE_LANGUAGE:CXX>:-fcoroutines>)
target_link_libraries(picoro_coroutine INTERFACE
  picoro_headers
)

add_library(picoro_debug INTERFACE)
target_link_libraries(picoro_debug INTERFACE
  picoro_headers
  pico_stdio
)

add_library(picoro_event_loop INTERFACE)
target_link_libraries(picoro_event_loop INTERFACE
  picoro_headers
  picoro_debug
  pico_async_context_poll
)

add_library(picoro_sleep INTERFACE)
target_link_libraries(picoro_sleep INTERFACE
  picoro_headers
  picoro_event_loop
  pico_time
)

add_library(picoro_tcp INTERFACE)
target_link_libraries(picoro_tcp INTERFACE
  picoro_headers
  picoro_debug
  pico_cyw43_arch_lwip_poll
)

add_library(picoro_drivers_scd4x INTERFACE)
target_link_libraries(picoro_drivers_scd4x INTERFACE
  picoro_headers
  picoro_coroutine
  picoro_sleep
  hardware_i2c
)
