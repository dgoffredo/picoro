#include <pico/async_context_poll.h>
#include <pico/stdio.h>
#include <picoro/coroutine.h>
#include <picoro/drivers/dht22.h>
#include <picoro/event_loop.h>
#include <picoro/sleep.h>

#include <cassert>
#include <chrono>

// Work around `-Werror=unused-variable` in release builds.
#define ASSERT(WHAT)    \
  do {                  \
    assert(WHAT);       \
    (void)sizeof(WHAT); \
  } while (false)

picoro::Coroutine<void> monitor_sensor(async_context_t *ctx,
                                       picoro::dht22::Driver *driver, PIO pio,
                                       uint8_t gpio_pin, const char *name) {
  using Sensor = picoro::dht22::Sensor;
  Sensor sensor(driver, pio, gpio_pin);
  for (;;) {
    co_await picoro::sleep_for(ctx, std::chrono::seconds(2));
    float celsius, humidity_percent;
    if (Sensor::Result rc =
            co_await sensor.measure(&celsius, &humidity_percent)) {
      std::printf("{\"error\": \"%s\"}\n", Sensor::describe(rc));
      sensor.reset();
      continue;
    }
    std::printf(
        "{"
        "\"sensor\": \"%s\", "
        "\"celsius\": %.1f, "
        "\"humidity_percent\": %.1f"
        "}\n",
        name, celsius, humidity_percent);
  }
}

int main() {
  stdio_init_all();

  async_context_poll_t context;
  const bool ok = async_context_poll_init_with_defaults(&context);
  ASSERT(ok);
  async_context_t *const ctx = &context.core;

  constexpr int which_dma_irq = 0;
  picoro::dht22::Driver driver(ctx, which_dma_irq);

  picoro::run_event_loop(
      ctx, monitor_sensor(ctx, &driver, pio0, 15, "top shelf"),
      monitor_sensor(ctx, &driver, pio0, 16, "middle shelf"),
      monitor_sensor(ctx, &driver, pio0, 22, "bottom shelf"));

  // unreachable
  async_context_deinit(ctx);
}
