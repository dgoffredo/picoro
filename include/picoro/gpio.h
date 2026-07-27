#pragma once

// TODO: documentation

#include <hardware/gpio.h>
#include <hardware/irq.h>
#include <pico/async_context.h>
#include <pico/time.h>

#include <cstdint>
#include <iterator>
#include <span>
#include <utility>

namespace picoro {

struct GPIOEvent {
    absolute_time_t when;
    std::uint32_t mask;
};

class GPIOSubscriber {
public:
    GPIOSubscriber* next;
    enum Result { keep_me, remove_me };
    virtual Result handle_events(std::span<const GPIOEvent> events) = 0;
};

struct MonitoredGPIO {
    // `worker` is the first data member so we can reinterpret_cast.
    // `worker.user_data` is the associated `async_context_t*`.
    async_when_pending_worker_t worker;
    unsigned pin;
    GPIOSubscriber* subscribers;
    volatile unsigned next_write; // cyclic offset into `events`
    volatile unsigned next_read;
    volatile unsigned skipped;
    // Set while do_work() is dispatching, during which `subscribers` holds
    // only newly added subscribers -- see refresh_irq_enabled().
    bool dispatching;
    GPIOEvent events[8];

    static void do_work(async_context_t*, async_when_pending_worker_t*);
};

void gpio_subscribe(async_context_t* ctx, unsigned gpio, GPIOSubscriber&);

void gpio_unsubscribe(unsigned gpio, GPIOSubscriber&);

inline MonitoredGPIO* monitored_gpio_pins[NUM_CORES][NUM_BANK0_GPIOS] = {};

void handle_gpio_irq(unsigned gpio, std::uint32_t event_mask);

// Implementations
// ---------------

// Arm the pin's edge interrupts only while somebody is actually waiting on
// it. A pin nobody is watching can still generate edges -- notably one left
// floating because whatever drives it got powered down -- and each edge wakes
// the event loop for nothing.
inline
void refresh_irq_enabled(MonitoredGPIO& state) {
    if (state.dispatching) {
        // `state.subscribers` currently holds only subscribers added during
        // dispatch; do_work() calls us again once it has spliced the list
        // back together.
        return;
    }
    gpio_set_irq_enabled(state.pin, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE,
                         state.subscribers != nullptr);
}

inline
void gpio_subscribe(async_context_t* ctx, unsigned gpio, GPIOSubscriber& subscriber) {
    auto& ptr = monitored_gpio_pins[get_core_num()][gpio];
    if (ptr == nullptr) {
        // Leak it: it's a microcontroller.
        ptr = new MonitoredGPIO {
            .worker = {
                .next = nullptr, // set by async_context_add_when_pending_worker
                .do_work = &MonitoredGPIO::do_work,
                .work_pending = false,
                .user_data = ctx, // for use by the irq handler
            },
            .pin = gpio,
            .subscribers = &subscriber,
            .next_write = 0,
            .next_read = 0,
            .skipped = 0,
            .dispatching = false,
            .events = {},
        };
        subscriber.next = nullptr;
        async_context_add_when_pending_worker(ctx, &ptr->worker);
        gpio_set_irq_callback(&handle_gpio_irq); // potentially redundant, but harmless
        gpio_set_irq_enabled(gpio, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true);
        // Neither of the two calls above enables the bank's interrupt in the
        // NVIC -- only gpio_set_irq_enabled_with_callback() does that. Without
        // this we'd be relying on some other library (the cyw43 driver, say)
        // having enabled it for us.
        irq_set_enabled(IO_IRQ_BANK0, true);
        return;
    }

    MonitoredGPIO& state = *ptr;
    subscriber.next = state.subscribers;
    state.subscribers = &subscriber;
    refresh_irq_enabled(state); // re-arm: unsubscribing disarms
}

inline
void gpio_unsubscribe(unsigned gpio, GPIOSubscriber& subscriber) {
    MonitoredGPIO& state = *monitored_gpio_pins[get_core_num()][gpio];
    GPIOSubscriber* prev = nullptr;
    for (auto iter = state.subscribers; iter; prev = iter, iter = iter->next) {
        if (iter == &subscriber) {
            (prev ? prev->next : state.subscribers) = iter->next;
            break;
        }
    }
    refresh_irq_enabled(state);
}

inline
void handle_gpio_irq(unsigned gpio, std::uint32_t event_mask) {
    if (monitored_gpio_pins[get_core_num()][gpio] == nullptr) {
        return;
    }

    auto& [worker, pin, subscribers, next_write, next_read, skipped, dispatching, events] = *monitored_gpio_pins[get_core_num()][gpio];
    (void)pin;
    (void)subscribers;
    (void)dispatching;

    if ((next_write + 1) % std::size(events) == next_read) {
        skipped += 1;
        // Still nudge the context. Dropping the event is fine, but dropping
        // the wakeup too is not: if no drain is already scheduled, nothing
        // would ever empty the queue and this pin would wedge for good.
        async_context_set_work_pending(static_cast<async_context_t*>(worker.user_data), &worker);
        return;
    }

    events[next_write] = GPIOEvent{
      .when = get_absolute_time(),
      .mask = event_mask
    };
    next_write = (next_write + 1) % std::size(events);

    async_context_set_work_pending(static_cast<async_context_t*>(worker.user_data), &worker);
}

inline
void MonitoredGPIO::do_work(async_context_t*, async_when_pending_worker_t* worker) {
    auto& self = *reinterpret_cast<MonitoredGPIO*>(worker);

    // Unwrap the ready events from the circular buffer so that subscribers have a contiguous view.
    // Also, advance `self.read_next` so we can get out of the irq handler's way.
    decltype(self.events) events;
    unsigned num_new_events = 0;
    unsigned i = self.next_read;
    for (; i != self.next_write; i = (i + 1) % std::size(self.events)) {
        events[num_new_events] = self.events[i];
        ++num_new_events;
    }
    self.next_read = i;
    const std::span<const GPIOEvent> new_events(events, num_new_events);

    // Give subscribers an opportunity to handle the new events.
    // Upon handling the events, a subscriber might request that we remove it from the list of
    // subscribers.
    // Keep in mind that in handling the events, subscribers might resume coroutines that result in
    // more subscribers being added to `self.subscribers`.
    // The order of subscribers doesn't matter, so we can use forward lists and reverse/splice
    // things without concern.
    self.dispatching = true;
    GPIOSubscriber* head = std::exchange(self.subscribers, nullptr);
    GPIOSubscriber* prev = nullptr;
    for (GPIOSubscriber* iter = head, *next; iter; iter = next) {
        next = iter->next;
        switch (iter->handle_events(new_events)) {
        case GPIOSubscriber::keep_me:
            prev = iter;
            break;
        case GPIOSubscriber::remove_me:
            (prev ? prev->next : head) = next;
        }
    }

    if (prev) {
        prev->next = self.subscribers;
        self.subscribers = head;
    }
    self.dispatching = false;
    refresh_irq_enabled(self); // the last subscriber may have just left
}

} // namespace picoro
