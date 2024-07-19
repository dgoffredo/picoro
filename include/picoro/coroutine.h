#pragma once

// `class Coroutine<Value>` is a coroutine that `co_return`s a `Value`.  It's
// what's called a "task" in other libraries.
//
// `Coroutine` is eagerly executed, so execution begins as soon as the
// coroutine is invoked — there is no initial suspend.
//
// `Coroutine` doesn't actually do anything, since it doesn't know about the
// scheduler that drives it.  It is meant to be used in conjunction with the
// facilities provided by `sleep.h`, `tcp.h`, and `event_loop.h`.  Those
// components explicitly involve `async_context_t`, which is what manages the
// resumption of suspended `Coroutine`s.
//
// Call `Coroutine::detach()` to allow the coroutine's storage to outlive the
// `Coroutine` object.  This is useful for "fire and forget" scenarios, such as
// handling an `accept`ed TCP connection.  A detached coroutine's storage is
// reclaimed when the coroutine finishes executing.  It is not possible to
// `co_await` a return value from a detached coroutine.
//
// Use `Coroutine<Value>` as the return value of any function that contains
// `co_await` expressions or `co_return` statements.
//
// Example usage:
//
//     #include <picoro/coroutine.h>
//     #include <picoro/event_loop.h>
//     #include <picoro/sleep.h>
//
//     #include <pico/async_context_poll.h>
//     #include <pico/stdlib.h>
//
//     #include <algorithm>
//     #include <cassert>
//     #include <chrono>
//     #include <iostream>
//     #include <string>
//
//     Coroutine<void> every_second(async_context_t *context) {
//       for (;;) {
//         co_await picoro::sleep_for(context, std::chrono::seconds(1));
//         std::cout << "every_second: here I am\n";
//       }
//     }
//
//     Coroutine<std::string> reverse_later(
//         async_context_t *context,
//         std::string text,
//         std::chrono::seconds delay) {
//       co_await picoro::sleep_for(context, delay);
//       std::reverse(text.begin(), text.end());
//       co_return text;
//     }
//
//     Coroutine<void> drift_away(async_context_t *context) {
//       every_second(context).detach();
//       for (auto delay = std::chrono::milliseconds(100); ; delay *= 2) {
//         std::cout << co_await reverse_later(context, "tacocat", delay) << '\n';
//       }
//     }
//
//     int main() {
//       stdio_init_all();
//       async_context_poll_t context = {};
//       const bool ok = async_context_poll_init_with_defaults(&context);
//       assert(ok);
//       picoro::run_event_loop(&context.core, drift_away(&context.core));
//     }

#include <coroutine>
#include <cstddef>
#include <cstdlib>
#include <memory>
#include <new>
#include <utility>

namespace picoro {
//              _.---..._
//           ./^         ^-._
//         ./^C===.         ^\.   /\.
//        .|'     \\        _ ^|.^.|
//   ___.--'_     ( )  .      ./ /||
//  /.---^T\      ,     |     / /|||
// C'   ._`|  ._ /  __,-/    / /-,||
//      \ \/    ;  /O  / _    |) )|,
//       i \./^O\./_,-^/^    ,;-^,'
//        \ |`--/ ..-^^      |_-^
//         `|  \^-           /|:
//          i.  .--         / '|.
//           i   =='       /'  |\._
//         _./`._        //    |.  ^-ooo.._
//  _.oo../'  |  ^-.__./X/   . `|    |#######b
// d####     |'      ^^^^   /   |    _\#######
// #####b ^^^^^^^^--. ...--^--^^^^^^^_.d######
// ######b._         Y            _.d#########
// ##########b._     |        _.d#############
//
//                     --- Steven J. Simmons

class FinalAwaitable {
  std::coroutine_handle<> coroutine_;
  bool detached_ = false;

 public:
  FinalAwaitable(std::coroutine_handle<> coroutine, bool detached);

  bool await_ready() noexcept;
  std::coroutine_handle<> await_suspend(std::coroutine_handle<>) noexcept;
  void await_resume() noexcept;
};

// - - - - - - - - - - - - - - - - - - - - - - - - - - - -

template <typename Ret>
class Awaiter;

template <typename Ret>
class Promise;

template <typename Ret>
class Coroutine {
 public:
  struct Deleter {
    void operator()(Promise<Ret> *);
  };

  using promise_type = Promise<Ret>;
  using UniqueHandle = std::unique_ptr<Promise<Ret>, Deleter>;

 private:
  UniqueHandle promise_;

 public:
  explicit Coroutine(UniqueHandle promise);

  Coroutine(Coroutine &&) = default;

  Coroutine() = delete;
  Coroutine(const Coroutine &) = delete;

  Awaiter<Ret> operator co_await();

  void detach();
};

// - - - - - - - - - - - - - - - - - - - - - - - - - - - -

template <typename Ret>
class Promise {
  // `continuation_` is what runs after we're done.
  // By default, it's a no-op handle, which means "nothing to do after."
  // Sometimes, `Coroutine::Awaiter::await_suspend` will assign a non-no-op
  // handle to `continuation_`.
  // `continuation_` is passed to `FinalAwaitable` in `final_suspend`.
  // `FinalAwaitable` will then return it in `FinalAwaitable::await_suspend`,
  // which will cause it to be `.resume()`d.
  std::coroutine_handle<> continuation_;
  bool detached_ = false;

  // `value_` is where we store the `foo` from `co_return foo;`.
  alignas(Ret) std::byte value_[sizeof(Ret)];

  friend class Awaiter<Ret>;

 public:
  Promise();
  Promise(const Promise &) = delete;
  Promise(Promise &&) = delete;
  ~Promise();

  void detach();

  Coroutine<Ret> get_return_object();
  std::suspend_never initial_suspend();
  FinalAwaitable final_suspend() noexcept;
  // `return_value` is a template so that the argument to `co_return` need not
  // be exactly a `Ret`, but anything convertible to `Ret`.
  template <typename Value>
  void return_value(Value &&);

  void unhandled_exception();
};

// - - - - - - - - - - - - - - - - - - - - - - - - - - - -

template <>
class Promise<void> {
  // `continuation_` is what runs after we're done.
  // By default, it's a no-op handle, which means "nothing to do after."
  // Sometimes, `Coroutine::Awaiter::await_suspend` will assign a non-no-op
  // handle to `continuation_`.
  // `continuation_` is passed to `FinalAwaitable` in `final_suspend`.
  // `FinalAwaitable` will then return it in `FinalAwaitable::await_suspend`,
  // which will cause it to be `.resume()`d.
  std::coroutine_handle<> continuation_;
  bool detached_ = false;

  friend class Awaiter<void>;

 public:
  Promise();
  Promise(const Promise &) = delete;
  Promise(Promise &&) = delete;

  void detach();

  Coroutine<void> get_return_object();
  std::suspend_never initial_suspend();
  FinalAwaitable final_suspend() noexcept;
  void return_void();

  void unhandled_exception();
};

// - - - - - - - - - - - - - - - - - - - - - - - - - - - -

template <typename Ret>
class Awaiter {
  Promise<Ret> *promise_;

 public:
  explicit Awaiter(Promise<Ret> *promise);

  bool await_ready();
  bool await_suspend(std::coroutine_handle<> continuation);
  Ret await_resume();
};

// - - - - - - - - - - - - - - - - - - - - - - - - - - - -

template <>
class Awaiter<void> {
  Promise<void> *promise_;

 public:
  explicit Awaiter(Promise<void> *promise);

  bool await_ready();
  bool await_suspend(std::coroutine_handle<> continuation);
  void await_resume();
};

// Implementations
// ===============

// class Coroutine<Ret>
// --------------------
template <typename Ret>
void Coroutine<Ret>::Deleter::operator()(Promise<Ret> *promise) {
  std::coroutine_handle<Promise<Ret>>::from_promise(*promise).destroy();
}

template <typename Ret>
inline Coroutine<Ret>::Coroutine(UniqueHandle promise)
    : promise_(std::move(promise)) {}

template <typename Ret>
inline Awaiter<Ret> Coroutine<Ret>::operator co_await() {
  return Awaiter<Ret>(promise_.get());
}

template <typename Ret>
inline void Coroutine<Ret>::detach() {
  promise_.release()->detach();
}

// class FinalAwaitable
// -------------------------------
inline FinalAwaitable::FinalAwaitable(std::coroutine_handle<> coroutine,
                                      bool detached)
    : coroutine_(coroutine), detached_(detached) {}

inline bool FinalAwaitable::await_ready() noexcept { return detached_; }

inline std::coroutine_handle<> FinalAwaitable::await_suspend(
    std::coroutine_handle<>) noexcept {
  return coroutine_;
}

inline void FinalAwaitable::await_resume() noexcept {}

// class Promise<Ret>
// ------------------
template <typename Ret>
Promise<Ret>::Promise() : continuation_(std::noop_coroutine()) {}

template <typename Ret>
Promise<Ret>::~Promise() {
  Ret *value = std::launder(reinterpret_cast<Ret *>(&value_[0]));
  value->~Ret();
}

template <typename Ret>
void Promise<Ret>::detach() {
  detached_ = true;
}

template <typename Ret>
Coroutine<Ret> Promise<Ret>::get_return_object() {
  return Coroutine<Ret>(typename Coroutine<Ret>::UniqueHandle(this));
}

template <typename Ret>
std::suspend_never Promise<Ret>::initial_suspend() {
  return std::suspend_never();
}

template <typename Ret>
FinalAwaitable Promise<Ret>::final_suspend() noexcept {
  return FinalAwaitable(continuation_, detached_);
}

template <typename Ret>
template <typename Value>
void Promise<Ret>::return_value(Value &&value) {
  new (&value_[0]) Ret(std::forward<Value>(value));
}

template <typename Ret>
void Promise<Ret>::unhandled_exception() {
  std::terminate();
}

// class Promise<void>
// -------------------
inline Promise<void>::Promise() : continuation_(std::noop_coroutine()) {}

inline void Promise<void>::detach() { detached_ = true; }

inline Coroutine<void> Promise<void>::get_return_object() {
  return Coroutine<void>(Coroutine<void>::UniqueHandle(this));
}

inline std::suspend_never Promise<void>::initial_suspend() {
  return std::suspend_never();
}

inline FinalAwaitable Promise<void>::final_suspend() noexcept {
  return FinalAwaitable(continuation_, detached_);
}

inline void Promise<void>::return_void() {}

inline void Promise<void>::unhandled_exception() { std::terminate(); }

// class Awaiter<Ret>
// ------------------
template <typename Ret>
Awaiter<Ret>::Awaiter(Promise<Ret> *promise) : promise_(promise) {}

template <typename Ret>
bool Awaiter<Ret>::await_ready() {
  auto handle = std::coroutine_handle<Promise<Ret>>::from_promise(*promise_);
  return handle.done();
}

template <typename Ret>
bool Awaiter<Ret>::await_suspend(std::coroutine_handle<> continuation) {
  promise_->continuation_ = continuation;
  return true;
}

template <typename Ret>
Ret Awaiter<Ret>::await_resume() {
  Ret *value = std::launder(reinterpret_cast<Ret *>(&promise_->value_[0]));
  return std::move(*value);
}

// class Awaiter<void>
// -------------------
inline Awaiter<void>::Awaiter(Promise<void> *promise) : promise_(promise) {}

inline bool Awaiter<void>::await_ready() {
  auto handle = std::coroutine_handle<Promise<void>>::from_promise(*promise_);
  return handle.done();
}

inline bool Awaiter<void>::await_suspend(std::coroutine_handle<> continuation) {
  promise_->continuation_ = continuation;
  return true;
}

inline void Awaiter<void>::await_resume() {}

}  // namespace picoro
