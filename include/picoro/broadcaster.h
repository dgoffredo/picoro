#pragma once

// TODO

#include <coroutine>
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

template <typename Value>
struct Waiter;

template <typename Value>
class NextAwaitable;

template <typename Value>
class Broadcaster {
    Waiter<Value> *head_;

 public:
    void publish(const Value& value);
    NextAwaitable<Value> next();
};

template <typename Value>
struct Waiter {
    Waiter *next;
    std::coroutine_handle<> continuation;
    alignas(Value) char buffer[sizeof(Value)];

    ~Waiter();
};

template <typename Value>
class NextAwaitable {
    Waiter<Value>*& head_;
    Waiter<Value> waiter_;

 public:
    explicit NextAwaitable(Waiter<Value>*& head);
    bool await_ready();
    void await_suspend(std::coroutine_handle<> continuation);
    Value await_resume();
};


// Implementations
// ===============

// class Broadcaster<Value>
// ------------------------
template <typename Value>
void Broadcaster<Value>::publish(const Value& value) {
    Waiter<Value> *waiter = std::exchange(head_, nullptr);
    while (waiter) {
        Waiter<Value> *next = waiter->next;
        new (waiter->buffer) Value(value);
        waiter->continuation();
        waiter = next;
    }
}

template <typename Value>
NextAwaitable<Value> Broadcaster<Value>::next() {
    return NextAwaitable<Value>(head_);
}

// struct Waiter<Value>
// --------------------
template <typename Value>
Waiter<Value>::~Waiter() {
    std::launder(reinterpret_cast<Value*>(buffer))->~Value();
}

// class NextAwaitable<Value>
// --------------------------
template <typename Value>
NextAwaitable<Value>::NextAwaitable(Waiter<Value>*& head) : head_(head) {}

template <typename Value>
bool NextAwaitable<Value>::await_ready() {
    return false;
}

template <typename Value>
void NextAwaitable<Value>::await_suspend(std::coroutine_handle<> continuation) {
    waiter_.continuation = continuation;
    // Prepend `waiter_` onto the waiter list.
    waiter_.next = head_;
    head_ = &waiter_;
}

template <typename Value>
Value NextAwaitable<Value>::await_resume() {
    return std::move(*std::launder(reinterpret_cast<Value*>(waiter_.buffer)));
}

} // namespace picoro
