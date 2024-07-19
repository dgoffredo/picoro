#pragma once

// This component provides a socket-like coroutine facility for server-side TCP
// connections.
//
// `class Listener` listens on all interfaces on a specified port with a
// specified backlog.  A coroutine can `co_await` the listener's `accept()`
// member function to get a tuple `(Connection, err_t)`, Go style. `err_t` is
// zero (`ERR_OK`) if no error occurred.
//
// `class Connection` is a connection to a client.
//
// A coroutine can `co_await` the connection's `send(std::string_view)` member
// function to send data to the client.  `send` returns a tuple `(int, err_t)`,
// where the `int` is the number of bytes sent.  The `int` might be less than
// the `std::string_view::size()` if an error occurred, i.e. if the `err_t` is
// nonzero.
//
// A coroutine can `co_await` the connection's `recv(char*, int)` member
// function to receive from the client up to `int` bytes into the buffer
// pointed to by `char*`.  `recv` returns a tuple `(int, err_t)`, where the
// `int` is the number of bytes received.  `recv` returns as soon as any data
// is available, so the number of bytes received might be less than the number
// of bytes requested, even if no error occurs.
//
// Both `Listener` and `Connection` have a member function `close()` that
// closes the associated socket.  `close` is called in the objects' destructors
// if it has not been called already.
//
// Example usage:
//
//     #include <picoro/coroutine.h>
//     #include <picoro/tcp.h>
//     #include <pico/async_context_poll.h>
//     #include <pico/cyw43_arch.h>
//     #include <cassert>
//     #include <coroutine>
//     #include <string_view>
//     #include <tuple>
//     #include <utility>
//
//     picoro::Coroutine<void> handle_client(picoro::Connection conn) {
//       char recvbuf[2048];
//       auto [count, err] = co_await conn.recv(recvbuf, sizeof recvbuf);
//       if (err) {
//         co_return;
//       }
//       const std::string_view response =
//         "HTTP/1.1 200 OK\r\n"
//         "Connection: close\r\n"
//         "Content-Type: text/plain\r\n"
//         "\r\n"
//         "Hi!\n";
//       std::tie(count, err) = co_await conn.send(response);
//     }
//
//     picoro::Coroutine<void> http_server(int port, int listen_backlog) {
//       auto [listener, err] = picoro::listen(port, listen_backlog);
//       if (err) {
//         co_return;
//       }
//
//       for (;;) {
//         auto [conn, err] = co_await listener.accept();
//         if (err) {
//           continue;
//         }
//         handle_client(std::move(conn)).detach();
//       }
//     }
//
//     int main() {
//       async_context_poll_t context = {};
//       const bool ok = async_context_poll_init_with_defaults(&context);
//       assert(ok);
//
//       cyw43_arch_set_async_context(&context.core);
//       const int rc = cyw43_arch_init_with_country(CYW43_COUNTRY_USA); // 🇺🇸 🦅
//       assert(rc == 0);
//
//       const int port = 80;
//       const int backlog = 1;
//       picoro::run_event_loop(&context.core, http_server(port, backlog));
//
//       // unreachable
//       cyw43_arch_deinit();
//       async_context_deinit(&context.core);
//     }

#include <lwip/pbuf.h>
#include <lwip/tcp.h>
#include <picoro/debug.h>

#include <algorithm>
#include <coroutine>
#include <cstddef>
#include <memory>
#include <queue>
#include <string_view>
#include <tuple>

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

const char *lwip_describe(err_t error);

class Listener;
class Connection;
class AcceptAwaiter;
class SendAwaiter;
class RecvAwaiter;

class Listener {
 public:
  struct State {
    err_t error;
    tcp_pcb *pcb;  // 👾 🤖 PROTOCOL CONTROL BLOCK 🤖 👾
    std::size_t backlog;
    std::queue<tcp_pcb *> unaccepted;
    std::queue<AcceptAwaiter *> accepters;
  };

 private:
  std::unique_ptr<State> state_;

  friend class AcceptAwaiter;

  static err_t on_accept(void *user_data, tcp_pcb *client_pcb, err_t err);

 public:
  explicit Listener(int port, int backlog);
  Listener(Listener &&) = default;
  ~Listener();

  AcceptAwaiter accept();
  err_t close();

  err_t error() const;
};

std::tuple<Listener, err_t> listen(int port, int backlog);

class Connection {
 public:
  struct State {
    tcp_pcb *client_pcb;   // 👾 🤖 PROTOCOL CONTROL BLOCK 🤖 👾
    std::string received;  // TODO: consider making a `dequeue<char>`
    std::queue<SendAwaiter *> senders;
    std::queue<RecvAwaiter *> receivers;
  };

 private:
  std::unique_ptr<State> state_;

  static err_t on_recv(void *user_data, tcp_pcb *client_pcb, pbuf *buffer,
                       err_t error);
  static err_t on_sent(void *user_data, tcp_pcb *client_pcb, u16_t length);
  static void on_err(void *user_data, err_t err);

 public:
  explicit Connection(tcp_pcb *client_pcb);
  Connection(Connection &&) = default;
  ~Connection();

  SendAwaiter send(std::string_view data);
  RecvAwaiter recv(char *destination, int size);
  err_t close();
};

class AcceptAwaiter {
  Listener::State *listener;

  tcp_pcb *client_pcb;  // 👾 🤖 PROTOCOL CONTROL BLOCK 🤖 👾
  err_t error;
  std::coroutine_handle<> continuation;

  friend class Listener;

 public:
  explicit AcceptAwaiter(Listener::State *listener);

  bool await_ready();
  void await_suspend(std::coroutine_handle<> continuation);
  std::tuple<Connection, err_t> await_resume();
};

class RecvAwaiter {
  Connection::State *connection;
  char *destination;
  std::size_t length;
  std::size_t received;
  err_t error;
  std::coroutine_handle<> continuation;

  friend class Connection;

 public:
  RecvAwaiter(Connection::State *connection, char *destination, int size);

  bool await_ready();
  void await_suspend(std::coroutine_handle<> continuation);
  std::tuple<int, err_t> await_resume();
};

class SendAwaiter {
  Connection::State *connection;
  std::size_t length;
  std::size_t remaining;
  err_t error;
  std::coroutine_handle<> continuation;

  friend class Connection;

 public:
  SendAwaiter(Connection::State *connection, std::string_view data);

  bool await_ready();
  void await_suspend(std::coroutine_handle<> continuation);
  std::tuple<int, err_t> await_resume();
};

// Implementations
// ===============

// const char *lwip_describe(err_t error)
// --------------------------------------
inline const char *lwip_describe(err_t error) {
  switch (error) {
    case ERR_OK:
      return "[ERR_OK] No error, everything OK";
    case ERR_MEM:
      return "[ERR_MEM] Out of memory";
    case ERR_BUF:
      return "[ERR_BUF] Buffer error";
    case ERR_TIMEOUT:
      return "[ERR_TIMEOUT] Timeout";
    case ERR_RTE:
      return "[ERR_RTE] Routing problem";
    case ERR_INPROGRESS:
      return "[ERR_INPROGRESS] Operation in progress";
    case ERR_VAL:
      return "[ERR_VAL] Illegal value";
    case ERR_WOULDBLOCK:
      return "[ERR_WOULDBLOCK] Operation would block";
    case ERR_USE:
      return "[ERR_USE] Address in use";
    case ERR_ALREADY:
      return "[ERR_ALREADY] Already connecting";
    case ERR_ISCONN:
      return "[ERR_ISCONN] Conn already established";
    case ERR_CONN:
      return "[ERR_CONN] Not connected";
    case ERR_IF:
      return "[ERR_IF] Low-level netif error";
    case ERR_ABRT:
      return "[ERR_ABRT] Connection aborted";
    case ERR_RST:
      return "[ERR_RST] Connection reset";
    case ERR_CLSD:
      return "[ERR_CLSD] Connection closed";
    case ERR_ARG:
      return "[ERR_ARG] Illegal argument";
  }
  return "Unknown lwIP error code";
}

// class Listener
// --------------
inline Listener::Listener(int port, int backlog)
    : state_(std::make_unique<Listener::State>()) {
  state_->error = ERR_OK;
  state_->pcb = nullptr;
  state_->backlog = backlog;

  state_->pcb = tcp_new_ip_type(IPADDR_TYPE_ANY);
  if (!state_->pcb) {
    state_->error = ERR_MEM;
    return;
  }

  if ((state_->error = tcp_bind(state_->pcb, IP_ADDR_ANY, port))) {
    return;
  }
  state_->pcb =
      tcp_listen_with_backlog_and_err(state_->pcb, backlog, &state_->error);
  if (state_->error) {
    return;
  }
  tcp_arg(state_->pcb, state_.get());
  tcp_accept(state_->pcb, &Listener::on_accept);
}

inline Listener::~Listener() { (void)close(); }

inline AcceptAwaiter Listener::accept() { return AcceptAwaiter(state_.get()); }

inline err_t Listener::close() {
  if (!state_ || !state_->pcb) {
    // We were moved from, closed, or otherwise invalid.
    return ERR_ARG;
  }

  // Abort any connections that we received but didn't accept().
  while (!state_->unaccepted.empty()) {
    tcp_pcb *client_pcb = state_->unaccepted.front();
    state_->unaccepted.pop();
    tcp_abort(client_pcb);
  }

  // Return an error to any pending accept()ers.
  while (!state_->accepters.empty()) {
    AcceptAwaiter *accepter = state_->accepters.front();
    state_->accepters.pop();
    accepter->error = ERR_CLSD;  // TODO: technically not "connection closed"
    accepter->continuation.resume();
  }

  // Close our  👾 🤖 PROTOCOL CONTROL BLOCK 🤖 👾.
  // For client sockets, setting the callbacks to nullptr prevents a crash,
  // even though the pattern is undocumented.  For a listening socket, I'm not
  // sure, but let's do it to be safe.
  tcp_accept(state_->pcb, nullptr);
  state_->error = tcp_close(state_->pcb);
  state_->pcb = nullptr;

  return state_->error;
}

inline err_t Listener::error() const { return state_->error; }

inline err_t Listener::on_accept(void *user_data, tcp_pcb *client_pcb,
                                 err_t error) {
  debug("in Listener::on_accept\n");
  auto *state = static_cast<Listener::State *>(user_data);

  if (!state->accepters.empty()) {
    AcceptAwaiter *accepter = state->accepters.front();
    state->accepters.pop();
    if (error) {
      accepter->error = error;
    } else {
      accepter->client_pcb = client_pcb;
    }
    accepter->continuation.resume();
    return ERR_OK;
  }

  // There are no accepters. Add `client_pcb` to the backlog unless there isn't
  // any space.
  if (state->unaccepted.size() == state->backlog) {
    tcp_abort(client_pcb);
    return ERR_ABRT;
  }

  state->unaccepted.push(client_pcb);
  return ERR_OK;
}

inline std::tuple<Listener, err_t> listen(int port, int backlog) {
  Listener listener(port, backlog);
  err_t error = listener.error();
  return {std::move(listener), error};
}

// class AcceptAwaiter
// -------------------
inline AcceptAwaiter::AcceptAwaiter(Listener::State *listener)
    : listener(listener), client_pcb(nullptr), error(ERR_OK) {
  debug("AcceptAwaiter constructor\n");
  // If the listener has a connection ready for us, then take it.
  // Otherwise, await_ready() will subsequently return false, and then
  // await_suspend() will enqueue us onto listener->accepters.
  if (listener->unaccepted.empty() || !listener->accepters.empty()) {
    return;
  }

  client_pcb = listener->unaccepted.front();
  listener->unaccepted.pop();
  debug("end AcceptAwaiter constructor\n");
}

inline bool AcceptAwaiter::await_ready() {
  debug("AcceptAwaiter::await_ready()\n");
  return client_pcb != nullptr;
}

inline void AcceptAwaiter::await_suspend(std::coroutine_handle<> continuation) {
  debug("AcceptAwaiter::await_suspend()\n");
  this->continuation = continuation;
  listener->accepters.push(this);
}

inline std::tuple<Connection, err_t> AcceptAwaiter::await_resume() {
  debug("AcceptAwaiter::await_resume()\n");
  return {Connection(client_pcb), error};
}

// class Connection
// ----------------
inline Connection::Connection(tcp_pcb *client_pcb) {
  if (!client_pcb) {
    return;
  }
  state_ = std::make_unique<Connection::State>();
  state_->client_pcb = client_pcb;

  debug("Connection::Connection is registering callbacks\n");
  tcp_arg(client_pcb, state_.get());
  tcp_sent(client_pcb, &Connection::on_sent);
  tcp_recv(client_pcb, &Connection::on_recv);
  tcp_err(client_pcb, &Connection::on_err);
  debug("Connection::Connection finished registering callbacks\n");
}

inline Connection::~Connection() { (void)close(); }

inline err_t Connection::on_recv(void *user_data, tcp_pcb *, pbuf *buffer,
                                 err_t error) {
  debug("in Connection::on_recv\n");
  auto *state = static_cast<Connection::State *>(user_data);

  struct Guard {
    pbuf *buffer;
    ~Guard() {
      if (buffer) {
        pbuf_free(buffer);
      }
      debug("finished on_recv\n");
    }
  } guard{buffer};

  if (error || buffer == nullptr) {
    // error or connection closed
    debug("on_recv: error or connection closed. error: %s buffer: %p\n",
          lwip_describe(error), buffer);
    // TODO: Can `buffer` have data in it if `error != ERR_OK`? If so, should
    // we deliver the data to receivers?
    while (!state->receivers.empty()) {
      RecvAwaiter *receiver = state->receivers.front();
      state->receivers.pop();
      receiver->error = error;
      receiver->continuation.resume();
    }
    return ERR_OK;
  }

  // data received
  // First, append it to the end of `state->received`.
  // Then look for receivers to fill up with data from the beginning of
  // `state->received`.
  std::string &received = state->received;  // brevity
  const std::size_t old_size = received.size();
  received.resize(old_size + buffer->tot_len);
  const u16_t buffer_offset = 0;
  const u16_t copied = pbuf_copy_partial(buffer, received.data() + old_size,
                                         buffer->tot_len, buffer_offset);
  received.resize(old_size + copied);
  debug("receive buffer is now: %s\n", received.c_str());
  // Deal out data from the beginning of `received` until we're either out of
  // data or out of receivers.
  std::size_t i = 0;
  while (i < received.size() && !state->receivers.empty()) {
    RecvAwaiter *receiver = state->receivers.front();
    const auto to_copy =
        std::min<std::size_t>(receiver->length, received.size() - i);
    std::copy_n(received.begin() + i, to_copy, receiver->destination);
    i += to_copy;
    receiver->received += to_copy;
    // This receiver has received its requested data, and so now can be
    // resumed.  Also, we can tell lwIP that we've "processed" however much
    // data the receiver requested.
    state->receivers.pop();
    receiver->continuation.resume();
    tcp_recved(state->client_pcb, to_copy);
  }
  received.erase(0, i);

  return ERR_OK;
}

inline err_t Connection::on_sent(void *user_data, tcp_pcb *, u16_t length) {
  debug("in Connection::on_sent. length: %hu\n", length);
  auto *state = static_cast<Connection::State *>(user_data);

  // Resume any senders that are "filled up" by the client's acknowledgement of
  // `length` bytes.
  while (length && !state->senders.empty()) {
    SendAwaiter *sender = state->senders.front();
    const auto to_ack = std::min<std::size_t>(sender->remaining, length);
    length -= to_ack;
    sender->remaining -= to_ack;
    if (sender->remaining == 0) {
      state->senders.pop();
      sender->continuation.resume();
    }
  }

  return ERR_OK;
}

inline void Connection::on_err(void *user_data, err_t error) {
  debug("in Connection::on_err. user_data: %p error: %s\n", user_data,
        lwip_describe(error));
  auto *state = static_cast<Connection::State *>(user_data);

  // The pcb is already freed (per lwIP's documentation), so set it to null in
  // `state`. This way, we won't try to `tcp_close` it in the future.
  state->client_pcb = nullptr;

  // Convey the error to all senders and all receivers.
  const auto consume = [error](auto &queue) {
    while (queue.empty()) {
      auto *awaiter = queue.front();
      queue.pop();
      awaiter->error = error;
      awaiter->continuation.resume();
    }
  };

  consume(state->senders);
  consume(state->receivers);
  debug("end Connection::on_err\n");
}

inline SendAwaiter Connection::send(std::string_view data) {
  return SendAwaiter(state_.get(), data);
}

inline RecvAwaiter Connection::recv(char *destination, int size) {
  return RecvAwaiter(state_.get(), destination, size);
}

inline err_t Connection::close() {
  if (!state_) {
    // We were moved from or invalid to begin with.
    return ERR_ARG;
  }

  if (state_->client_pcb == nullptr) {
    // We were already closed.
    return ERR_CLSD;
  }

  debug("Connection::close() is deregistering callbacks.\n");
  tcp_sent(state_->client_pcb, nullptr);
  tcp_recv(state_->client_pcb, nullptr);
  tcp_err(state_->client_pcb, nullptr);
  const err_t error = tcp_close(state_->client_pcb);
  state_->client_pcb = nullptr;

  // Wake up senders and receivers. Deliver a ERR_CLSD (connection closed)
  // error to them.
  const auto consume = [](auto &queue) {
    while (!queue.empty()) {
      auto *awaiter = queue.front();
      queue.pop();
      awaiter->error = ERR_CLSD;
      awaiter->continuation.resume();
    }
  };

  consume(state_->senders);
  consume(state_->receivers);

  return error;  // whatever `tcp_close` returned
}

// class RecvAwaiter
// -----------------
inline RecvAwaiter::RecvAwaiter(Connection::State *connection,
                                char *destination, int size)
    : connection(connection),
      destination(destination),
      length(size),
      received(0),
      error(ERR_OK) {
  debug("in RecvAwaiter constructor\n");
  // If `connection` is null, we return (0, ERR_CLSD) without suspending.
  if (!connection) {
    debug("in RecvAwaiter constructor: connection is null\n");
    error = ERR_CLSD;
    return;
  }

  // If nobody else is enqueued to consume received data from the connection,
  // then consume data if it's already available.  Then we won't even have to
  // suspend.
  // TODO
  if (connection->receivers.empty()) {
    debug("in RecvAwaiter constructor: no other receivers\n");
    const auto to_consume =
        std::min<std::size_t>(connection->received.size(), size);
    debug("RecvAwaiter constructor: going to consume %u bytes\n", to_consume);
    std::copy_n(connection->received.begin(), to_consume, destination);
    debug("RecvAwaiter constructor: returned from copy_n\n");
    this->received += to_consume;
    connection->received.erase(0, to_consume);
    debug(
        "RecvAwaiter constructor: erased the relevant prefix of "
        "connection->received\n");
  }

  debug("at end of RecvAwaiter constructor\n");
}

inline bool RecvAwaiter::await_ready() {
  debug("RecvAwaiter::await_ready: %d\n", error || received != 0);
  return error || received != 0;
}

void RecvAwaiter::await_suspend(std::coroutine_handle<> continuation) {
  debug("RecvAwaiter::await_suspend. this: %p, connection: %p\n", this,
        connection);
  this->continuation = continuation;
  assert(connection);
  connection->receivers.push(this);
  debug("finished RecvAwaiter::await_suspend\n");
}

inline std::tuple<int, err_t> RecvAwaiter::await_resume() {
  debug("RecvAwaiter::await_resume\n");
  return {received, error};
}

// class SendAwaiter
// -----------------
inline SendAwaiter::SendAwaiter(Connection::State *connection,
                                std::string_view data)
    : connection(connection),
      length(data.size()),
      remaining(data.size()),
      error(ERR_OK) {
  // TODO: `data` doesn't have to be null-terminated, but it is.
  debug("in SendAwaiter constructor. connection: %p data.size(): %u data: %s\n",
        connection, data.size(), data.data());
  // If `connection` is null, we return (0, ERR_CLSD) without suspending.
  if (!connection) {
    error = ERR_CLSD;
    return;
  }

  const u8_t flags = TCP_WRITE_FLAG_COPY;
  error = tcp_write(connection->client_pcb, data.data(), data.size(), flags);
  debug("in SendAwaiter constructor. tcp_write returned %s\n",
        lwip_describe(error));
  // `tcp_write` enqueues data for sending "later." `tcp_output` actually tries
  // to send the data.
  if (error == ERR_OK) {
    error = tcp_output(connection->client_pcb);
    debug("in SendAwaiter constructor. tcp_output returned %s\n",
          lwip_describe(error));
  }
}

inline bool SendAwaiter::await_ready() {
  debug("in SendAwaiter::await_ready() returning %d\n", error);
  return error;
}

inline void SendAwaiter::await_suspend(std::coroutine_handle<> continuation) {
  debug("in SendAwaiter::await_suspend()\n");
  this->continuation = continuation;
  assert(connection);
  connection->senders.push(this);
}

inline std::tuple<int, err_t> SendAwaiter::await_resume() {
  debug("in SendAwaiter::await_resume() returning (%d, %s)\n",
        length - remaining, lwip_describe(error));
  return {length - remaining, error};
}

}  // namespace picoro
