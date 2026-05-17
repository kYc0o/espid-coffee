// ESPID-Coffee — Result<T>/Status error-handling primitives.
//
// Lightweight discriminated-union used throughout the project in place of
// exceptions (which are disabled). All driver methods return either:
//   - Result<T>: a value-or-error
//   - Status   : a void-or-error
//
// Both types are [[nodiscard]] so callers cannot silently drop failures.

#pragma once

#include <cstdint>
#include <new>
#include <type_traits>
#include <utility>

namespace espid::hal_cpp {

enum class Error : std::uint8_t {
    kOk = 0,
    kInvalidArg,
    kNotInitialised,
    kAlreadyInit,
    kBusError,
    kTimeout,
    kCrcMismatch,
    kRangeError,
    kHardwareFault,
    kUnsupported,
};

constexpr const char* to_string(Error e) noexcept {
    switch (e) {
        case Error::kOk:             return "ok";
        case Error::kInvalidArg:     return "invalid-arg";
        case Error::kNotInitialised: return "not-initialised";
        case Error::kAlreadyInit:    return "already-init";
        case Error::kBusError:       return "bus-error";
        case Error::kTimeout:        return "timeout";
        case Error::kCrcMismatch:    return "crc-mismatch";
        case Error::kRangeError:     return "range-error";
        case Error::kHardwareFault:  return "hardware-fault";
        case Error::kUnsupported:    return "unsupported";
    }
    return "unknown";
}

// Result<T> — value-or-error. Union storage handles non-trivial T.
template <typename T>
class [[nodiscard]] Result {
   public:
    // Implicit conversions are intentional: `return value;` / `return Error::kFoo;`.
    Result(T v) noexcept(std::is_nothrow_move_constructible_v<T>) : has_value_(true) {
        new (&storage_.value) T(std::move(v));
    }
    Result(Error e) noexcept : has_value_(false), error_(e) {}

    Result(const Result& other) noexcept(std::is_nothrow_copy_constructible_v<T>)
        : has_value_(other.has_value_) {
        if (has_value_) {
            new (&storage_.value) T(other.storage_.value);
        } else {
            error_ = other.error_;
        }
    }
    Result(Result&& other) noexcept(std::is_nothrow_move_constructible_v<T>)
        : has_value_(other.has_value_) {
        if (has_value_) {
            new (&storage_.value) T(std::move(other.storage_.value));
        } else {
            error_ = other.error_;
        }
    }

    Result& operator=(const Result&) = delete;
    Result& operator=(Result&&) = delete;

    ~Result() {
        if (has_value_) {
            storage_.value.~T();
        }
    }

    [[nodiscard]] bool ok() const noexcept { return has_value_; }
    explicit operator bool() const noexcept { return has_value_; }

    // Precondition: ok().
    const T& value() const& noexcept { return storage_.value; }
    T&       value() & noexcept       { return storage_.value; }
    T        value() && noexcept      { return std::move(storage_.value); }

    T value_or(T fallback) const noexcept {
        return has_value_ ? storage_.value : std::move(fallback);
    }

    // Precondition: !ok().
    Error error() const noexcept { return error_; }

   private:
    union Storage {
        Storage() {}
        ~Storage() {}
        T value;
    } storage_{};
    bool  has_value_;
    Error error_{Error::kOk};
};

// Status — void-or-error.
class [[nodiscard]] Status {
   public:
    Status() noexcept = default;             // defaults to kOk
    Status(Error e) noexcept : error_(e) {}  // implicit on purpose

    [[nodiscard]] bool ok() const noexcept { return error_ == Error::kOk; }
    explicit operator bool() const noexcept { return ok(); }
    Error error() const noexcept { return error_; }

   private:
    Error error_;
};

}  // namespace espid::hal_cpp
