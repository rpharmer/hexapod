#pragma once

#include <type_traits>
#include <utility>

namespace autonomy::modules {

// Stores shell state snapshots (last_*_) and returns the same value.
template <typename T, typename U>
[[nodiscard]] T storeLast(T& slot, U&& value) {
    static_assert(std::is_assignable_v<T&, U&&>, "storeLast requires assignable slot/value types");
    slot = std::forward<U>(value);
    return slot;
}

} // namespace autonomy::modules
