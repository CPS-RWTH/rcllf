// Copyright 2026 rcllf Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/// @file per_subscription.hpp
/// @brief State kept per live subscription, safe against address reuse.
///
/// State keyed by a subscription's address alone outlives the
/// subscription, so a later subscription the allocator places at the same
/// address inherits it (a loss total that continues the old count, a take
/// debt the new reader never owed). PerSubscription keeps, next to each
/// value, a weak reference to the subscription that created it. A lookup
/// whose weak reference names a different subscription (an expired one that
/// lived at this address) starts the value fresh. Not thread safe: callers
/// hold their own lock around every call.
///
/// The weak reference can be the last one to a destroyed
/// subscription's control block, and dropping it frees the block through the
/// block's virtual destroy function. That code and its vtable live in the
/// library that created the subscription (make_shared is instantiated there).
/// The component manager unloads a component's library (dlclose) right after
/// it destroys the component's nodes, so the binding's static maps (take
/// debt, message_lost totals) dropped those references at exit through
/// unmapped memory: SIGSEGV in ~PerSubscription after every brake_fusion
/// T2Approx run. So a slot's first use marks that library RTLD_NODELETE: it
/// stays mapped until the process ends, and every weak reference here can be
/// dropped at any time (prune, clear, static destruction). The component
/// manager's dlclose then only drops its reference; the library's own static
/// destructors run at exit like those of a linked library.

#ifndef PER_SUBSCRIPTION_HPP_
#define PER_SUBSCRIPTION_HPP_

#include <dlfcn.h>

#include <cstddef>
#include <cstring>
#include <iterator>
#include <map>
#include <memory>
#include <utility>

#include "rclcpp/subscription_base.hpp"

namespace ros2_lf_binding
{

template<class V>
class PerSubscription
{
public:
  /// The value of `sub`, default constructed the first time `sub` asks.
  /// References stay valid until clear(), or until `sub` is destroyed and a
  /// later lookup prunes it.
  V & at(const rclcpp::SubscriptionBase & sub)
  {
    Owner self = sub.weak_from_this();
    auto it = slots_.find(&sub);
    if (it != slots_.end() && same_owner(it->second.owner, self)) {
      return it->second.value;
    }
    prune_expired();
    keep_block_code_loaded(self);
    Slot & slot = slots_[&sub];
    slot.owner = std::move(self);
    slot.value = V{};
    return slot.value;
  }

  void clear() {slots_.clear();}

  size_t size() const {return slots_.size();}

private:
  using Owner = std::weak_ptr<const rclcpp::SubscriptionBase>;

  struct Slot
  {
    Owner owner;
    V value{};
  };

  /// True when both name the same control block (or both name none, a
  /// subscription not owned by a shared_ptr, which keeps the address key).
  static bool same_owner(
    const std::weak_ptr<const rclcpp::SubscriptionBase> & a,
    const std::weak_ptr<const rclcpp::SubscriptionBase> & b)
  {
    return !a.owner_before(b) && !b.owner_before(a);
  }

  /// Mark the library holding the virtual functions of `o`'s control
  /// block (the code that frees it) RTLD_NODELETE. Runs once per new slot.
  /// libstdc++ lays a weak_ptr out as {element pointer, control block
  /// pointer}, and a control block starts with its vtable pointer; that is
  /// read from the bytes of `o` (a copy of the object representation, no
  /// access through the private members). A library linked at startup or the
  /// program itself is never unloaded, and pinning it changes nothing.
  static void keep_block_code_loaded(const Owner & o)
  {
    static_assert(sizeof(Owner) == 2 * sizeof(void *), "weak_ptr is not {ptr, block}");
    const void * words[2] = {nullptr, nullptr};
    std::memcpy(words, &o, sizeof(words));
    const auto * block = static_cast<const void * const *>(words[1]);
    if (block == nullptr) {
      return;  // not owned by a shared_ptr: no block to free
    }
    Dl_info info{};
    if (dladdr(*block, &info) == 0 || info.dli_fname == nullptr || info.dli_fname[0] == '\0') {
      return;
    }
    // NOLOAD: only an already loaded library (the one the vtable lies in).
    // The handle is kept on purpose: the library must outlive this process's
    // last weak reference.
    (void)dlopen(info.dli_fname, RTLD_LAZY | RTLD_NOLOAD | RTLD_NODELETE);
  }

  /// Drop the values of destroyed subscriptions. Runs only when a new
  /// subscription asks, so lookups of known ones stay a single find.
  void prune_expired()
  {
    for (auto it = slots_.begin(); it != slots_.end(); ) {
      const auto & owner = it->second.owner;
      const bool had_owner =
        owner.owner_before(std::weak_ptr<const rclcpp::SubscriptionBase>{}) ||
        std::weak_ptr<const rclcpp::SubscriptionBase>{}.owner_before(owner);
      it = had_owner && owner.expired() ? slots_.erase(it) : std::next(it);
    }
  }

  std::map<const rclcpp::SubscriptionBase *, Slot> slots_;
};

}  // namespace ros2_lf_binding

#endif  // PER_SUBSCRIPTION_HPP_
