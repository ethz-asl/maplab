#ifndef MAPLAB_COMMON_ORDERED_MONITORS_H_
#define MAPLAB_COMMON_ORDERED_MONITORS_H_

#include <memory>
#include <tuple>

#include <glog/logging.h>

#include "maplab-common/monitor.h"

namespace common {

// This is a class to enforce lock ordering to avoid deadlocks when dealing
// with multiple lockable resources, see
// https://en.wikipedia.org/wiki/Deadlock#Avoiding_database_deadlocks
// See unit test for usage.
template <typename... MonitoredTypes>
class OrderedMonitors {
 public:
  typedef std::tuple<Monitor<MonitoredTypes>...> MonitorTuple;

  class OrderedAccess {
   public:
    typedef std::tuple<
        std::unique_ptr<typename Monitor<MonitoredTypes>::WriteAccess>...>
        AccessPtrTuple;
    typedef std::tuple<typename Monitor<MonitoredTypes>::WriteAccess&...>
        AccessTuple;

    explicit OrderedAccess(OrderedMonitors* monitors)
        : monitors_(CHECK_NOTNULL(monitors)), latest_access_(-1) {}

    template <int I>
    const typename std::tuple_element<I, AccessTuple>::type& get() {
      if (I <= latest_access_) {
        CHECK(std::get<I>(accesses_)) << "Tried to lock monitor " << I
                                      << " after " << latest_access_;
      } else {
        std::get<I>(accesses_).reset(
            std::get<I>(monitors_->monitors_).allocatedWriteAccess());
        latest_access_ = I;
      }
      return *std::get<I>(accesses_);
    }

   private:
    OrderedMonitors* monitors_;

    AccessPtrTuple accesses_;
    int latest_access_;
  };

  OrderedMonitors(MonitoredTypes... objects)  // NOLINT
      : monitors_(std::make_tuple(Monitor<MonitoredTypes>(objects)...)) {}

  OrderedAccess orderedAccess() {
    return OrderedAccess(this);
  }

 private:
  MonitorTuple monitors_;
};

}  // namespace common

#endif  // MAPLAB_COMMON_ORDERED_MONITORS_H_
