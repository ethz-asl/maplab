#ifndef MAPLAB_COMMON_DATA_SYNCHRONIZER_H_
#define MAPLAB_COMMON_DATA_SYNCHRONIZER_H_

#include <functional>
#include <list>
#include <map>
#include <mutex>
#include <iostream>
#include <vector>

#include <glog/logging.h>

#include "maplab-common/accessors.h"

namespace common {

template<typename DataType>
struct DefaultTimestampExtractor {
  int64_t operator()(const DataType& data) {
    return data.timestamp_ns;
  }
};

// Accumulates data from two streams and invokes a callback for each matching
// timestamp. It is assumed that the data is added in temporal order. Data is
// dropped if no temporally corresponding pair is found for A and B.
template<typename DataTypeA, typename DataTypeB,
         typename TimestampExtractorA = DefaultTimestampExtractor<DataTypeA>,
         typename TimestampExtractorB = DefaultTimestampExtractor<DataTypeB>>
class DataSynchronizer {
 public:
  typedef std::function<void(const DataTypeA&, const DataTypeB&)>
      SynchronizedDataCallback;

  DataSynchronizer()
      : last_timestamp_a_(std::numeric_limits<int64_t>::min()),
        last_timestamp_b_(std::numeric_limits<int64_t>::min()) {}

  void processTypeA(const DataTypeA& data) {
    {
      std::lock_guard<std::mutex> lock(m_list_a_);
      const int64_t timestamp_a = TimestampExtractorA()(data);
      CHECK_GT(timestamp_a, last_timestamp_a_) << "Data not in temporal order.";
      list_a_.emplace_back(data);
      last_timestamp_a_ = timestamp_a;
    }
    findAndProcessMatches();
  }
  void processTypeB(const DataTypeB& data) {
    {
      std::lock_guard<std::mutex> lock(m_map_b_);
      const int64_t timetamp_b = TimestampExtractorB()(data);
      CHECK_GT(timetamp_b, last_timestamp_b_) << "Data not in temporal order.";
      CHECK(map_b_.emplace(timetamp_b, data).second);
      last_timestamp_b_ = timetamp_b;
    }
    findAndProcessMatches();
  }

  void clear() {
    std::lock_guard<std::mutex> lock_a(m_list_a_);
    std::lock_guard<std::mutex> lock_b(m_map_b_);
    list_a_.clear();
    map_b_.clear();
  }

  // Unit test interface.
  std::pair<size_t, size_t> getContainerSizes() const {
    std::lock_guard<std::mutex> lock_a(m_list_a_);
    std::lock_guard<std::mutex> lock_b(m_map_b_);
    return {list_a_.size(), map_b_.size()};
  }

  void registerCallback(const SynchronizedDataCallback& cb) {
    CHECK(cb);
    std::lock_guard<std::mutex> lock(m_sync_data_callback_);
    sync_data_callback_ = cb;
  }

 private:
  void findAndProcessMatches() {
    std::lock_guard<std::mutex> lock_a(m_list_a_);
    std::lock_guard<std::mutex> lock_b(m_map_b_);

    if (list_a_.empty() || map_b_.empty()) {
      return;
    }

    // Remove all elements in map B up to (but no including) the lowest
    // timestamp in A; the will never match assuming temporal order of input.
    const int64_t lowest_timestamp_a = TimestampExtractorA()(list_a_.front());
    typename std::map<int64_t, DataTypeB>::iterator it_b =
        map_b_.lower_bound(lowest_timestamp_a);
    map_b_.erase(map_b_.begin(), it_b);


    // Go over all elements of the list A and try to find matching elements in
    // map B. We also remove all entries that won't match in the future
    // (assuming input is in temporal order).
    //  list_a:  front -> oldest/smallest - back   -> newest/largest timestamp
    //  map_a:   begin -> oldest/smallest - rbegin -> newest/largest timestamp
    typename std::list<DataTypeA>::iterator it_a = list_a_.begin();
    while (it_a != list_a_.end()) {
      const int64_t timestamp_a = TimestampExtractorA()(*it_a);

      // Early exit if no matches are possible.
      if (map_b_.empty()) {
        break;
      }

      // Nothing to match if current timestamp in A is ahead of the most
      // recent element in map B.
      const int64_t largest_timestamp_b = map_b_.rbegin()->first;
      if (timestamp_a > largest_timestamp_b) {
        break;
      }

      // We can remove all entries with timestamps lower than the lowest
      // timestamp in map_b as they will never match.
      const int64_t lowest_timestamp_b = map_b_.begin()->first;
      if (timestamp_a < lowest_timestamp_b) {
        it_a = list_a_.erase(it_a);
        continue;
      }

      // Try to find a matching element in map B.
      typename std::map<int64_t, DataTypeB>::iterator it_b =
          map_b_.find(timestamp_a);
      if (it_b != map_b_.end()) {
        // Process the match and remove the element from list A. We will remove
        // all elements from map B at the end.
        CHECK_EQ(TimestampExtractorA()(*it_a),
                 TimestampExtractorB()(it_b->second));

        {
          std::lock_guard<std::mutex> lock(m_sync_data_callback_);
          if (sync_data_callback_) {
            sync_data_callback_(*it_a, it_b->second);
          }
        }

        map_b_.erase(map_b_.begin(), ++it_b);
        it_a = list_a_.erase(it_a);
        continue;
      } else {
        // We can delete the oldest entry in A if there are newer entries in
        // map B.
        if (timestamp_a < map_b_.rbegin()->first) {
          it_a = list_a_.erase(it_a);
          continue;
        }
      }

      ++it_a;
    }
  }

  int64_t last_timestamp_a_;
  std::list<DataTypeA> list_a_;
  mutable std::mutex m_list_a_;

  int64_t last_timestamp_b_;
  std::map<int64_t, DataTypeB> map_b_;
  mutable std::mutex m_map_b_;

  SynchronizedDataCallback sync_data_callback_;
  std::mutex m_sync_data_callback_;
};

}  // namespace common

#endif  // MAPLAB_COMMON_DATA_SYNCHRONIZER_H_
