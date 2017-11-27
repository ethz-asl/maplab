#ifndef MESSAGE_FLOW_SUBSCRIBER_LIST_H_
#define MESSAGE_FLOW_SUBSCRIBER_LIST_H_

#include <mutex>
#include <thread>
#include <vector>

#include <glog/logging.h>

namespace message_flow {
// An abstract interface to an ordered subscriber list.
class SubscriberListBase {
 public:
  SubscriberListBase() {}
  virtual ~SubscriberListBase() {}
  virtual void clear() = 0;
};
typedef std::shared_ptr<SubscriberListBase> SubscriberListBasePtr;

// Implementation of a subscriber list with an ordering equal to the
// registration order.
template <typename MessageType>
class SubscriberList : public SubscriberListBase {
 public:
  typedef std::function<void(const MessageType&)> SubscriberCallback;

  SubscriberList() {}

  virtual ~SubscriberList() {
    // De-register all subscribers on shutdown. All publish requests are thus
    // rejected and the remaining messages are processed.
    clear();
  }

  virtual void clear() {
    std::lock_guard<std::mutex> lock(m_subscriber_list_);
    subscriber_list_.clear();
  }

  void addSubscriber(const SubscriberCallback& subscriber) {
    CHECK(subscriber);
    std::lock_guard<std::mutex> lock(m_subscriber_list_);
    subscriber_list_.emplace_back(subscriber);
  }

  void publishToAllSubscribersBlocking(const MessageType& message) const {
    std::lock_guard<std::mutex> lock(m_subscriber_list_);
    for (const SubscriberCallback& subscriber_callback : subscriber_list_) {
      CHECK(subscriber_callback);
      subscriber_callback(message);
    }
  }

 private:
  mutable std::mutex m_subscriber_list_;
  std::vector<SubscriberCallback> subscriber_list_;
};

template <typename MessageType>
using SubscriberListPtr = std::shared_ptr<SubscriberList<MessageType>>;
}  // namespace message_flow
#endif  // MESSAGE_FLOW_SUBSCRIBER_LIST_H_
