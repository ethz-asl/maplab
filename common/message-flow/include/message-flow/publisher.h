#ifndef MESSAGE_FLOW_PUBLISHER_H_
#define MESSAGE_FLOW_PUBLISHER_H_

#include <memory>
#include <mutex>

#include "message-flow/subscriber-list.h"

namespace message_flow {

template <typename MessageType>
class Publisher {
 public:
  explicit Publisher(
      const std::weak_ptr<SubscriberList<MessageType>>& topic_subscribers)
      : topic_subscribers_(topic_subscribers) {}

  // Publish the message to all subscribers.
  void publish(const MessageType& message) {
    SubscriberListPtr<MessageType> topic_subscribers =
        topic_subscribers_.lock();
    if (topic_subscribers) {
      topic_subscribers->publishToAllSubscribersBlocking(message);
    }
  }

 private:
  std::weak_ptr<SubscriberList<MessageType>> topic_subscribers_;
};
}  // namespace message_flow
#endif  // MESSAGE_FLOW_PUBLISHER_H_
