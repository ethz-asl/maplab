#ifndef MESSAGE_FLOW_MESSAGE_DELIVERY_QUEUE_H_
#define MESSAGE_FLOW_MESSAGE_DELIVERY_QUEUE_H_

#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <glog/logging.h>
#include <maplab-common/unique-id.h>

namespace message_flow {
UNIQUE_ID_DEFINE_ID(MessageDeliveryQueueId);

struct DeliveryOptions {
  DeliveryOptions() : exclusivity_group_id(-1) {}
  // Ensures the exclusive execution of deliveries across all subscribers with
  // the same group id. With a FIFO message dispatcher, this will expand the
  // delivery order guarantees across multiple subscribers. I.e. not only all
  // messages on individual subscribers will be delivered in publishing order,
  // but also all messages to subscribers with the same exclusivity id will be
  // delivered in the publishing order.
  // A negative value means no exclusivity is enforced.
  int exclusivity_group_id;
};

class MessageDeliveryQueueBase {
 public:
  virtual ~MessageDeliveryQueueBase() {}
  virtual void deliverOldestMessage() = 0;
  virtual std::string getTopicName() const = 0;
  virtual const DeliveryOptions& getDeliveryOptions() const = 0;
  virtual size_t size() const = 0;
};
typedef std::shared_ptr<MessageDeliveryQueueBase> MessageDeliveryQueueBasePtr;

// Maintains a list of messages scheduled for delivery to a specific subscriber.
template <typename MessageTopicDefinition>
class MessageDeliveryQueue : public MessageDeliveryQueueBase {
 public:
  typedef typename MessageTopicDefinition::message_type MessageType;
  typedef std::function<void(const MessageType&)> SubscriberCallback;

  MessageDeliveryQueue(
      const SubscriberCallback& subscriber_callback,
      const DeliveryOptions& delivery_options)
      : delivery_options_(delivery_options),
        subscriber_callback_(subscriber_callback) {
    CHECK(subscriber_callback);
  }
  virtual ~MessageDeliveryQueue() {}

  void queueMessageForDelivery(const MessageType& message) {
    std::lock_guard<std::mutex> lock(m_message_queue_);
    message_queue_.emplace_back(message);
  }

  void deliverOldestMessage() final {
    MessageType message;
    {
      std::lock_guard<std::mutex> lock(m_message_queue_);
      CHECK(!message_queue_.empty());
      message = message_queue_.front();
      message_queue_.pop_front();
    }

    // Run the subscriber callback; the lock ensures only one callback can be
    // run simultaneously.
    std::lock_guard<std::mutex> lock_subscriber(m_subscriber_execution_);
    subscriber_callback_(message);
  }

  std::string getTopicName() const final {
    return MessageTopicDefinition::kMessageTopic;
  }

  const DeliveryOptions& getDeliveryOptions() const final {
    return delivery_options_;
  }

  bool empty() const {
    std::lock_guard<std::mutex> lock(m_message_queue_);
    return message_queue_.empty();
  }

  virtual size_t size() const {
    std::lock_guard<std::mutex> lock(m_message_queue_);
    return message_queue_.size();
  }

 private:
  const DeliveryOptions delivery_options_;

  // Protects the callback to prevent concurrent calls to the subscriber
  // callback.
  std::mutex m_subscriber_execution_;
  const SubscriberCallback subscriber_callback_;

  mutable std::mutex m_message_queue_;
  std::deque<MessageType> message_queue_;
};
}  // namespace message_flow
UNIQUE_ID_DEFINE_ID_HASH(message_flow::MessageDeliveryQueueId);
#endif  // MESSAGE_FLOW_MESSAGE_DELIVERY_QUEUE_H_
