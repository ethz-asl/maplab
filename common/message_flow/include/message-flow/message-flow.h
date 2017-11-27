#ifndef MESSAGE_FLOW_MESSAGE_FLOW_H_
#define MESSAGE_FLOW_MESSAGE_FLOW_H_

#include <mutex>
#include <string>
#include <unordered_map>

#include "message-flow/callback-types.h"
#include "message-flow/message-delivery-queue.h"
#include "message-flow/message-dispatcher.h"
#include "message-flow/subscriber-network.h"

namespace message_flow {
class MessageFlow {
 public:
  // Caller takes ownership.
  template <typename MessageDispatcherType>
  static MessageFlow* create(size_t num_threads) {
    return new MessageFlow(
        std::make_shared<MessageDispatcherType>(num_threads));
  }
  ~MessageFlow();

  // Register a new publisher for the topic declared in MessageTopicDefinition.
  // The return function handle can be used to publish a message on this topic.
  template <typename MessageTopicDefinition>
  PublisherFunction<MessageTopicDefinition> registerPublisher();

  // The node name is just used to print human-readable queue statistics. It
  // has no meaning as an identifier internally.
  template <typename MessageTopicDefinition>
  void registerSubscriber(
      const std::string& subscriber_node_name,
      const DeliveryOptions& delivery_options,
      const SubscriberCallback<MessageTopicDefinition>& callback);

  // Usually, a shutdown should consist of a call to shutdown(), followed by a
  // call to waitUntilIdle(). This ensures all queues are shutdown and all
  // remaining messages in the pipeline are still delivered.
  void shutdown();
  void waitUntilIdle() const;

  std::string printDeliveryQueueStatistics() const;

 protected:
  explicit MessageFlow(const MessageDispatcherPtr& dispatcher);

 private:
  // Dispatches the subscriber callbacks according to its policy.
  MessageDispatcherPtr message_dispatcher_;

  // Protects the maps and the subscriber network.
  mutable std::mutex mutex_network_and_maps_;
  // One queue per subscriber that maintains the ordered list of undelivered
  // messages.
  typedef std::unordered_map<MessageDeliveryQueueId,
                             MessageDeliveryQueueBasePtr>
      MessageDeliveryQueueMap;
  MessageDeliveryQueueMap subscriber_message_queues_;
  std::unordered_map<MessageDeliveryQueueId, std::string>
      subscriber_node_names_;
  // Maintains the subscriber network.
  SubscriberNetwork subscriber_network_;
};
}  // namespace message_flow
#include "message-flow/message-flow-inl.h"
#endif  // MESSAGE_FLOW_MESSAGE_FLOW_H_
