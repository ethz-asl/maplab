#ifndef MESSAGE_FLOW_MESSAGE_FLOW_INL_H_
#define MESSAGE_FLOW_MESSAGE_FLOW_INL_H_

#include <string>
#include <unordered_map>

#include <glog/logging.h>

#include "message-flow/callback-types.h"
#include "message-flow/message-delivery-queue.h"
#include "message-flow/message-dispatcher.h"
#include "message-flow/publisher.h"
#include "message-flow/subscriber-network.h"

namespace message_flow {
template <typename MessageTopicDefinition>
PublisherFunction<MessageTopicDefinition> MessageFlow::registerPublisher() {
  typedef typename MessageTopicDefinition::message_type MessageType;
  std::shared_ptr<Publisher<MessageType>> publisher_to_queue;
  {
    std::lock_guard<std::mutex> lock(mutex_network_and_maps_);
    SubscriberListPtr<MessageType> subscriber_list =
        subscriber_network_
            .getSubscriberListAndAllocateIfNecessary<MessageTopicDefinition>();

    // Create a publisher that insert the message into to the delivery queues
    // of all topic subscribers.
    publisher_to_queue =
        std::make_shared<Publisher<MessageType>>(subscriber_list);
  }

  // Wrap the publisher function such that the publisher is kept alive.
  std::function<void(const MessageType&)> publisher_fct = std::bind(
      &Publisher<MessageType>::publish, publisher_to_queue,
      std::placeholders::_1);
  return publisher_fct;
}

template <typename MessageTopicDefinition>
void MessageFlow::registerSubscriber(
    const std::string& subscriber_node_name,
    const DeliveryOptions& delivery_options,
    const SubscriberCallback<MessageTopicDefinition>& callback) {
  CHECK(!subscriber_node_name.empty());
  CHECK(callback);
  {
    std::lock_guard<std::mutex> lock(mutex_network_and_maps_);

    // Allocate new subscriber queue for the node if necessary. The callback
    // defines the function to which all messages of this queue are delivered
    // to.
    MessageDeliveryQueueId queue_id;
    common::generateId(&queue_id);

    MessageDeliveryQueueBasePtr& node_queue =
        subscriber_message_queues_[queue_id];
    CHECK(node_queue == nullptr) << "Subscriber with id" << queue_id
                                 << " already registered.";

    typedef MessageDeliveryQueue<MessageTopicDefinition> MessageQueueDerived;
    node_queue.reset(new MessageQueueDerived(callback, delivery_options));
    CHECK(
        subscriber_node_names_.emplace(queue_id, subscriber_node_name).second);

    // Also add the new subscriber to the pub/sub network definitions. The
    // callback defines the logic executed when a message is published. In this
    // case we add it to the delivery queue and the dispatcher sends the message
    // according to its policy later.
    const auto add_message_to_queue_fct = [&node_queue, this](
        const typename MessageTopicDefinition::message_type& message) -> void {
      std::static_pointer_cast<MessageQueueDerived>(node_queue)
          ->queueMessageForDelivery(message);
      // Signal the dispatcher that a new message has been put into the queues.
      this->message_dispatcher_->newMessageInQueue(node_queue);
    };

    subscriber_network_.addSubscriber<MessageTopicDefinition>(
        add_message_to_queue_fct);
  }
}
}  // namespace message_flow
#endif  // MESSAGE_FLOW_MESSAGE_FLOW_INL_H_
