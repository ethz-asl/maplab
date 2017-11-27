#include "message-flow/message-flow.h"

#include <iomanip>
#include <mutex>
#include <sstream>
#include <string>

#include <glog/logging.h>
#include <maplab-common/accessors.h>

#include "message-flow/message-dispatcher.h"
#include "message-flow/subscriber-network.h"

namespace message_flow {
MessageFlow::MessageFlow(const MessageDispatcherPtr& dispatcher)
    : message_dispatcher_(dispatcher) {
  CHECK(dispatcher);
}

MessageFlow::~MessageFlow() {
  shutdown();
}

void MessageFlow::shutdown() {
  // First clear all subscribers such that published messages get rejected
  // from now on. Then signal the dispatcher to shutdown. An application could
  // then call to WaitUntilIdle() for a clean shutdown where all remaining
  // tasks can execute until the end.
  std::lock_guard<std::mutex> lock(mutex_network_and_maps_);
  subscriber_network_.unregisterAllSubscribers();
  message_dispatcher_->shutdown();
}

void MessageFlow::waitUntilIdle() const {
  message_dispatcher_->waitUntilIdle();
}

std::string MessageFlow::printDeliveryQueueStatistics() const {
  std::lock_guard<std::mutex> lock(mutex_network_and_maps_);

  std::stringstream output;
  constexpr size_t kNumAlignment = 30u;
  output << "Message delivery queues:" << std::endl;
  output << std::setiosflags(std::ios::left) << std::setw(kNumAlignment)
         << "subscriber-node" << std::setw(kNumAlignment) << "queue-topic"
         << std::setw(kNumAlignment) << "queue-id" << std::setw(kNumAlignment)
         << "num elements" << std::endl;

  for (const MessageDeliveryQueueMap::value_type& value :
       subscriber_message_queues_) {
    const MessageDeliveryQueueId& queue_id = value.first;
    const MessageDeliveryQueueBasePtr& queue = value.second;
    const std::string& subscriber_node_name =
        common::getChecked(subscriber_node_names_, queue_id);
    CHECK(queue);
    output << std::setiosflags(std::ios::left) << std::setw(kNumAlignment)
           << subscriber_node_name << std::setw(kNumAlignment)
           << queue->getTopicName() << std::setw(kNumAlignment) << queue_id
           << std::setw(kNumAlignment) << queue->size() << std::endl;
  }
  return output.str();
}
}  // namespace message_flow
