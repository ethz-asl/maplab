#ifndef MESSAGE_FLOW_MESSAGE_DISPATCHER_FIFO_H_
#define MESSAGE_FLOW_MESSAGE_DISPATCHER_FIFO_H_

#include <deque>

#include <aslam/common/thread-pool.h>
#include <glog/logging.h>

#include "message-flow/message-delivery-queue.h"
#include "message-flow/message-dispatcher.h"

namespace message_flow {
// A thread pool delivers the published messages in incoming order.
class MessageDispatcherFifo : public MessageDispatcher {
 public:
  explicit MessageDispatcherFifo(size_t num_threads)
      : thread_pool_(num_threads) {}

  virtual ~MessageDispatcherFifo() {
    shutdown();
  }

  virtual void newMessageInQueue(const MessageDeliveryQueueBasePtr& queue) {
    CHECK(queue);
    const DeliveryOptions& delivery_options = queue->getDeliveryOptions();
    size_t exclusivity_group_id;
    if (delivery_options.exclusivity_group_id < 0) {
      // If no external exclusivity is specified, we derive it from the queue.
      // This means that the messages for each subscriber are delivered in the
      // same order as published i.e. only a single thread can work on a queue
      // at the same time.
      // Theoretically, we could collide with manually specified IDs, the
      // chances are pretty low though.
      exclusivity_group_id = reinterpret_cast<size_t>(queue.get());
    } else {
      exclusivity_group_id = delivery_options.exclusivity_group_id;
    }
    thread_pool_.enqueueOrdered(
        exclusivity_group_id,
        std::bind(
            &MessageDeliveryQueueBase::deliverOldestMessage, queue.get()));
  }

  virtual void shutdown() {
    thread_pool_.stop();
  }

  virtual void waitUntilIdle() const {
    thread_pool_.waitForEmptyQueue();
  }

 private:
  aslam::ThreadPool thread_pool_;
};
}  // namespace message_flow
#endif  // MESSAGE_FLOW_MESSAGE_DISPATCHER_FIFO_H_
