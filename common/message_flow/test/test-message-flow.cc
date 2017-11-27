#include <cstdlib>
#include <future>

#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/threadsafe-queue.h>

#include "message-flow/message-dispatcher-fifo.h"
#include "message-flow/message-flow.h"
#include "message-flow/message-topic-registration.h"

MESSAGE_FLOW_TOPIC(TopicA, double);
MESSAGE_FLOW_TOPIC(TopicB, double);
MESSAGE_FLOW_TOPIC(TopicX, double);

namespace message_flow {
const std::string kSubscriberNode("SubNode");

TEST(MessageFlow, PublishSubscribeE2E) {
  class AdderNode {
   public:
    void AttachToMessageFlow(MessageFlow* flow) {
      CHECK_NOTNULL(flow);
      flow->registerSubscriber<message_flow_topics::TopicA>(
          kSubscriberNode, DeliveryOptions(),
          [this](double a) { this->a_.set_value(a); });
      flow->registerSubscriber<message_flow_topics::TopicB>(
          kSubscriberNode, DeliveryOptions(),
          [this](double b) { this->b_.set_value(b); });
      publish_x_ = flow->registerPublisher<message_flow_topics::TopicX>();
    }

    void Launch() {
      std::thread(std::bind(&AdderNode::AdderTask, this)).detach();
    }

   private:
    void AdderTask() {
      std::future<double> f_a = a_.get_future();
      std::future<double> f_b = b_.get_future();
      f_a.wait();
      f_b.wait();

      CHECK(f_a.valid());
      CHECK(f_b.valid());
      publish_x_(f_a.get() + f_b.get());
    }

    std::promise<double> a_;
    std::promise<double> b_;
    std::function<void(const double&)> publish_x_;  // NOLINT
  };

  // Create a network with the following nodes:
  //  - Publish a number on TopicA and TopicB
  //  - Subscribe to TopicA/TopicB and publish TopicA+TopicB on TopicX
  //  - Subscribe to TopicX
  std::unique_ptr<MessageFlow> flow(
      MessageFlow::create<MessageDispatcherFifo>(
          std::thread::hardware_concurrency()));

  // Add an adder node to the flow network that adds the two number published
  // on TopicA and TopicB and publishes the result on TopicC.
  AdderNode adder_node;
  adder_node.AttachToMessageFlow(flow.get());
  adder_node.Launch();

  // Catch the result published on TopicC.
  std::promise<double> result_on_topic_x;
  flow->registerSubscriber<message_flow_topics::TopicX>(
      kSubscriberNode, DeliveryOptions(),
      [&result_on_topic_x](double x) { result_on_topic_x.set_value(x); });

  // Now publish two numbers on TopicA and TopicB.
  std::function<void(const double&)> publisher_topic_a =  // NOLINT
      flow->registerPublisher<message_flow_topics::TopicA>();
  std::function<void(const double&)> publisher_topic_b =  // NOLINT
      flow->registerPublisher<message_flow_topics::TopicB>();

  constexpr double kNumberA = 10.0;
  publisher_topic_a(kNumberA);
  constexpr double kNumberB = 20.0;
  publisher_topic_b(kNumberB);

  std::future<double> f_result_on_topic_c = result_on_topic_x.get_future();
  CHECK(f_result_on_topic_c.valid());
  EXPECT_EQ(f_result_on_topic_c.get(), kNumberA + kNumberB);
  flow->shutdown();
  flow->waitUntilIdle();
}

TEST(MessageFlow, MessageDispatcherThreadedFifo_MessageDeliveryOrder) {
  // Create a network with the following nodes:
  //  - Publish numbers on TopicA
  //  - Subscribe to TopicA and ensure the publishing order corresponds to the
  //    receive order.
  constexpr size_t kNumThreads = 32u;
  std::unique_ptr<MessageFlow> flow(
      MessageFlow::create<MessageDispatcherFifo>(kNumThreads));

  std::function<void(const double&)> publish_on_topic_a =
      flow->registerPublisher<message_flow_topics::TopicA>();
  common::ThreadSafeQueue<double> receive_queue;

  flow->registerSubscriber<message_flow_topics::TopicA>(
      kSubscriberNode, DeliveryOptions(), [&receive_queue](double value) {
        // Let's wait a small random amount of time to check if the threads
        // respect the incoming order of the messages when being delivered
        // by multiple threads.
        std::this_thread::sleep_for(
            std::chrono::microseconds(rand() % 10));  // NOLINT
        receive_queue.Push(value);
      });

  // Publish a lot of number and wait until all message are delivered.
  size_t kNumNumbers = 1000u;
  for (size_t number = 0; number < kNumNumbers; ++number) {
    publish_on_topic_a(number);
  }
  flow->waitUntilIdle();

  // Check order of the result queue.
  ASSERT_EQ(receive_queue.Size(), kNumNumbers);

  size_t counter = 0u;
  double value;
  while (receive_queue.PopNonBlocking(&value)) {
    EXPECT_EQ(static_cast<double>(counter), value);
    ++counter;
  }
  EXPECT_EQ(counter, kNumNumbers);

  LOG(INFO) << flow->printDeliveryQueueStatistics();
  flow->shutdown();
  flow->waitUntilIdle();
}

TEST(
    MessageFlow,
    MessageDispatcherThreadedFifo_MessageDeliveryOrderExclusivity) {
  // Create a network with the following nodes:
  //  - Publish numbers on TopicA and TopicB.
  //  - Subscribe to TopicA and TopicB; both subscribers are in the
  //    same exclusivity group. The callback execution order should correspond
  //    to the publishing order; independent of the callback runtime.
  constexpr size_t kNumThreads = 32u;
  std::unique_ptr<MessageFlow> flow(
      MessageFlow::create<MessageDispatcherFifo>(kNumThreads));

  std::function<void(const double&)> publish_on_topic_a =
      flow->registerPublisher<message_flow_topics::TopicA>();
  std::function<void(const double&)> publish_on_topic_b =
      flow->registerPublisher<message_flow_topics::TopicB>();
  common::ThreadSafeQueue<double> receive_queue;

  const auto receive_callback = [&receive_queue](double value) {
    // Let's wait a small random amount of time to check if the threads
    // respect the incoming order of the messages when being delivered
    // by multiple threads.
    std::this_thread::sleep_for(
        std::chrono::microseconds(rand() % 10));  // NOLINT
    receive_queue.Push(value);
  };

  DeliveryOptions delivery_options;
  delivery_options.exclusivity_group_id = 0;
  flow->registerSubscriber<message_flow_topics::TopicA>(
      kSubscriberNode, delivery_options, receive_callback);
  flow->registerSubscriber<message_flow_topics::TopicB>(
      kSubscriberNode, delivery_options, receive_callback);

  // Publish a lot of number on either topic A or B, which we select randomly.
  size_t kNumNumbers = 1000u;
  for (size_t number = 0; number < kNumNumbers; ++number) {
    if (rand() % 2 == 0) {  // NOLINT
      publish_on_topic_a(number);
    } else {
      publish_on_topic_b(number);
    }
  }
  flow->waitUntilIdle();

  // Check order of the result queue.
  ASSERT_EQ(receive_queue.Size(), kNumNumbers);

  size_t counter = 0u;
  double value;
  while (receive_queue.PopNonBlocking(&value)) {
    EXPECT_EQ(static_cast<double>(counter), value);
    ++counter;
  }
  EXPECT_EQ(counter, kNumNumbers);

  LOG(INFO) << flow->printDeliveryQueueStatistics();
  flow->shutdown();
  flow->waitUntilIdle();
}
}  // namespace message_flow
MAPLAB_UNITTEST_ENTRYPOINT
