#ifndef MESSAGE_FLOW_CALLBACK_TYPES_H_
#define MESSAGE_FLOW_CALLBACK_TYPES_H_

#include <functional>

namespace message_flow {
template <typename MessageTopicDefinition>
using MessageCallback =
    std::function<void(const typename MessageTopicDefinition::message_type&)>;
template <typename MessageTopicDefinition>
using PublisherFunction = MessageCallback<MessageTopicDefinition>;
template <typename MessageTopicDefinition>
using SubscriberCallback = MessageCallback<MessageTopicDefinition>;
}  // namespace message_flow
#endif  // MESSAGE_FLOW_CALLBACK_TYPES_H_
