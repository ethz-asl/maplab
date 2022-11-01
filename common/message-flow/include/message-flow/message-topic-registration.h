#ifndef MESSAGE_FLOW_MESSAGE_TOPIC_REGISTRATION_H_
#define MESSAGE_FLOW_MESSAGE_TOPIC_REGISTRATION_H_

#define MESSAGE_FLOW_TOPIC(NAME, MESSAGE_TYPE)          \
  namespace message_flow_topics {                       \
  struct NAME {                                         \
    static constexpr const char* kMessageTopic = #NAME; \
    typedef MESSAGE_TYPE message_type;                  \
  };                                                    \
  }
#endif  // MESSAGE_FLOW_MESSAGE_TOPIC_REGISTRATION_H_
