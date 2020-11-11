#ifndef DDS_ABSTRACT_SUBSCRIBER_H_
#define DDS_ABSTRACT_SUBSCRIBER_H_

#include <fastrtps/Domain.h>
#include <fastrtps/attributes/SubscriberAttributes.h>
#include <fastrtps/subscriber/SampleInfo.h>
#include <fastrtps/subscriber/Subscriber.h>
#include <fastrtps/subscriber/SubscriberListener.h>

#include "dds_participant.h"

// This is an abstract subscriber developed for SLAM node, others needn't to use
// it
// 为方便 SLAM
// 模块在vector中管理多个subscriber，开发了这个类，具体消息类型的声明使用模板函数
// Create<T M>() 生成，其他模块不需要使用它

namespace cmdrDDS {

class DdsAbstractSubscriber {
 public:
  DdsAbstractSubscriber() : mp_subscriber_(nullptr), mp_participant_(nullptr){};
  virtual ~DdsAbstractSubscriber() {
    if (mp_sublistener_ != nullptr)
      eprosima::fastrtps::Domain::removeSubscriber(mp_subscriber_);
  };
  /**
   * Implementation of subscriber creation.
   * @param DdsParticipant &participant_ref  Reference of participant.
   * @param const std::string &topicname     topicname, string.
   * @param uint32_t history                 Depth of message.
   * @param function &callback_ptr           Callback function potiner.
   * @return True if success.
   * */
  template <typename T, typename M>
  bool Create(DdsParticipant &participant_ref, const std::string &topicname,
              uint32_t history, const std::function<void(T *)> &callback_ptr) {
    if (mp_subscriber_ != nullptr) {
      std::cout << "Subscriber already created.\n";
      return false;
    }
    mp_sublistener_ = std::unique_ptr<SubListener<T, M>>(new SubListener<T, M>);
    SubListener<T, M> *p_subListener =
        static_cast<SubListener<T, M> *>(mp_sublistener_.get());

    eprosima::fastrtps::TopicDataType *tmptype = nullptr;
    mp_participant_ = participant_ref.get_participant_ptr();
    if (eprosima::fastrtps::Domain::getRegisteredType(
            mp_participant_, p_subListener->m_type_.getName(), &tmptype)) {
      // std::cout << "already exists";
    } else
      eprosima::fastrtps::Domain::registerType(mp_participant_,
                                               &(p_subListener->m_type_));

    p_subListener->callback_pointer_ = callback_ptr;
    Rparam.topic.topicKind = eprosima::fastrtps::rtps::NO_KEY;
    Rparam.topic.topicDataType = p_subListener->m_type_.getName();
    Rparam.topic.topicName = topicname;
    Rparam.topic.historyQos.kind = eprosima::fastrtps::KEEP_LAST_HISTORY_QOS;
    Rparam.topic.historyQos.depth = history;
    Rparam.topic.resourceLimitsQos.max_samples = 200;
    Rparam.qos.m_reliability.kind =
        eprosima::fastrtps::BEST_EFFORT_RELIABILITY_QOS;
    Rparam.qos.m_durability.kind = eprosima::fastrtps::VOLATILE_DURABILITY_QOS;
    mp_subscriber_ = eprosima::fastrtps::Domain::createSubscriber(
        mp_participant_, Rparam,
        (eprosima::fastrtps::SubscriberListener *)mp_sublistener_.get());
    if (mp_subscriber_ == nullptr) return false;

    return true;
  }
  /**
   * Implementation of detach function for a subscriber.
   * The create function should be called before detach.
   * @return True if success.
   * */
  bool detach() {
    if (mp_participant_ == nullptr || mp_subscriber_ == nullptr) {
      std::cout << "Call method error in " << __FILE__ << "," << __LINE__
                << ". You are supposed to CREATE before a DETACH." << std::endl;
      return false;
    }

    if (eprosima::fastrtps::Domain::removeSubscriber(mp_subscriber_)) {
      mp_sublistener_ = nullptr;
      return true;
    } else {
      return false;
    }
  }

 private:
  eprosima::fastrtps::Subscriber *mp_subscriber_;
  eprosima::fastrtps::SubscriberAttributes Rparam;
  eprosima::fastrtps::Participant *mp_participant_;
  std::unique_ptr<eprosima::fastrtps::SubscriberListener> mp_sublistener_;

 public:
  template <typename T, typename M>
  class SubListener : public eprosima::fastrtps::SubscriberListener {
   public:
    SubListener(){};
    ~SubListener() override {
      std::cout << "deconstrunct sub listener of " << m_type_.getName()
                << std::endl;
    };
    void onNewDataMessage(eprosima::fastrtps::Subscriber *sub) override {
      if (sub->takeNextData((void *)&m_data_, &m_info_)) {
        if (m_info_.sampleKind == eprosima::fastrtps::rtps::ALIVE) {
          callback_pointer_(&m_data_);
        }
      }
    }
    eprosima::fastrtps::SampleInfo_t m_info_;
    T m_data_;
    M m_type_;
    std::function<void(T *)> callback_pointer_;
  };
};
}  // namespace cmdrDDS

#endif /*end of DDS_ABSTACT_SUBSCRIBER_H_*/
