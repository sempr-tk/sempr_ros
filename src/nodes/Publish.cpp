#include "Publish.hpp"

#include <sempr_ros/PublishEffect.h>

namespace sempr { namespace ros {

Publish::Publish(rete::PersistentInterpretation<std::string> topic)
    :
        topic_(std::move(topic))
{
}


Publish::Publish(
        std::unique_ptr<rete::ConstantAccessor<std::string>> topic)
    :
        topic_(topic->getInterpretation<std::string>()->makePersistent())
{
    if (topic)
    {
        std::string topicName;
        topic->getInterpretation<std::string>()->getValue(rete::WME::Ptr(), topicName);
        pubs_[topicName] = nh_.advertise<sempr_ros::PublishEffect>(topicName, 100);
    }
}

void Publish::addValue(rete::builtin::NumberToStringConversion value)
{
    values_.push_back(std::move(value));
}

std::string Publish::toString() const
{
    return "ros:publish";
}

void Publish::execute(
        rete::Token::Ptr token, rete::PropagationFlag flag,
        std::vector<rete::WME::Ptr>&)
{
    // which topic to publish on
    std::string topic;
    topic_.interpretation->getValue(token, topic);

    // construct the message
    sempr_ros::PublishEffect msg;
    msg.action =
        (flag == rete::PropagationFlag::ASSERT ? sempr_ros::PublishEffect::ASSERT :
         flag == rete::PropagationFlag::RETRACT ? sempr_ros::PublishEffect::RETRACT :
         flag == rete::PropagationFlag::UPDATE ? sempr_ros::PublishEffect::UPDATE :
         0);

    for (auto& val : values_)
    {
        std::string value;
        val.getValue(token, value);
        msg.values.push_back(value);
    }

    // get a publisher to send the message
    auto it = pubs_.find(topic);
    if (it != pubs_.end())
    {
        it->second.publish(msg);
    }
    else
    {
        pubs_[topic] = nh_.advertise<sempr_ros::PublishEffect>(topic, 100);
        ::ros::Duration(0.1).sleep(); // small sleep to allow subscribers to connect!
        pubs_[topic].publish(msg);
    }
}

}}
