#ifndef SEMPR_ROS_PUBLISH_HPP_
#define SEMPR_ROS_PUBLISH_HPP_

#include <rete-core/Production.hpp>
#include <rete-core/Accessors.hpp>
#include <rete-core/builtins/NumberToStringConversion.hpp>

#include <ros/ros.h>

#include <vector>
#include <map>

namespace sempr { namespace ros {

/**
    A rule effect that publishes sempr_ros::PublishEffect messages
*/
class Publish : public rete::Production {
    rete::PersistentInterpretation<std::string> topic_;
    std::vector<rete::builtin::NumberToStringConversion> values_;

    ::ros::NodeHandle nh_;

    // buffer publishers for different topics so we wont need to setup
    // connections every time a message is send
    std::map<std::string, ::ros::Publisher> pubs_;

public:
    using Ptr = std::shared_ptr<Publish>;
    Publish(
        rete::PersistentInterpretation<std::string> topic
    );

    void addValue(rete::builtin::NumberToStringConversion value);

    void execute(
            rete::Token::Ptr, rete::PropagationFlag,
            std::vector<rete::WME::Ptr>&) override;

    std::string toString() const override;
};

}}


#endif /* include guard: SEMPR_ROS_PUBLISH_HPP_ */
