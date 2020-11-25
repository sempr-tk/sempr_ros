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
    /**
        C'tor for dynamic topics, which are created on the fly during effect
        execution. One major drawback of this is that the first messages may be
        lost as possible subscribers need some time to create the connection.
    */
    Publish(
        rete::PersistentInterpretation<std::string> topic
    );

    /**
        C'tor for static topics. The topic is advertised in the c'tor already,
        making it easy to give subscribers enough time to connect.
    */
    Publish(
        std::unique_ptr<rete::ConstantAccessor<std::string>> topic
    );

    void addValue(rete::builtin::NumberToStringConversion value);

    void execute(
            rete::Token::Ptr, rete::PropagationFlag,
            std::vector<rete::WME::Ptr>&) override;

    std::string toString() const override;
};

}}


#endif /* include guard: SEMPR_ROS_PUBLISH_HPP_ */
