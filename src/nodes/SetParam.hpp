#ifndef SEMPR_ROS_SETPARAM_HPP_
#define SEMPR_ROS_SETPARAM_HPP_

#include <rete-core/Production.hpp>
#include <rete-core/Accessors.hpp>
#include <rete-core/builtins/NumberToStringConversion.hpp>

#include <ros/ros.h>

namespace sempr { namespace ros {

/**
    A rule effect that sets a ros parameter.
    TODO: Discuss: Shall a parameter be deleted when a match gets retracted?
*/
class SetParam : public rete::Production {
    rete::PersistentInterpretation<std::string> paramName_;
    rete::builtin::NumberToStringConversion paramValue_;

    ::ros::NodeHandle nh_;
public:
    using Ptr = std::shared_ptr<SetParam>;
    SetParam(
        rete::PersistentInterpretation<std::string> name,
        rete::builtin::NumberToStringConversion value
    );

    void execute(rete::Token::Ptr, rete::PropagationFlag, std::vector<rete::WME::Ptr>&) override;
    std::string toString() const override;
};

}}

#endif /* include guard: SEMPR_ROS_SETPARAM_HPP_ */

