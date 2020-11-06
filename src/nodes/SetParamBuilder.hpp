#ifndef SEMPR_ROS_SETPARAMBUILDER_HPP_
#define SEMPR_ROS_SETPARAMBUILDER_HPP_

#include <rete-reasoner/NodeBuilder.hpp>
#include "SetParam.hpp"

namespace sempr { namespace ros {

class SetParamBuilder : public rete::NodeBuilder {
public:
    SetParamBuilder();
    rete::Production::Ptr buildEffect(rete::ArgumentList& args) const override;
};

}}

#endif /* include guard: SEMPR_ROS_SETPARAMBUILDER_HPP_ */
