#ifndef SEMPR_ROS_DYNAMICRECONFIGUREBUILDER_HPP_
#define SEMPR_ROS_DYNAMICRECONFIGUREBUILDER_HPP_

#include <rete-reasoner/NodeBuilder.hpp>

namespace sempr { namespace ros {

class DynamicReconfigureBuilder : public rete::NodeBuilder {
public:
    DynamicReconfigureBuilder();
    rete::Production::Ptr buildEffect(rete::ArgumentList& args) const override;
};

}}

#endif /* include guard: SEMPR_ROS_DYNAMICRECONFIGUREBUILDER_HPP_ */
