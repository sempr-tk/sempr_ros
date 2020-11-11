#ifndef SEMPR_ROS_PUBLISHBUILDER_HPP_
#define SEMPR_ROS_PUBLISHBUILDER_HPP_

#include <rete-reasoner/NodeBuilder.hpp>

namespace sempr { namespace ros {

class PublishBuilder : public rete::NodeBuilder {
public:
    PublishBuilder();
    rete::Production::Ptr buildEffect(rete::ArgumentList& args) const override;
};

}}

#endif /* include guard: SEMPR_ROS_PUBLISHBUILDER_HPP_ */
