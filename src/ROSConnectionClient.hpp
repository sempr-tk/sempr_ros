#ifndef SEMPR_ROS_CONNECTIONCLIENT_HPP
#define SEMPR_ROS_CONNECTIONCLIENT_HPP

#include <sempr-gui/AbstractInterface.hpp>

#include <ros/ros.h>
#include "sempr_ros/SemprECUpdate.h"
#include "sempr_ros/SemprTripleUpdate.h"

namespace sempr { namespace ros {

/**
    Implementation of the sempr::gui::AbstractInterface using ROS for the
    communication.
*/
class ROSConnectionClient : public gui::AbstractInterface {
    ::ros::NodeHandle nh_;
    ::ros::Subscriber subECUpdates_;
    ::ros::Subscriber subTripleUpdates_;

    ::ros::ServiceClient addEC_, modifyEC_, removeEC_;
    ::ros::ServiceClient listTriples_, listEC_, getRules_;
    ::ros::ServiceClient getReteNetwork_;
    ::ros::ServiceClient explainTriple_, explainEC_;

    void onECUpdate(const sempr_ros::SemprECUpdate& update);
    void onTripleUpdate(const sempr_ros::SemprTripleUpdate& update);
public:
    using Ptr = std::shared_ptr<ROSConnectionClient>;

    ROSConnectionClient();

    gui::Graph getReteNetworkRepresentation() override;
    gui::ExplanationGraph getExplanation(sempr::Triple::Ptr triple) override;
    gui::ExplanationGraph getExplanation(const gui::ECData &ec) override;
    std::vector<gui::Rule> getRulesRepresentation() override;
    std::vector<gui::ECData> listEntityComponentPairs() override;
    std::vector<sempr::Triple> listTriples() override;
    void addEntityComponentPair(const gui::ECData &) override;
    void modifyEntityComponentPair(const gui::ECData &) override;
    void removeEntityComponentPair(const gui::ECData &) override;
};

}}


#endif /* include guard: SEMPR_ROS_CONNECTIONCLIENT_HPP */
