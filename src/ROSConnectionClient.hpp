#include <sempr-gui/AbstractInterface.hpp>

#include <ros/ros.h>
#include "sempr_ros/SemprECUpdate.h"
#include "sempr_ros/SemprTripleUpdate.h"

namespace sempr { namespace gui {

/**
    Implementation of the sempr::gui::AbstractInterface using ROS for the
    communication.
*/
class ROSConnectionClient : public AbstractInterface {
    ros::NodeHandle nh_;
    ros::Subscriber subECUpdates_;
    ros::Subscriber subTripleUpdates_;

    ros::ServiceClient addEC_, modifyEC_, removeEC_;
    ros::ServiceClient listTriples_, listEC_, getRules_;
    ros::ServiceClient getReteNetwork_;
    ros::ServiceClient explainTriple_, explainEC_;

    void onECUpdate(const sempr_ros::SemprECUpdate& update);
    void onTripleUpdate(const sempr_ros::SemprTripleUpdate& update);
public:
    using Ptr = std::shared_ptr<ROSConnectionClient>;

    ROSConnectionClient();

    Graph getReteNetworkRepresentation() override;
    ExplanationGraph getExplanation(sempr::Triple::Ptr triple) override;
    ExplanationGraph getExplanation(const ECData &ec) override;
    std::vector<Rule> getRulesRepresentation() override;
    std::vector<ECData> listEntityComponentPairs() override;
    std::vector<sempr::Triple> listTriples() override;
    void addEntityComponentPair(const ECData &) override;
    void modifyEntityComponentPair(const ECData &) override;
    void removeEntityComponentPair(const ECData &) override;
};

}}
