#ifndef SEMPR_ROS_CONNECTIONSERVER_HPP_
#define SEMPR_ROS_CONNECTIONSERVER_HPP_

#include <sempr-gui/DirectConnection.hpp>
#include <ros/ros.h>

#include "sempr_ros/EC.h"
#include "sempr_ros/ListEC.h"
#include "sempr_ros/ListTriples.h"
#include "sempr_ros/ListRules.h"
#include "sempr_ros/GetReteNetwork.h"
#include "sempr_ros/ExplainEC.h"
#include "sempr_ros/ExplainTriple.h"

namespace sempr { namespace ros {

/**
    Takes a DirectConnection and exposes its interface in the ros world through
    topics and services.
*/
class ROSConnectionServer {
    ::ros::NodeHandle nh_;
    ::ros::Publisher pubECUpdates_, pubTripleUpdates_;

    ::ros::ServiceServer addEC_, modifyEC_, removeEC_;
    ::ros::ServiceServer listTriples_, listEC_, listRules_;
    ::ros::ServiceServer getReteNetwork_;
    ::ros::ServiceServer explainTriple_, explainEC_;

    sempr::gui::DirectConnection::Ptr connection_;

public:
    ROSConnectionServer(sempr::gui::DirectConnection::Ptr connection);

    bool addEC(sempr_ros::EC::Request&, sempr_ros::EC::Response&);
    bool modifyEC(sempr_ros::EC::Request&, sempr_ros::EC::Response&);
    bool removeEC(sempr_ros::EC::Request&, sempr_ros::EC::Response&);

    bool listTriples(sempr_ros::ListTriples::Request&, sempr_ros::ListTriples::Response&);
    bool listEC(sempr_ros::ListEC::Request&, sempr_ros::ListEC::Response&);
    bool listRules(sempr_ros::ListRules::Request&, sempr_ros::ListRules::Response&);

    bool getReteNetwork(sempr_ros::GetReteNetwork::Request&, sempr_ros::GetReteNetwork::Response&);
    bool explainTriple(sempr_ros::ExplainTriple::Request&, sempr_ros::ExplainTriple::Response&);
    bool explainEC(sempr_ros::ExplainEC::Request&, sempr_ros::ExplainEC::Response&);
};


}}


#endif /* include guard: SEMPR_ROS_CONNECTIONSERVER_HPP_ */
