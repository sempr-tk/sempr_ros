#include "ROSConnectionServer.hpp"

#include "sempr_ros/SemprECUpdate.h"
#include "sempr_ros/SemprTripleUpdate.h"

namespace sempr { namespace ros {

ROSConnectionServer::ROSConnectionServer(sempr::gui::DirectConnection::Ptr connection)
    : nh_(""), connection_(connection)
{
    pubECUpdates_ = nh_.advertise<sempr_ros::SemprECUpdate>("sempr/ECUpdates", 100);
    pubTripleUpdates_ = nh_.advertise<sempr_ros::SemprTripleUpdate>("sempr/TripleUpdates", 100);

    addEC_ = nh_.advertiseService("sempr/addEC", &ROSConnectionServer::addEC, this);
    modifyEC_ = nh_.advertiseService("sempr/modifyEC", &ROSConnectionServer::modifyEC, this);
    removeEC_ = nh_.advertiseService("sempr/removeEC", &ROSConnectionServer::removeEC, this);

    listTriples_ = nh_.advertiseService("sempr/listTriples", &ROSConnectionServer::listTriples, this);
    listEC_ = nh_.advertiseService("sempr/listEC", &ROSConnectionServer::listEC, this);
    listRules_ = nh_.advertiseService("sempr/listRules", &ROSConnectionServer::listRules, this);

    getReteNetwork_ = nh_.advertiseService("sempr/getReteNetwork", &ROSConnectionServer::getReteNetwork, this);
    explainTriple_ = nh_.advertiseService("sempr/explainTriple", &ROSConnectionServer::explainTriple, this);
    explainEC_ = nh_.advertiseService("sempr/explainEC", &ROSConnectionServer::explainEC, this);


    connection_->setUpdateCallback(
        std::bind(
            &ROSConnectionServer::ecUpdateCallback, this,
            std::placeholders::_1,
            std::placeholders::_2
        )
    );

    connection_->setTripleUpdateCallback(
        std::bind(
            &ROSConnectionServer::tripleUpdateCallback, this,
            std::placeholders::_1,
            std::placeholders::_2
        )
    );
}

gui::ECData msgToECData(const sempr_ros::ECData& msg)
{
    gui::ECData data;
    data.componentId = msg.componentId;
    data.componentJSON = msg.componentJSON;
    data.entityId = msg.entityId;
    data.isComponentMutable = msg.isComponentMutable;
    data.tag = msg.tag;

    return data;
}


void ROSConnectionServer::ecUpdateCallback(
        gui::AbstractInterface::callback_t::first_argument_type data,
        gui::AbstractInterface::callback_t::second_argument_type action)
{
    sempr_ros::SemprECUpdate msg;
    msg.data.componentId = data.componentId;
    msg.data.componentJSON = data.componentJSON;
    msg.data.entityId = data.entityId;
    msg.data.isComponentMutable = data.isComponentMutable;
    msg.data.tag = data.tag;

    switch(action) {
        case gui::AbstractInterface::Notification::ADDED:
            msg.updateType = msg.ADDED;
            break;
        case gui::AbstractInterface::Notification::REMOVED:
            msg.updateType = msg.REMOVED;
            break;
        case gui::AbstractInterface::Notification::UPDATED:
            msg.updateType = msg.UPDATED;
            break;
    }

    pubECUpdates_.publish(msg);
}

void ROSConnectionServer::tripleUpdateCallback(
        gui::AbstractInterface::triple_callback_t::first_argument_type data,
        gui::AbstractInterface::triple_callback_t::second_argument_type action)
{
    sempr_ros::SemprTripleUpdate msg;
    msg.data.subject = data.getField(sempr::Triple::Field::SUBJECT);
    msg.data.predicate = data.getField(sempr::Triple::Field::PREDICATE);
    msg.data.object = data.getField(sempr::Triple::Field::OBJECT);

    switch(action) {
        case gui::AbstractInterface::Notification::ADDED:
            msg.updateType = msg.ADDED;
            break;
        case gui::AbstractInterface::Notification::REMOVED:
            msg.updateType = msg.REMOVED;
            break;
        case gui::AbstractInterface::Notification::UPDATED:
            // TODO this must be an error, triples are immutable...
            break;
    }

    pubTripleUpdates_.publish(msg);
}

bool ROSConnectionServer::addEC(sempr_ros::EC::Request& req, sempr_ros::EC::Response& res)
{
    gui::ECData data = msgToECData(req.data);
    connection_->addEntityComponentPair(data);

    // TODO set response result / any error handling?

    return true;
}

bool ROSConnectionServer::modifyEC(sempr_ros::EC::Request& req, sempr_ros::EC::Response& res)
{
    gui::ECData data = msgToECData(req.data);
    connection_->modifyEntityComponentPair(data);

    return true;
}

bool ROSConnectionServer::removeEC(sempr_ros::EC::Request& req, sempr_ros::EC::Response& res)
{
    gui::ECData data = msgToECData(req.data);
    connection_->removeEntityComponentPair(data);

    return true;
}



bool ROSConnectionServer::listTriples(sempr_ros::ListTriples::Request&,
                                      sempr_ros::ListTriples::Response& res)
{
    auto triples = connection_->listTriples();
    res.triples.reserve(triples.size());

    for (auto& t : triples)
    {
        sempr_ros::Triple triple;
        triple.subject = t.getField(sempr::Triple::Field::SUBJECT);
        triple.predicate = t.getField(sempr::Triple::Field::PREDICATE);
        triple.object = t.getField(sempr::Triple::Field::OBJECT);

        res.triples.push_back(triple);
    }

    return true;
}

bool ROSConnectionServer::listEC(sempr_ros::ListEC::Request&,
                                 sempr_ros::ListEC::Response& res)
{
    auto ecs = connection_->listEntityComponentPairs();
    res.ecs.reserve(ecs.size());

    for (auto& ec : ecs)
    {
        sempr_ros::ECData msg;
        msg.componentId = ec.componentId;
        msg.componentJSON = ec.componentJSON;
        msg.entityId = ec.entityId;
        msg.isComponentMutable = ec.isComponentMutable;
        msg.tag = ec.tag;

        res.ecs.push_back(msg);
    }

    return true;
}

bool ROSConnectionServer::listRules(sempr_ros::ListRules::Request&,
                                    sempr_ros::ListRules::Response& res)
{
    auto rules = connection_->getRulesRepresentation();
    res.rules.reserve(rules.size());

    for (auto& r : rules)
    {
        sempr_ros::Rule rule;
        rule.effectNodes = r.effectNodes;
        rule.id = r.id;
        rule.name = r.name;
        rule.ruleString = r.ruleString;

        res.rules.push_back(rule);
    }

    return true;
}

bool ROSConnectionServer::getReteNetwork(sempr_ros::GetReteNetwork::Request&,
                                         sempr_ros::GetReteNetwork::Response& res)
{
    auto network = connection_->getReteNetworkRepresentation();

    for (auto& n : network.nodes)
    {
        sempr_ros::Node node;
        node.id = n.id;
        node.text = n.label;
        node.type = n.type;

        res.network.nodes.push_back(node);
    }

    for (auto& e : network.edges)
    {
        sempr_ros::Edge edge;
        edge.from = e.from;
        edge.to = e.to;

        res.network.edges.push_back(edge);
    }

    return true;
}


bool ROSConnectionServer::explainTriple(sempr_ros::ExplainTriple::Request& req,
                                        sempr_ros::ExplainTriple::Response& res)
{
    auto triple = std::make_shared<sempr::Triple>(
                    req.triple.subject,
                    req.triple.predicate,
                    req.triple.object);
    auto explanation = connection_->getExplanation(triple);

    for (auto& n : explanation.nodes)
    {
        sempr_ros::Node node;
        node.id = n.id;
        node.text = n.str;
        node.type = n.type;

        res.explanation.nodes.push_back(node);
    }

    for (auto& e : explanation.edges)
    {
        sempr_ros::Edge edge;
        edge.from = std::get<0>(e);
        edge.to = std::get<1>(e);

        res.explanation.edges.push_back(edge);
    }

    return true;
}

bool ROSConnectionServer::explainEC(sempr_ros::ExplainEC::Request& req,
                                    sempr_ros::ExplainEC::Response& res)
{
    gui::ECData data = msgToECData(req.ec);

    auto explanation = connection_->getExplanation(data);

    for (auto& n : explanation.nodes)
    {
        sempr_ros::Node node;
        node.id = n.id;
        node.text = n.str;
        node.type = n.type;

        res.explanation.nodes.push_back(node);
    }

    for (auto& e : explanation.edges)
    {
        sempr_ros::Edge edge;
        edge.from = std::get<0>(e);
        edge.to = std::get<1>(e);

        res.explanation.edges.push_back(edge);
    }

    return true;
}


}}
