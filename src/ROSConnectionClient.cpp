#include "ROSConnectionClient.hpp"
#include <sempr/component/TripleContainer.hpp> // for sempr::Triple

#include "sempr_ros/ListRules.h"
#include "sempr_ros/ListTriples.h"
#include "sempr_ros/ListEC.h"
#include "sempr_ros/EC.h"
#include "sempr_ros/GetReteNetwork.h"
#include "sempr_ros/ExplainTriple.h"
#include "sempr_ros/ExplainEC.h"

namespace sempr { namespace ros {

    using namespace sempr::gui;

ROSConnectionClient::ROSConnectionClient()
    : nh_("")
{
    subECUpdates_ = nh_.subscribe("sempr/ECUpdates", 100,
                                  &ROSConnectionClient::onECUpdate, this);
    subTripleUpdates_ = nh_.subscribe("sempr/TripleUpdates", 100,
                                  &ROSConnectionClient::onTripleUpdate, this);

    addEC_ = nh_.serviceClient<sempr_ros::EC>("sempr/addEC");
    modifyEC_ = nh_.serviceClient<sempr_ros::EC>("sempr/modifyEC");
    removeEC_ = nh_.serviceClient<sempr_ros::EC>("sempr/removeEC");

    listTriples_ = nh_.serviceClient<sempr_ros::ListTriples>("sempr/listTriples");
    listEC_ = nh_.serviceClient<sempr_ros::ListEC>("sempr/listEC");
    getRules_ = nh_.serviceClient<sempr_ros::ListRules>("sempr/listRules");

    getReteNetwork_ = nh_.serviceClient<sempr_ros::GetReteNetwork>("sempr/getReteNetwork");
    explainEC_ = nh_.serviceClient<sempr_ros::ExplainEC>("sempr/explainEC");
    explainTriple_ = nh_.serviceClient<sempr_ros::ExplainTriple>("sempr/explainTriple");

    addEC_.waitForExistence();
    modifyEC_.waitForExistence();
    removeEC_.waitForExistence();
    listTriples_.waitForExistence();
    listEC_.waitForExistence();
    getRules_.waitForExistence();

}


void ROSConnectionClient::onECUpdate(const sempr_ros::SemprECUpdate& update)
{
    ECData data;
    data.entityId = update.data.entityId;
    data.componentId = update.data.componentId;
    data.componentJSON = update.data.componentJSON;
    data.isComponentMutable = update.data.isComponentMutable;
    data.tag = update.data.tag;

    AbstractInterface::Notification n;
    switch (update.updateType) {
        case sempr_ros::SemprECUpdate::ADDED:
            n = AbstractInterface::Notification::ADDED;
            break;
        case sempr_ros::SemprECUpdate::REMOVED:
            n = AbstractInterface::Notification::REMOVED;
            break;
        case sempr_ros::SemprECUpdate::UPDATED:
            n = AbstractInterface::Notification::UPDATED;
            break;
    }

    this->triggerCallback(data, n);
}


void ROSConnectionClient::onTripleUpdate(const sempr_ros::SemprTripleUpdate& update)
{
    sempr::Triple triple(update.data.subject,
                         update.data.predicate,
                         update.data.object);

    AbstractInterface::Notification n;
    switch (update.updateType) {
        case sempr_ros::SemprECUpdate::ADDED:
            n = AbstractInterface::Notification::ADDED;
            break;
        case sempr_ros::SemprECUpdate::REMOVED:
            n = AbstractInterface::Notification::REMOVED;
            break;
        case sempr_ros::SemprECUpdate::UPDATED:
            n = AbstractInterface::Notification::UPDATED;
            break;
    }

    this->triggerTripleCallback(triple, n);
}

std::vector<Rule> ROSConnectionClient::getRulesRepresentation()
{
    sempr_ros::ListRules list;
    getRules_.call(list.request, list.response);

    std::vector<Rule> rules;
    for (auto& r : list.response.rules)
    {
        Rule rule;
        rule.id = r.id;
        rule.name = r.name;
        rule.ruleString = r.ruleString;
        rule.effectNodes = r.effectNodes;

        rules.push_back(rule);
    }

    return rules;
}


std::vector<ECData> ROSConnectionClient::listEntityComponentPairs()
{
    sempr_ros::ListEC list;
    listEC_.call(list.request, list.response);

    std::vector<ECData> ecs;
    for (auto& ec : list.response.ecs)
    {
        ECData data;
        data.componentId = ec.componentId;
        data.componentJSON = ec.componentJSON;
        data.entityId = ec.entityId;
        data.isComponentMutable = ec.isComponentMutable;
        data.tag = ec.tag;

        ecs.push_back(data);
    }

    return ecs;
}


std::vector<sempr::Triple> ROSConnectionClient::listTriples()
{
    sempr_ros::ListTriples list;
    listTriples_.call(list.request, list.response);

    std::vector<sempr::Triple> triples;
    for (auto& t : list.response.triples)
    {
        sempr::Triple triple(t.subject, t.predicate, t.object);
        triples.push_back(triple);
    }

    return triples;
}

void ROSConnectionClient::addEntityComponentPair(const ECData& data)
{
    sempr_ros::EC ec;
    ec.request.data.componentId = data.componentId;
    ec.request.data.componentJSON = data.componentJSON;
    ec.request.data.entityId = data.entityId;
    ec.request.data.isComponentMutable = data.isComponentMutable;
    ec.request.data.tag = data.tag;

    addEC_.call(ec.request, ec.response);
    // TODO: error handling?
}

void ROSConnectionClient::modifyEntityComponentPair(const ECData& data)
{
    sempr_ros::EC ec;
    ec.request.data.componentId = data.componentId;
    ec.request.data.componentJSON = data.componentJSON;
    ec.request.data.entityId = data.entityId;
    ec.request.data.isComponentMutable = data.isComponentMutable;
    ec.request.data.tag = data.tag;

    modifyEC_.call(ec.request, ec.response);
}

void ROSConnectionClient::removeEntityComponentPair(const ECData& data)
{
    sempr_ros::EC ec;
    ec.request.data.componentId = data.componentId;
    ec.request.data.componentJSON = data.componentJSON;
    ec.request.data.entityId = data.entityId;
    ec.request.data.isComponentMutable = data.isComponentMutable;
    ec.request.data.tag = data.tag;

    removeEC_.call(ec.request, ec.response);
}

Graph ROSConnectionClient::getReteNetworkRepresentation()
{
    sempr_ros::GetReteNetwork net;
    getReteNetwork_.call(net.request, net.response);

    Graph graph;
    for (auto& n : net.response.network.nodes)
    {
        Node node;
        node.id = n.id;
        node.label = n.text;
        node.type = static_cast<Node::Type>(n.type);

        graph.nodes.insert(node);
    }

    for (auto& e : net.response.network.edges)
    {
        Edge edge;
        edge.from = e.from;
        edge.to = e.to;

        graph.edges.insert(edge);
    }

    return graph;
}

ExplanationGraph ROSConnectionClient::getExplanation(sempr::Triple::Ptr triple)
{
    sempr_ros::ExplainTriple explain;
    explain.request.triple.subject = triple->getField(sempr::Triple::Field::SUBJECT);
    explain.request.triple.predicate = triple->getField(sempr::Triple::Field::PREDICATE);
    explain.request.triple.object = triple->getField(sempr::Triple::Field::OBJECT);

    explainTriple_.call(explain.request, explain.response);

    ExplanationGraph graph;
    for (auto& n : explain.response.explanation.nodes)
    {
        ExplanationNode node;
        node.id = n.id;
        node.str = n.text;
        node.type = static_cast<ExplanationNode::Type>(n.type);

        graph.nodes.insert(node);
    }

    for (auto& e : explain.response.explanation.edges)
    {
        ExplanationEdge edge(e.from, e.to);
        graph.edges.insert(edge);
    }

    return graph;
}

ExplanationGraph ROSConnectionClient::getExplanation(const ECData& ec)
{
    sempr_ros::ExplainEC explain;
    explain.request.ec.componentId = ec.componentId;
    explain.request.ec.componentJSON = ec.componentJSON;
    explain.request.ec.entityId = ec.entityId;
    explain.request.ec.isComponentMutable = ec.isComponentMutable;
    explain.request.ec.tag = ec.tag;

    explainEC_.call(explain.request, explain.response);

    ExplanationGraph graph;
    for (auto& n : explain.response.explanation.nodes)
    {
        ExplanationNode node;
        node.id = n.id;
        node.str = n.text;
        node.type = static_cast<ExplanationNode::Type>(n.type);

        graph.nodes.insert(node);
    }

    for (auto& e : explain.response.explanation.edges)
    {
        ExplanationEdge edge(e.from, e.to);
        graph.edges.insert(edge);
    }

    return graph;
}

}}

