#include <sempr/nodes/ECNodeBuilder.hpp>
#include <sempr/component/TextComponent.hpp>
#include <sempr/component/TripleContainer.hpp>
#include <sempr/component/TripleVector.hpp>
#include <sempr/component/TriplePropertyMap.hpp>
#include <sempr/component/AffineTransform.hpp>
#include <sempr/component/GeosGeometry.hpp>

#include <sempr/SeparateFileStorage.hpp>

#include <sempr/Core.hpp>

#include <sempr-gui/DirectConnectionBuilder.hpp>
#include "ROSConnectionServer.hpp"
#include "nodes/SetParamBuilder.hpp"
#include "nodes/DynamicReconfigureBuilder.hpp"
#include "nodes/PublishBuilder.hpp"


#include <ros/ros.h>

using namespace sempr;
using namespace sempr::gui;


int main(int argc, char** args)
{
    ::ros::init(argc, args, "sempr_server");

    // Create a "sempr_data" directory in the working dir if it does not
    // already exist. This is the place where everything will be stored in.
    // TODO: Make this a configuration option/parameter.
    //
    ::ros::NodeHandle nh("~");
    std::string dbPath;
    nh.param<std::string>("directory", dbPath, "./sempr_data");

    if (!fs::exists(dbPath)) fs::create_directory(dbPath);
    auto db = std::make_shared<SeparateFileStorage>(dbPath);

    // create a sempr-instance
    Core sempr(db, db); // use the storage for persistence and id generation
    sempr.loadPlugins();

    // and load everything that has been persisted previously.
    auto savedEntities = db->loadAll();
    for (auto& e : savedEntities)
    {
        sempr.addEntity(e);
    }

    // Create a mutex to get a lock on when working with sempr.
    std::mutex semprMutex;

    // create a direct connection implementing the interface for the gui
    auto directConnection = std::make_shared<DirectConnection>(&sempr, semprMutex);

    // wrap the connection in a ROSConnectionServer to expose it in the network
    sempr::ros::ROSConnectionServer connection(directConnection);

    // Before the connection fully works we need to insert it into the reasoner.
    // Well, also register the node builders we might need.
    // TODO: This is tedious, and I probably forgot a whole bunch of builders.
    //       Maybe a plugin-system to would be cool, to just initialize
    //       everything that's available on the system?
    rete::RuleParser& parser = sempr.parser();
    // the next to are used to create a connection between the reasoner and the
    // "directConnection" object
    parser.registerNodeBuilder<ECNodeBuilder<Component>>();
    parser.registerNodeBuilder<DirectConnectionBuilder>(directConnection);
    parser.registerNodeBuilder<DirectConnectionTripleBuilder>(directConnection);
    parser.registerNodeBuilder<sempr::ros::SetParamBuilder>();
    parser.registerNodeBuilder<sempr::ros::DynamicReconfigureBuilder>();
    parser.registerNodeBuilder<sempr::ros::PublishBuilder>();

    // add rules that actually implement the connection into the network
    sempr.addRules(
        "[connectionEC: EC<Component>(?e ?c) -> DirectConnection(?e ?c)]"
        "[connectionTriple: (?s ?p ?o) -> DirectConnectionTriple(?s ?p ?o)]"
    );


    // debug
    auto rules = sempr.addRules(
//        "[true() -> ros:setParam(\"test\" \"hello!\")]"
/*
        "[true(), sum(?int 21 21), mul(?float 0.5 0.5) \
                 -> ros:dynamicReconfigure(\
                     \"dynconf_test/set_parameters\" \
                     \"string\" \"str_param\"    \"foooobar!\" \
                     \"int\"    \"int_param\"    ?int \
                     \"double\" \"double_param\" ?float \
                     \"bool\"   \"bool_param\"   0)]"
*/
        "[true(), sum(?int 21 21), mul(?float 0.5 0.5) \
            -> ros:publish(\"foo/bar/baz\" \"Hello, World!\" ?int ?float)]"
    );


    ::ros::Rate rate(10);
    while (::ros::ok())
    {
        ::ros::spinOnce();
        sempr.performInference();

        rate.sleep();
    }

    return 0;
}
