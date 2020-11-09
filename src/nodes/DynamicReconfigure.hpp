#ifndef SEMPR_ROS_DYNAMICRECONFIGURE_HPP_
#define SEMPR_ROS_DYNAMICRECONFIGURE_HPP_

#include <rete-core/Production.hpp>
#include <rete-core/Accessors.hpp>
#include <rete-core/builtins/NumberToNumberConversion.hpp>
#include <rete-core/builtins/NumberToStringConversion.hpp>

#include <ros/ros.h>
#include <dynamic_reconfigure/Reconfigure.h>

#include <map>
#include <vector>

namespace sempr { namespace ros {

/**
    A rule effect that makes a ros service call to a dynamic-reconfigure service
*/
class DynamicReconfigure : public rete::Production {
    std::vector<
        rete::builtin::NumberToNumberConversion<int>> ints_;
    std::vector<
        rete::PersistentInterpretation<std::string>> intNames_;

    std::vector<
        rete::builtin::NumberToNumberConversion<double>> doubles_;
    std::vector<
        rete::PersistentInterpretation<std::string>> doubleNames_;

    std::vector<
        rete::builtin::NumberToStringConversion> strs_;
    std::vector<
        rete::PersistentInterpretation<std::string>> strNames_;

    std::vector<
        rete::builtin::NumberToNumberConversion<int>> bools_;
    std::vector<
        rete::PersistentInterpretation<std::string>> boolNames_;

    rete::PersistentInterpretation<std::string> serviceName_;

    ::ros::NodeHandle nh_;

    // buffer ::ros::ServiceServers
    std::map<std::string, ::ros::ServiceClient> reconfServices_;

public:

    DynamicReconfigure(
        rete::PersistentInterpretation<std::string> srvName
    );

    void addIntParam(
        rete::PersistentInterpretation<std::string> name,
        rete::builtin::NumberToNumberConversion<int> value);
    void addDoubleParam(
        rete::PersistentInterpretation<std::string> name,
        rete::builtin::NumberToNumberConversion<double> value);
    void addStringParam(
        rete::PersistentInterpretation<std::string> name,
        rete::builtin::NumberToStringConversion value);
    void addBoolParam(
        rete::PersistentInterpretation<std::string> name,
        rete::builtin::NumberToNumberConversion<int> value);


    void execute(
            rete::Token::Ptr, rete::PropagationFlag,
            std::vector<rete::WME::Ptr>&) override;
    std::string toString() const override;
};

}}

#endif /* include guard: SEMPR_ROS_DYNAMICRECONFIGURE_HPP_ */
