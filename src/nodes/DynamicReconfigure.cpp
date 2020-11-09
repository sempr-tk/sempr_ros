#include "DynamicReconfigure.hpp"

#include <ros/duration.h>

namespace sempr { namespace ros {

DynamicReconfigure::DynamicReconfigure(
        rete::PersistentInterpretation<std::string> srvName)
    :
        serviceName_(std::move(srvName))
{
}


void DynamicReconfigure::addIntParam(
        rete::PersistentInterpretation<std::string> name,
        rete::builtin::NumberToNumberConversion<int> value)
{
    intNames_.push_back(std::move(name));
    ints_.push_back(std::move(value));
}

void DynamicReconfigure::addDoubleParam(
        rete::PersistentInterpretation<std::string> name,
        rete::builtin::NumberToNumberConversion<double> value)
{
    doubleNames_.push_back(std::move(name));
    doubles_.push_back(std::move(value));
}

void DynamicReconfigure::addStringParam(
        rete::PersistentInterpretation<std::string> name,
        rete::builtin::NumberToStringConversion value)
{
    strNames_.push_back(std::move(name));
    strs_.push_back(std::move(value));
}

void DynamicReconfigure::addBoolParam(
        rete::PersistentInterpretation<std::string> name,
        rete::builtin::NumberToNumberConversion<int> value)
{
    boolNames_.push_back(std::move(name));
    bools_.push_back(std::move(value));
}


std::string DynamicReconfigure::toString() const
{
    return "DynamicReconfigure " + serviceName_.accessor->toString();
}

void DynamicReconfigure::execute(
        rete::Token::Ptr token, rete::PropagationFlag flag,
        std::vector<rete::WME::Ptr>&)
{
    // how to handle a retraction? ... just ignore...
    if (flag == rete::PropagationFlag::RETRACT) return;

    std::string srvName;
    serviceName_.interpretation->getValue(token, srvName);

    // construct reconf request
    dynamic_reconfigure::Reconfigure::Request req;

    for (size_t i = 0; i < intNames_.size(); i++)
    {
        dynamic_reconfigure::IntParameter param;
        intNames_[i].interpretation->getValue(token, param.name);
        ints_[i].getValue(token, param.value);
        req.config.ints.push_back(param);
    }

    for (size_t i = 0; i < doubleNames_.size(); i++)
    {
        dynamic_reconfigure::DoubleParameter param;
        doubleNames_[i].interpretation->getValue(token, param.name);
        doubles_[i].getValue(token, param.value);
        req.config.doubles.push_back(param);
    }

    for (size_t i = 0; i < strNames_.size(); i++)
    {
        dynamic_reconfigure::StrParameter param;
        strNames_[i].interpretation->getValue(token, param.name);
        strs_[i].getValue(token, param.value);
        req.config.strs.push_back(param);
    }

    for (size_t i = 0; i < boolNames_.size(); i++)
    {
        dynamic_reconfigure::BoolParameter param;
        boolNames_[i].interpretation->getValue(token, param.name);
        int tmp;
        bools_[i].getValue(token, tmp);
        param.value = (tmp != 0);
        req.config.bools.push_back(param);
    }

    dynamic_reconfigure::Reconfigure::Response response;
    auto it = reconfServices_.find(srvName);
    if (it != reconfServices_.end())
    {
        it->second.call(req, response);
    }
    else
    {
        reconfServices_[srvName] =
            nh_.serviceClient<dynamic_reconfigure::Reconfigure>(srvName);

        bool ok = reconfServices_[srvName].waitForExistence(::ros::Duration(2));
        if (ok) reconfServices_[srvName].call(req, response);
        else
        {
            ROS_ERROR_STREAM_NAMED(
                "sempr::ros::DynamicReconfigure",
                srvName << " does not exist");
            reconfServices_.erase(srvName);
        }
    }
}



}}
