#include "SetParam.hpp"

namespace sempr { namespace ros {

SetParam::SetParam(
        rete::PersistentInterpretation<std::string> name,
        rete::builtin::NumberToStringConversion value)
    :
        paramName_(std::move(name)),
        paramValue_(std::move(value))
{
}

void SetParam::execute(
        rete::Token::Ptr token,
        rete::PropagationFlag flag,
        std::vector<rete::WME::Ptr>&)
{
    std::string name, value;
    paramName_.interpretation->getValue(token, name);
    paramValue_.getValue(token, value);

    if (flag == rete::PropagationFlag::ASSERT || flag == rete::PropagationFlag::UPDATE)
    {
        nh_.setParam(name, value);
    }
    else if (flag == rete::PropagationFlag::RETRACT)
    {
        nh_.deleteParam(name);
    }
}

std::string SetParam::toString() const
{
    return "ros::setParam(" +
                paramName_.accessor->toString() + ", " +
                paramValue_.toString() + ")";
}


}}
