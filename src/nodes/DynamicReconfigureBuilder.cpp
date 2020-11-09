#include "DynamicReconfigureBuilder.hpp"
#include "DynamicReconfigure.hpp"
#include <rete-reasoner/Exceptions.hpp>

namespace {

rete::PersistentInterpretation<std::string> stringInterpretationFromArg(
        const rete::Argument& arg)
{
    if (arg.isConst())
    {
        if (arg.getAST().isString())
        {
            auto acc = std::make_shared<rete::ConstantAccessor<std::string>>(
                            arg.getAST());
            acc->index() = 0;
            return acc->getInterpretation<std::string>()->makePersistent();
        }
        else
        {
            throw rete::NodeBuilderException(
                    "Given constant " + arg.getAST() + " is not a string.");
        }
    }
    else
    {
        if (arg.getAccessor() &&
            arg.getAccessor()->getInterpretation<std::string>())
        {
            return arg.getAccessor()->getInterpretation<std::string>()->makePersistent();
        }
        else
        {
            throw rete::NodeBuilderException(
                    "Given variable " + arg.getVariableName() +
                    " is not bound to a string value");
        }
    }
}


rete::AccessorBase::Ptr accessorFromConst(const rete::ast::Argument& arg)
{
    rete::AccessorBase::Ptr acc;
    if (arg.isString())
        acc = std::make_shared<rete::ConstantAccessor<std::string>>(arg.toString());
    else if (arg.isInt())
        acc = std::make_shared<rete::ConstantAccessor<int>>(arg.toInt());
    else if (arg.isFloat())
        acc = std::make_shared<rete::ConstantAccessor<double>>(arg.toFloat());
    else
        throw rete::NodeBuilderException(
                "Given const is neither string nor int or float: " + arg);

    acc->index() = 0;
    return acc;
}

template <class T>
rete::builtin::NumberToNumberConversion<T> numberConversionFromArg(
        const rete::Argument& arg)
{
    std::unique_ptr<rete::AccessorBase> acc;
    if (arg.isConst())
        acc.reset(accessorFromConst(arg.getAST())->clone());
    else if (arg.getAccessor())
        acc.reset(arg.getAccessor()->clone());

    rete::builtin::NumberToNumberConversion<T> conv(std::move(acc));
    return conv;
}

rete::builtin::NumberToStringConversion stringConversionFromArg(
        const rete::Argument& arg)
{
    std::unique_ptr<rete::AccessorBase> acc;
    if (arg.isConst())
        acc.reset(accessorFromConst(arg.getAST())->clone());
    else if (arg.getAccessor())
        acc.reset(arg.getAccessor()->clone());

    rete::builtin::NumberToStringConversion conv(std::move(acc));
    return conv;
}

}


namespace sempr { namespace ros {

DynamicReconfigureBuilder::DynamicReconfigureBuilder()
    : rete::NodeBuilder("ros:dynamicReconfigure", rete::NodeBuilder::BuilderType::EFFECT)
{
}


/**
    Required arguments?
    [0]: name of the service to call
    [3n+1] = "int" | "bool" | "double" | "string"
    [3n+2] = parameter name
    [3n+3] = parameter value
*/
rete::Production::Ptr DynamicReconfigureBuilder::buildEffect(
        rete::ArgumentList& args) const
{
    if (args.size() < 4 || args.size() % 3 != 1)
        throw rete::NodeBuilderException(
                "Wrong number of arguments. "
                "srvName (type name value)+");

    // set the first parameter -- the service name
    rete::PersistentInterpretation<std::string> srvName;
    if (args[0].isConst() && args[0].getAST().isString())
    {
        auto acc = std::make_shared<rete::ConstantAccessor<std::string>>(args[0].getAST());
        acc->index() = 0;
        srvName = acc->getInterpretation<std::string>()->makePersistent();
    }
    else if (args[0].isVariable() && args[0].getAccessor() &&
             args[0].getAccessor()->getInterpretation<std::string>())
    {
        srvName = args[0].getAccessor()->getInterpretation<std::string>()->makePersistent();
    }
    else
    {
        throw rete::NodeBuilderException(
                "First argument must be a string specifying the service name");
    }

    auto node = std::make_shared<DynamicReconfigure>(std::move(srvName));

    // setup all other parameters
    for (size_t i = 1; i < args.size()-2; i += 3)
    {
        auto& paramType = args[i];
        auto& paramName = args[i+1];
        auto& paramValue = args[i+2];

        if (paramType.isConst() && paramType.getAST().isString())
        {

            if (paramType.getAST() == "int")
            {
                auto name = stringInterpretationFromArg(paramName);
                auto value = numberConversionFromArg<int>(paramValue);
                if (!value)
                    throw rete::NodeBuilderException(
                            "Value of " + paramValue.getAST() +
                            " not convertible to int.");
                node->addIntParam(std::move(name), std::move(value));
            }
            else if (paramType.getAST() == "double")
            {
                auto name = stringInterpretationFromArg(paramName);
                auto value = numberConversionFromArg<double>(paramValue);
                if (!value)
                    throw rete::NodeBuilderException(
                            "Value of " + paramValue.getAST() +
                            " not convertible to double.");
                node->addDoubleParam(std::move(name), std::move(value));
            }
            else if (paramType.getAST() == "bool")
            {
                auto name = stringInterpretationFromArg(paramName);
                auto value = numberConversionFromArg<int>(paramValue);
                if (!value)
                    throw rete::NodeBuilderException(
                            "Value of " + paramValue.getAST() +
                            " not convertible to int (for bool interpretation)");
                node->addBoolParam(std::move(name), std::move(value));
            }
            else if (paramType.getAST() == "string")
            {
                auto name = stringInterpretationFromArg(paramName);
                auto value = stringConversionFromArg(paramValue);
                if (!value)
                    throw rete::NodeBuilderException(
                            "Value of " + paramValue.getAST() +
                            " not convertible to string");
                node->addStringParam(std::move(name), std::move(value));
            }
            else
            {
                throw rete::NodeBuilderException(
                        "Unsupported dynamic_reconfigure parameter type '"
                        + paramType.getAST() + "'. "
                        "Use one of 'int', 'double', 'bool', 'string'.");
            }
        }
        else
        {
            throw rete::NodeBuilderException(
                    paramType.getAST() + " -- "
                    "parameter type must be a parse-time-constant string");
        }
    }

    return node;
}

}}
