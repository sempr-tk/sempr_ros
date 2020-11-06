#include "SetParamBuilder.hpp"
#include <rete-reasoner/Exceptions.hpp>

namespace sempr { namespace ros {

SetParamBuilder::SetParamBuilder()
    : rete::NodeBuilder("ros:setParam", rete::NodeBuilder::BuilderType::EFFECT)
{
}

rete::Production::Ptr SetParamBuilder::buildEffect(rete::ArgumentList& args) const
{
    if (args.size() != 2)
    {
        throw rete::NodeBuilderException("Wrong number of arguments");
    }

    std::unique_ptr<rete::AccessorBase> name, value;

    if (args[0].isVariable())
    {
        if (args[0].getAccessor()->getInterpretation<std::string>())
            name.reset(args[0].getAccessor()->clone());
        else
            throw rete::NodeBuilderException("Param name must be a string");
    }
    else
    {
        name.reset(new rete::ConstantAccessor<std::string>(args[0].getAST()));
        name->index() = 0;
    }


    if (args[1].isVariable())
        value.reset(args[1].getAccessor()->clone());
    else
    {
        if (args[1].getAST().isString())
            value.reset(new rete::ConstantAccessor<std::string>(args[1].getAST()));
        else if (args[1].getAST().isInt())
            value.reset(new rete::ConstantAccessor<int>(args[1].getAST().toInt()));
        else if (args[1].getAST().isFloat())
            value.reset(new rete::ConstantAccessor<float>(args[1].getAST().toFloat()));
        else
            throw rete::NodeBuilderException("Invalid type of const in value part");

        value->index() = 0;
    }


    rete::builtin::NumberToStringConversion valueConversion(std::move(value));
    if (!valueConversion)
        throw rete::NodeBuilderException("Value part not convertible to string");

    SetParam::Ptr node(
        new SetParam(
            name->getInterpretation<std::string>()->makePersistent(),
            std::move(valueConversion)
        )
    );

    return node;
}

}}
