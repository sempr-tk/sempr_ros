#include "PublishBuilder.hpp"
#include "Publish.hpp"
#include <rete-reasoner/Exceptions.hpp>

namespace {

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

PublishBuilder::PublishBuilder()
    : rete::NodeBuilder("ros:publish", rete::NodeBuilder::BuilderType::EFFECT)
{
}

/**
    [0]     topic
    [1-...] (string) values
*/
rete::Production::Ptr PublishBuilder::buildEffect(
        rete::ArgumentList& args) const
{
    if (args.size() < 1)
        throw rete::NodeBuilderException(
                "Wrong number of arguments. Need at least a topic name.");

    rete::PersistentInterpretation<std::string> topic;
    if (args[0].isConst() && args[0].getAST().isString())
    {
        auto acc = std::make_shared<rete::ConstantAccessor<std::string>>(args[0].getAST());
        acc->index() = 0;
        topic = acc->getInterpretation<std::string>()->makePersistent();
    }
    else if (args[0].isVariable() && args[0].getAccessor() &&
             args[0].getAccessor()->getInterpretation<std::string>())
    {
        topic = args[0].getAccessor()->getInterpretation<std::string>()->makePersistent();
    }
    else
    {
        throw rete::NodeBuilderException(
                "First argument must be a string specifying the topic name");
    }

    auto node = std::make_shared<Publish>(std::move(topic));

    for (size_t i = 1; i < args.size(); i++)
    {
        auto val = stringConversionFromArg(args[i]);
        if (!val)
            throw rete::NodeBuilderException(
                    "Value of " + args[i].getAST() +
                    " not convertible to string.");
        node->addValue(std::move(val));
    }

    return node;
}

}}
