## SEMPR_ROS

This package contains tools to setup a sempr environment in ROS. It implements the `sempr::gui::AbstractInterface` with ros topics and services and thus can provide the equivalent of the `sempr-gui-example-server` and `sempr-gui-example-client` as ros nodes -- names `sempr` for the server and `gui` for the gui (obviously).

You'll probably want to implement your own sempr-environment-node for your use case and add specific interfaces and data to it. But if you just want to test the capabilities of sempr and/or need only a very basic version to model some rules and data, etc., you can use the nodes provided by this package.

### Provided ros-specific conditions/builtins/effects

#### ros:setParam(name, value)

This effect can be used to set a ros parameter. Be aware that the given values
will always be treated as strings. Also, a parameter can only be set once, so
multiple activations of this effect with the same parameter name will lead to
the parameter being overwritten in an undefined sequence. Furthermore, when a
single activation of it is retracted, the parameter is **deleted**! The effect
does not track a history of the values or similar.

Usage example:
```
[(<robot> <on> ?area),
 (?area <recommended_driving_mode> ?mode)
 ->
 ros:setParam("/robot/driving_mode" ?mode)]
```

### SEMPR (server)

Take a look at `launch/sempr.launch`. This (very simple) launch file starts the server part of sempr, the actual reasoner etc., and can be configured with a path to a directory in which persistent data gets stored. The default value is `sempr_data`, which, when started through roslaunch, will lead to your data being stored in `~/.ros/sempr_data`.

Every entity in sempr is stored in a separate file in the specified directory with the entity id as its name (so don't go crazy on the ids!). Manually deleting / cloning / ... an entity is as easy as deleting / copying / ... the file it's stored in. The format is json, so even editing is simple.

But manually editing files is no fun -- take a look at the gui instead.

#### ROS-Interface

Although the interface implemented here was primarly designed to suit the connection to the GUI, noone will stop you from using it in your nodes!

topic | description
-------|-------------
sempr/ECUpdates | Publishes data whenever an Entity-Component-Pair changes. Beware that not all fields in the message will always be filled, e.g. when a component is removed only the `entityId` and `componentId` fields are set.
sempr/TripleUpdates | Publishes triples that are asserted or retracted

service | description
----------|---------------
sempr/addEC | Adds a new component to an entity. The entity is created if it does not already exist. Only `entityId` and `componentJSON` are relevant. You do not get the id of the created component in return -- this is only sent through the topics.
sempr/modifyEC | Changes a component. Simply sets the newly given `componentJSON` value to the component identified through `entityId` and `componentId`.
sempr/removeEC | Removes the component identified by `entityId` and `componentId`.
sempr/listEC | Lists all entity-component information
sempr/listTriples | Lists all triples
sempr/listRules | Lists all rules
sempr/getReteNetwork | Returns a graph representation of the rete network, just for visualization as it only contains nodes with labels
sempr/explainEC | Returns a graph representing the paths on which the given EC was inferred. Only `entityId` and `componentId` need to be given.
sempr/explainTriple | Returns a graph representing the paths on which the given triple was inferred.



### SEMPR-GUI (client)

If not otherwise specified, the launchfile also starts the gui. But you can also
use `rosrun sempr_ros gui` to start more instances of it. The gui will wait for
required services (which are provided by the server) in the `sempr` namespace.

### Editing rules

There are exactly four rules loaded by default within the main sempr node:

- When an Entity-Component-Pair changes, notify the `ROSConnectionServer`
- When a Triple is inferred / retracted, notify the `ROSConnectionServer`

> _Note: The `ROSConnectionServer` is what provides the topics and services required by the GUI._

- Extract the Triples from TripleContainers.

> _Note: When specifying conditions on triples in your rules, e.g. `(?x <type> ?y)`, these refer to `rete::Triple`s which are "first-class" working memory elements (WMEs) in the reasoner. A `TripleContainer` is a component that holds information about any number of triples and is a WME on its own. In order to make use of the triples inside they need to be extracted and provided as WMEs to the reasoner -- and that's what this rule does._

- For every entity `?e` with `(?e <type> <Rules>)` which has a `TextComponent`, interpret the text in it as a rule definition and add it to the reasoner.

The first two rules are needed to be able to work with the gui in general, and the remaining two are what enables you to add rules as data in the system. In order to add custom rules to a fresh, empty sempr instance, do the following:

1. Add a `TriplePropertyMap` to an entity, let's call it `MyRules`.
1. Edit the `TriplePropertyMap`: Add a new entry, with the property name being `type`, the value `Rules` and the type `Resource`. You'll need to specifiy that it is a `Resource` first, else whatever you insert as a value is converted to an integer (which is the default setting).
> _Note: This is what provides the_ `(<MyRules> <type> <Rules>)` _triple._
1. Add a `TextComponent` to the same entity.
1. Define your rules in the newly create `TextComponent`
> _Note: You may add multiple_ `TextComponent`_s to the same entity to group your rules. Whenever a TextComponent is edited, all rules it defined before are removed and the new ones created, which also results in retracting all their consequences and re-inferring them._