# Transports information about a rule in the sempr core
# -- a unique id, a non-unique name, the string it was constructed from and
# a list of identifiers for the effect nodes in order to connect a rule with
# the part of the rete network that implements it.

uint64 id
string name
string ruleString
string[] effectNodes
