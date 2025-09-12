# OgnAscentNode [airlab.pegasus]

Provides the Script Node for use in OmniGraph graphs. This node allows custom python code to be executed when the node is computed. The python code is compiled and computed when the graph runs using the embedded interpreter in Kit. The python script itself is stored as an attribute value in the USD where the graph is stored. Dynamic inputs and output attributes can be added to the node to provide inputs and outputs from the script.

