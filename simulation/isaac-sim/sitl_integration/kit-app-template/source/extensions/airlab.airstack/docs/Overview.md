```{csv-table}
**Extension**: {{ extension_version }},**Documentation Generated**: {sub-ref}`today`,{ref}`changelog_airlab.airstack`
```

(ext_airlab.airstack)=

# Overview

This extension provides an implementation of an OmniGraph node that allows you to write scripts that mimic the
functionality of bespoke nodes. This node type executes custom python code when any node of its type is computed.
The python code is compiled and computed when the graph runs using the embedded interpreter in Kit. The python script
itself is stored as an attribute value in the USD where the graph is stored. Dynamic inputs and output attributes can
be added to the node to provide inputs and outputs to the script.

```{warning}
As this node executes arbitrary Python code there is an inherent security risk when the node is evaluated without
first being validated as being safe by the user. There is a setting that allows the user to opt in to using such
nodes until a more robust approach can be put in place.
```

See the {doc}`complete instructions for the script node<GeneratedNodeDocumentation/OgnAscentNode>`
to learn how it works.

```{toctree}
:maxdepth: 1
:hidden:

GeneratedNodeDocumentation/OgnAscentNode
```
