#!/usr/bin/python
import pydot

graph = pydot.Dot(graph_type='digraph')
node_a = pydot.Node("Node A", style="filled", fillcolor="red")
node_b = pydot.Node("Node B", style="filled", fillcolor="green")
node_c = pydot.Node("Node C", style="filled", fillcolor="#0000ff")
node_d = pydot.Node("Node D", style="filled", fillcolor="#976856")
graph.add_node(node_a)
graph.add_node(node_b)
graph.add_node(node_c)
graph.add_node(node_d)
graph.add_edge(pydot.Edge(node_a, node_b))
graph.add_edge(pydot.Edge(node_b, node_c))
graph.add_edge(pydot.Edge(node_c, node_d))
graph.add_edge(pydot.Edge(node_d, node_a, label="and back we go again", labelfontcolor="#009933", fontsize="10.0", color="blue"))
graph.write_png('rqt_test.png')
