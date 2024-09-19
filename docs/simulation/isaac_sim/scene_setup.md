
## Creating a New Scene
The easiest way is to reference and copy an existing scene.

## ROS Publishers Through OmniGraph


### Configure Robot Name, ROS_DOMAIN_ID, and Topic Namespaces

Under the Spirit drone prim is an `Omnigraph` component. This component is used to configure the ROS publishers for the robot. The `Omnigraph` component has the following fields:

- `robot_name`: The name of the robot. This is used as the top-level namespace for ROS topics.
- `domain_id`: The ROS domain ID. This is used as the `ROS_DOMAIN_ID` for DDS networking.

The Omnigraph has subgraphs for each ROS publisher type. For example, TFs, Images, and PointClouds. The top-level `robot_name` and `domain_id` fields get fed into the subgraphs. The `Topic Namespaces` field should be set to the topic namespace in the subgraphs. This is used to namespace the ROS topics.

![Image of omnigraphs](omnigraph_config.png)


### Customizing the Omnigraph

Common ROS graphs may be added through the top menu bar: `Isaac Utils > Common Graphs > ROS`.

We recommend copying them into the top-level `Omnigraph` component. Connect the `robot_name` and `domain_id` fields to your workflow. Then, select all the nodes in your workflow, right-click, and create a subgraph.