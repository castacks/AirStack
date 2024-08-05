# Behavior Tree

This package is a ROS implementation of a behavior tree.

![picture](images/rviz_example.png)

## Overview

Behavior trees define how a set of actions and conditions should be used to accomplish a task. The tree is made up of execution nodes, control flow nodes, and decorator nodes.

Action nodes and condition nodes are the two types of execution nodes. These nodes are where the state of the system is checked and actions are performed. A condition node returns either SUCCESS or FAILURE to indicate what the state of some part of the system is. For example, an "On Ground" condition node could indicate whether or not the robot is on the ground. These are shown as the oval shaped nodes in the figure above. Green ones indicate SUCCESS, red ones indicate FAILURE. If the body of the node is filled with color, this indicates that the behavior tree is currently checking the value of the condition. If the body color is white but the outline is colored, this means that the behavior tree is not currently checking the value of the condition, so it is not using it to decide which actions to perform. Action nodes are used to make the system perform some action if they are active. Action nodes are shown as square nodes with text in the figure above. They can be either active or inactive. Active nodes are shown with a red, blue, or green body color, inactive nodes are shown with a white body color. If a node is active this means the behavior tree has decided to perform the action. If the node is inactive the behavior tree is not trying to perform the action. For example, if the robot should be taking off, then the "Takeoff" action should be active, while the "Land" action should be inactive. An active action node can have a SUCCESS, RUNNING, or FAILURE status. SUCCESS indicates that the action has is done being performed and finished successfully, FAILURE indicates the action is done but has failed, RUNNING indicates that the action is still being performed. In the figure above green corresponds to SUCCESS, blue to RUNNING and red to FAILURE for action nodes.

Control flow nodes determine which condition nodes are checked and which action nodes are active or inactive. There are three types of control nodes currently implemented:

- Fallback Nodes: This node returns FAILURE if and only if all of its children return FAILURE. If one of its children return RUNNING or SUCCESS, it returns RUNNING or SUCCESS and no subsequent children's statuses are checked. These are shown with a ? in the figure above.
- Sequence Nodes: This node returns SUCCESS if and only if all of its children return SUCCESS. If one of its children return RUNNING or FAILURE, it returns RUNNING or FAILURE and no subsequent children's statuses are checked. These are shown with a → in the figure above.
- Parallel Nodes: This node has N chldren. It returns SUCCESS if M of these children return SUCCESS, for some M ≤ N. It returns FAILURE of N - M + 1 children return FAILURE. Otherwise, it returns RUNNING.

Decorator nodes have one child and its return value is determined by some user defined rule given the child's status as input. There is currently one type of decorator nodes implemented:

- Not Decorator Node: This has a single condition node as a child. If the child returns SUCCESS, the Not Decorator returns FAILURE. If the child returns FAILURE, the Not Decorator returns SUCCESS.

A more detailed description of behavior trees can be found here: https://arxiv.org/pdf/1709.00084.pdf.

## ROS Interface

This section describes how the behavior tree is implemented in ROS and how nodes implementing conditions and actions should interact with it. A full working example can be found in this repository: [https://bitbucket.org/cmusubt/behavior_tree_example/src/master/]

Click the image below to play the video. This is a demo of the behavior tree. It can be run using the behavior_tree_example package.

[![Behavior Tree Example](http://img.youtube.com/vi/-xRF52uCHo8/0.jpg)](https://www.youtube.com/watch?v=-xRF52uCHo8 "Behavior Tree Example")

### Defining the Structure

The strucutre of the behavior tree is defined with a configuration file, an example of which is shown below:

```
->
	?
		(Example Condition)
		[Example Action]
```

The first node in the behavior tree above is a Sequence node indicated by a "->". Tabs are used to indicate a parent chlid relationship. The Sequence node has a child Fallback node, indicated by a "?". This Fallback node has two children, a condition node "(Example Condition)" and an action node "\[Example Node\]". Condition nodes must be surrounded by parenthesis, action nodes must be surrounded by square brackets. The text inside of the parenthesis/brackets can be anything. This is meant to correspond to how the are visualized in the graph.

Parallel nodes are declared with "|| 2", where 2 is the number of child nodes that need to return SUCCESS. Decorator nodes are declared with "<X>" where X is the type of decorator node, for example, a Not Decorator Node would be declared "<!>". Other behavior tree files can also be included using the syntax "include filename" where filename is an absolute path to a behavior tree file. This filename supports the $(find pacakge_name) syntax that roslaunch also supports. Examples use cases can be found in the behavior_tree_example package in examples 2 and 3.

### Implementing Conditions and Actions

The `behavior_tree_node.py` takes a configuration file and uses the condition and action nodes defined in it to determine what topics to subscribe and publish to. As mentioned in the Overview section above, a condition node must return SUCCESS or FAILURE and an action node must return SUCCESS, RUNNING, or FAILURE if it is active. The behavior tree node expects the status of conditions to be published to a topic that is the name of the label of the node, converted to lowercase, with spaces replaced with underscores, and "\_success" appended to it. In the example config file above, the status of the node defined by the "(Example Condition)" line would be listened to on a topic named "/example_condition_success". A std_msgs::Bool should be published to this topic by a ROS node implementing a condition, where true indicates success and false indicates failure. For action nodes, the behavior tree notifies a ROS node that the topic is active by publishing a std_msgs::Bool topic, the name of which is determined in a similar way to the condition node topic. For the node defined by the "[Example Action]" line in the config file above, the behavior tree will publish the std_msgs::Bool to a topic named "/example_action_active".  True indicates that the action is active, false indicates it is inactive. While the action is active, the behavior tree expects to receive a status from the node indicating SUCCESS, RUNNING, or FAILURE. For the node defined by the "[Example Action]" line, the behavior tree would subscribe to a behavior_tree_msgs::Status published on a topic named "/example_action_status". The Status message type contains one uint8, which should be one of the constants defined in the message, either behavior_tree_msgs::Status::SUCCESS, behavior_tree_msgs::Status::RUNNING, or behavior_tree_msgs::Status::FAILURE.

Writing a ROS node which implements an action or condition can be done by directly publishing and subscribing to the topics as described above. Alternatively, Condition and Action c++ classes that are defined in the behavior_tree library of this package can be used. The two methods are described below. The behavior_tree_example package, linked above has examples of both methods.

*Important Note:* In both cases, you need to publish the status of the action or condition at a sufficiently high frequency so that the behavior tree does not consider them to be timed out. The behavior tree has a ROS parameter called "timeout" which indicates the amount of time in seconds it can go without receiving a status from before it considers the action or conditions to be timed out.

#### Publishing and Subscribing directly

- Conditions

For conditions you only need to a publish a std_msgs::Bool.


First make sure you include the message type header:

```c++
#include <std_msgs/Bool.h>
```

Ddeclare the publisher somewhere in your code:

```c++
ros::Publisher condition_pub = node_handle.advertise<std_msgs::Bool>("/example_condition_success", 1);
```

Now simply publish to it at a sufficiently high frequency. This can be done, for example, in a ros::Timer callback or in a while loop with a ros::Rate.sleep() call:

```c++
std_msgs::Bool msg;
msg.data = true;
condition_pub.publish(msg);
```

- Actions

For actions you need to subscribe to the std_msgs::Bool from the behavior tree which indiates whether or not the action is active. When it is active you need to publish the status as a behavior_tree_msgs::Status at a sufficiently high frequency.

First make sure you include the message type headers:

```c++
#include <std_msgs/Bool.h>
#include <behavior_tree_msgs/Status.h>
```

Initialize the subscriber and publisher:

```c++
bool is_action_active = false;
ros::Subscriber action_active_sub = node_handle.subscribe("/example_action_active", 10, action_active_callback);
ros::Publisher action_status_pub = node_handle.advertise<behavior_tree_msgs::Status>("/example_action_status", 1);
```

In the subscriber's callback function, update the bool variable:

```c++
void action_active_callback(std_msgs::Bool msg){
	is_action_active = true;
}
```

Now check whether the action is active and if it is perform some task and publish it's status. Again, make sure the status is published at a sufficiently high rate.

```c++
if(is_action_active){
	behavior_tree_msgs::Status status;
	
	if( /* the action isn't done yet */ )
		status.status = behavior_tree_msgs::Status::RUNNING;
	else if( /* the actions is done and was successful */ )
		status.status = behavior_tree_msgs::Status::SUCCESS;
	else
		status.status = behavior_tree_msgs::Status::FAILURE;
	
	action_status_pub.publish(status);
}
```

#### Condition and Action Helper Classes

Instead of directly sending ROS messages, you can use Condition and Action helper classes that set up the publishers and subscribers for you. Make sure your CMakeLists.txt and package.xml depend on the behavior_tree package.

- Conditions

Include the library:

```c++
#include <behavior_tree/behavior_tree.h>
```

Initialize a condition:

```c++
bt::Condition condition(node_handle, "Example Condition");
```

The second argument to the Condition class's construcutor is the same text that the condition node is labled with in your config file. The class automatically creates a subscriber to the corresponding topic that the behavior_tree expects to receive the std_msgs::Bool on.

Now, set the value of the condition and publish it. As before, make sure to publish at a high enough frequency:

```c++
condition.set(true);
condition.publish();
```

You can also get the value of the condition using get(), which returns a bool:

```c++
bool success = condition.get();
```

- Actions

Include the library:

```c++
#include <behavior_tree/behavior_tree.h>
```

Initialize an action:

```c++
bt::Action action(node_handle, "Example Action");
```

Like the condition node, the Action constructor's second argument is the same text that is on the label of the action node in your config file. The class automatically creates a subscriber to the std_msgs::Bool topic that tells whether the action is active, and a publisher that publishes the std_msgs::String status. Make sure that the node handle you pass to the Action class has a callback queue that can handle receiving the std_msgs::Bool without being delayed too much by  other callbacks.

Now, if the action is active, set the status of the action and publish it. Make sure to publish at a high enough frequency.

```c++
if(action.is_active()){
	action.set_success();
	// or action.set_running();
	// or action.set_failure();

	action->publish();
}
```

You can also check the status you set the action to with the following functions which return bools:

```c++
action.is_success();
action.is_running();
action.is_failure();
```

### Debugging Tools

There are also some debugging tools for visualizing the tree and seeing how it behaves with different conditions and actions.

#### Image Publisher

The behavior tree publishes a graphviz string that can be made into an image. You can use the `behavior_tree_image_publisher.py` node to subscribe to the graphviz string and publish an image. This is done in the behavior_tree_example launch file.

#### RQT Debug GUI

There is an rqt GUI for debugging that your behavior tree works as you expect. Using the interface shown below, you can set the status published by each condition and action using the buttons. This way you don't have to debug the strucuture of your behavior tree by creating ROS nodes.

To use the plugin, make sure the workspace you build the behavior_tree packages is sourced (source devel/setup.bash) and run `rqt`. In the `Plugins` menu there should be a `Behavior Tree` menu containing the `Behavior Tree` plugin. When you open it for the first time, it won't have a config file loaded. Use the `Open Config...` button and select your behavior tree config file.

![picture](images/behavior_tree_rqt.png)


Author: John Keller slack: kellerj email: jkeller2@andrew.cmu.edu