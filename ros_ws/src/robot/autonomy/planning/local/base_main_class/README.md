This package contains the BaseNode and BaseNodelet classes which monitor the status of the process. All nodes and nodelets you create should inherit from these classes.

For both nodes and nodelets three functions need to be implemented by derived classes:

initialize(): This is where you should set up ROS related objects like publishers and subscribers and check for ROS parameters.

execute(): This is where your main program logic should go. It gets called in a loop at a rate set by the execute_target parameter in your node’s namespace.

~Destructor(): Do cleanup, like memory deallocation etc., in the destructor.

For nodes, you also need to implement the BaseNode::get() function to return a pointer to an instance of your class derived from BaseNode. This is used by the main function to run your node. For nodelets, this is not necessary. They are started automatically.

This package also contains a HealthMonitor class. This lets you time code's running time by calling tic("name") then toc("name"). When tic("name") is called for the first time, a ROS parameter with the name "name_target" is looked up which indicates what the target update rate for the code between tic and toc is. For now, a message is printed if the target rate is not achieved.

An example implementation of a BaseNode can be found in the example_node package.
An example implementation of a BaseNodelet can be found in the example_nodelet package.

Author: John Keller jkeller2@andrew.cmu.edu slack: kellerj
