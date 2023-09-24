# Week#4

- Overview:
    - **[Understanding nodes](https://www.notion.so/Week-4-34b065fbd3704f47b474e997e63b9823?pvs=21)**
    - **[Understanding topics](https://www.notion.so/Week-4-34b065fbd3704f47b474e997e63b9823?pvs=21)**
    - **[Understanding services](https://www.notion.so/Week-4-34b065fbd3704f47b474e997e63b9823?pvs=21)**
    - **[Understanding parameters](https://www.notion.so/Week-4-34b065fbd3704f47b474e997e63b9823?pvs=21)**
    - **[Understanding actions](https://www.notion.so/Week-4-34b065fbd3704f47b474e997e63b9823?pvs=21)**
    - **[Launching nodes](https://www.notion.so/Week-4-34b065fbd3704f47b474e997e63b9823?pvs=21)**
    - **[Recording and playing back data](https://www.notion.so/Week-4-34b065fbd3704f47b474e997e63b9823?pvs=21)**

## **[Nodes in ROS 2](https://docs.ros.org/en/humble/_images/Nodes-TopicandService.gif)**

Each node in ROS should be responsible for a single, modular purpose, e.g. controlling the wheel motors or publishing the sensor data from a laser range-finder. Each node can send and receive data from other nodes via topics, services, actions, or parameters.

![Nodes-TopicandService.gif](Week#4%2034b065fbd3704f47b474e997e63b9823/Nodes-TopicandService.gif)

### **[1 ros2 run](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html#id6)**

The command `ros2 run` launches an executable from a package.

`ros2 run <package_name> <executable_name>`

To run turtlesim, open a new terminal, and enter the following command:

`ros2 run turtlesim turtlesim_node`

The turtlesim window will open, as you saw in the [previous tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html).

Here, the package name is `turtlesim` and the executable name is `turtlesim_node`.

We still don’t know the node name, however. You can find node names by using `ros2 node list`

### **[2 ros2 node list](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html#id7)**

`ros2 node list` will show you the names of all running nodes. This is especially useful when you want to interact with a node, or when you have a system running many nodes and need to keep track of them.

Open a new terminal while turtlesim is still running in the other one, and enter the following command:

```jsx
ros2 node list
```

The terminal will return the node name:

`/turtlesim`

Open another new terminal and start the teleop node with the command:

```jsx
ros2 run turtlesim turtle_teleop_key
```

Here, we are referring to the `turtlesim` package again, but this time we target the executable named `turtle_teleop_key`.

Return to the terminal where you ran `ros2 node list` and run it again. You will now see the names of two active nodes:

```jsx
/turtlesim
/teleop_turtle
```

### **2.1 Remapping**

[Remapping](https://design.ros2.org/articles/ros_command_line_arguments.html#name-remapping-rules) allows you to reassign default node properties, like node name, topic names, service names, etc., to custom values. In the last tutorial, you used remapping on `turtle_teleop_key` to change the cmd_vel topic and target **turtle2**.

Now, let’s reassign the name of our `/turtlesim` node. In a new terminal, run the following command:

```jsx
ros2 run turtlesim turtlesim_node --ros-args --remap __node:=my_turtle
```

Since you’re calling `ros2 run` on turtlesim again, another turtlesim window will open. However, now if you return to the terminal where you ran `ros2 node list`, and run it again, you will see three node names:

```jsx
/my_turtle
/turtlesim
/teleop_turtle
```

### **[3 ros2 node info](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html#id8)**

Now that you know the names of your nodes, you can access more information about them with:

```jsx
ros2 node info <node_name>
```

To examine your latest node, `my_turtle`, run the following command:

```bash
ros2 node info /my_turtle
```

`ros2 node info` returns a list of subscribers, publishers, services, and actions. i.e. the ROS graph connections that interact with that node. The output should look like this:

```jsx
/my_turtle
  Subscribers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /turtle1/cmd_vel: geometry_msgs/msg/Twist
  Publishers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
    /turtle1/color_sensor: turtlesim/msg/Color
    /turtle1/pose: turtlesim/msg/Pose
  Service Servers:
    /clear: std_srvs/srv/Empty
    /kill: turtlesim/srv/Kill
    /my_turtle/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /my_turtle/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /my_turtle/get_parameters: rcl_interfaces/srv/GetParameters
    /my_turtle/list_parameters: rcl_interfaces/srv/ListParameters
    /my_turtle/set_parameters: rcl_interfaces/srv/SetParameters
    /my_turtle/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
    /reset: std_srvs/srv/Empty
    /spawn: turtlesim/srv/Spawn
    /turtle1/set_pen: turtlesim/srv/SetPen
    /turtle1/teleport_absolute: turtlesim/srv/TeleportAbsolute
    /turtle1/teleport_relative: turtlesim/srv/TeleportRelative
  Service Clients:

  Action Servers:
    /turtle1/rotate_absolute: turtlesim/action/RotateAbsolute
  Action Clients:
```

Now try running the same command on the `/teleop_turtle` node, and see how its connections differ from `my_turtle`.

## ****Understanding topics****

### **[1 Setup](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html#id4)**

By now you should be comfortable starting up turtlesim.

Open a new terminal and run:

```bash
ros2 run turtlesim turtlesim_node
```

Open another terminal and run:

```bash
ros2 run turtlesim turtle_teleop_key
```

Recall from the [previous tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html) that the names of these nodes are `/turtlesim` and `/teleop_turtle` by default.

### **[2 rqt_graph](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html#id5)**

Throughout this tutorial, we will use `rqt_graph` to visualize the changing nodes and topics, as well as the connections between them.

The [turtlesim tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html) tells you how to install rqt and all its plugins, including `rqt_graph`.

To run rqt_graph, open a new terminal and enter the command:

```bash
rqt_graph
```

You can also open rqt_graph by opening `rqt` and selecting **Plugins** > **Introspection** > **Node Graph**.

![https://docs.ros.org/en/humble/_images/rqt_graph.png](https://docs.ros.org/en/humble/_images/rqt_graph.png)

You should see the above nodes and topic, as well as two actions around the periphery of the graph (let’s ignore those for now). If you hover your mouse over the topic in the center, you’ll see the color highlighting like in the image above.

The graph is depicting how the `/turtlesim` node and the `/teleop_turtle` node are communicating with each other over a topic. The `/teleop_turtle` node is publishing data (the keystrokes you enter to move the turtle around) to the `/turtle1/cmd_vel` topic, and the `/turtlesim` node is subscribed to that topic to receive the data.

The highlighting feature of rqt_graph is very helpful when examining more complex systems with many nodes and topics connected in many different ways.

rqt_graph is a graphical introspection tool. Now we’ll look at some command line tools for introspecting topics.

### **[3 ros2 topic list](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html#id6)**

Running the `ros2 topic list` command in a new terminal will return a list of all the topics currently active in the system:

```bash
/parameter_events
/rosout
/turtle1/cmd_vel
/turtle1/color_sensor
/turtle1/pose
```

`ros2 topic list -t` will return the same list of topics, this time with the topic type appended in brackets:

```bash
/parameter_events [rcl_interfaces/msg/ParameterEvent]
/rosout [rcl_interfaces/msg/Log]
/turtle1/cmd_vel [geometry_msgs/msg/Twist]
/turtle1/color_sensor [turtlesim/msg/Color]
/turtle1/pose [turtlesim/msg/Pose]
```

These attributes, particularly the type, are how nodes know they’re talking about the same information as it moves over topics.

If you’re wondering where all these topics are in rqt_graph, you can uncheck all the boxes under **Hide:**

![https://docs.ros.org/en/humble/_images/unhide.png](https://docs.ros.org/en/humble/_images/unhide.png)

For now, though, leave those options checked to avoid confusion.

### **[4 ros2 topic echo](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html#id7)[](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html#ros2-topic-echo)**

To see the data being published on a topic, use:

```bash
ros2 topic echo <topic_name>
```

Since we know that `/teleop_turtle` publishes data to `/turtlesim` over the `/turtle1/cmd_vel` topic, let’s use `echo` to introspect that topic:

```bash
ros2 topic echo /turtle1/cmd_vel
```

At first, this command won’t return any data. That’s because it’s waiting for `/teleop_turtle` to publish something.

Return to the terminal where `turtle_teleop_key` is running and use the arrows to move the turtle around. Watch the terminal where your `echo` is running at the same time, and you’ll see position data being published for every movement you make:

```bash
linear:
  x: 2.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0
  ---
```

Now return to rqt_graph and uncheck the **Debug** box.

![https://docs.ros.org/en/humble/_images/debug.png](https://docs.ros.org/en/humble/_images/debug.png)

`/_ros2cli_26646` is the node created by the `echo` command we just ran (the number might be different). Now you can see that the publisher is publishing data over the `cmd_vel` topic, and two subscribers are subscribed to it.

### **[5 ros2 topic info](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html#id8)**

Topics don’t have to only be one-to-one communication; they can be one-to-many, many-to-one, or many-to-many.

Another way to look at this is running:

```bash
ros2 topic info /turtle1/cmd_vel
```

Which will return:

```bash
Type: geometry_msgs/msg/Twist
Publisher count: 1
Subscription count: 2
```

### **[6 ros2 interface show](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html#id9)[](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html#ros2-interface-show)**

Nodes send data over topics using messages. Publishers and subscribers must send and receive the same type of message to communicate.

The topic types we saw earlier after running `ros2 topic list -t` let us know what message type is used on each topic. Recall that the `cmd_vel` topic has the type:

```bash
geometry_msgs/msg/Twist
```

This means that in the package `geometry_msgs` there is a `msg` called `Twist`.

Now we can run `ros2 interface show <msg type>` on this type to learn its details. Specifically, what structure of data the message expects.

```bash
ros2 interface show geometry_msgs/msg/Twist
```

For the message type from above it yields:

```bash
**#** This expresses velocity **in** free space broken into its linear and angular parts.

    Vector3  linear
            float64 x
            float64 y
            float64 z
    Vector3  angular
            float64 x
            float64 y
            float64 z
```

This tells you that the `/turtlesim` node is expecting a message with two vectors, `linear` and `angular`, of three elements each. If you recall the data we saw `/teleop_turtle` passing to `/turtlesim` with the `echo` command, it’s in the same structure:

```bash
linear:
  x: 2.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0
  ---
```

### **[7 ros2 topic pub](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html#id10)**

Now that you have the message structure, you can publish data onto a topic directly from the command line using:

```bash
ros2 topic pub <topic_name> <msg_type> '<args>'
```

The `'<args>'` argument is the actual data you’ll pass to the topic, in the structure you just discovered in the previous section.

It’s important to note that this argument needs to be input in YAML syntax. Input the full command like so:

```bash
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
```

- `-once` is an optional argument meaning “publish one message then exit”.

You will see the following output in the terminal:

```bash
publisher: beginning loop
publishing #1: geometry_msgs.msg.Twist(linear=geometry_msgs.msg.Vector3(x=2.0, y=0.0, z=0.0), angular=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=1.8))
```

And you will see your turtle move like so:

![https://docs.ros.org/en/humble/_images/pub_once.png](https://docs.ros.org/en/humble/_images/pub_once.png)

The turtle (and commonly the real robots which it is meant to emulate) require a steady stream of commands to operate continuously. So, to get the turtle to keep moving, you can run:

```bash
ros2 topic pub --rate 1 /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
```

The difference here is the removal of the `--once` option and the addition of the `--rate 1` option, which tells `ros2 topic pub` to publish the command in a steady stream at 1 Hz.

![https://docs.ros.org/en/humble/_images/pub_stream.png](https://docs.ros.org/en/humble/_images/pub_stream.png)

You can refresh rqt_graph to see what’s happening graphically. You will see that the `ros2 topic pub ...` node (`/_ros2cli_30358`) is publishing over the `/turtle1/cmd_vel` topic, which is being received by both the `ros2 topic echo ...` node (`/_ros2cli_26646`) and the `/turtlesim` node now.

![https://docs.ros.org/en/humble/_images/rqt_graph2.png](https://docs.ros.org/en/humble/_images/rqt_graph2.png)

Finally, you can run `echo` on the `pose` topic and recheck rqt_graph:

```bash
ros2 topic echo /turtle1/pose
```

![https://docs.ros.org/en/humble/_images/rqt_graph3.png](https://docs.ros.org/en/humble/_images/rqt_graph3.png)

You can see that the `/turtlesim` node is also publishing to the `pose` topic, which the new `echo` node has subscribed to.

### **[8 ros2 topic hz](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html#id11)**

For one last introspection on this process, you can view the rate at which data is published using:

```bash
ros2 topic hz /turtle1/pose
```

It will return data on the rate at which the `/turtlesim` node is publishing data to the `pose` topic.

```bash
average rate: 59.354
  min: 0.005s max: 0.027s std dev: 0.00284s window: 58
```

Recall that you set the rate of `turtle1/cmd_vel` to publish at a steady 1 Hz using `ros2 topic pub --rate 1`. If you run the above command with `turtle1/cmd_vel` instead of `turtle1/pose`, you will see an average reflecting that rate.

## **[Understanding services](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html#id1)**

Services are another method of communication for nodes in the ROS graph. Services are based on a call-and-response model versus the publisher-subscriber model of topics. While topics allow nodes to subscribe to data streams and get continual updates, services only provide data when they are specifically called by a client.

![https://docs.ros.org/en/humble/_images/Service-SingleServiceClient.gif](https://docs.ros.org/en/humble/_images/Service-SingleServiceClient.gif)

![https://docs.ros.org/en/humble/_images/Service-MultipleServiceClient.gif](https://docs.ros.org/en/humble/_images/Service-MultipleServiceClient.gif)

### **[1 Setup](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html#id4)[](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html#setup)**

Start up the two turtlesim nodes, `/turtlesim` and `/teleop_turtle`.

Open a new terminal and run:

```bash
ros2 run turtlesim turtlesim_node
```

Open another terminal and run:

```bash
ros2 run turtlesim turtle_teleop_key
```

### **[2 ros2 service list](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html#id5)**

Running the `ros2 service list` command in a new terminal will return a list of all the services currently active in the system:

```bash
/clear
/kill
/reset
/spawn
/teleop_turtle/describe_parameters
/teleop_turtle/get_parameter_types
/teleop_turtle/get_parameters
/teleop_turtle/list_parameters
/teleop_turtle/set_parameters
/teleop_turtle/set_parameters_atomically
/turtle1/set_pen
/turtle1/teleport_absolute
/turtle1/teleport_relative
/turtlesim/describe_parameters
/turtlesim/get_parameter_types
/turtlesim/get_parameters
/turtlesim/list_parameters
/turtlesim/set_parameters
/turtlesim/set_parameters_atomically
```

You will see that both nodes have the same six services with `parameters` in their names. Nearly every node in ROS 2 has these infrastructure services that parameters are built off of. There will be more about parameters in the next tutorial. In this tutorial, the parameter services will be omitted from the discussion.

For now, let’s focus on the turtlesim-specific services, `/clear`, `/kill`, `/reset`, `/spawn`, `/turtle1/set_pen`, `/turtle1/teleport_absolute`, and `/turtle1/teleport_relative`. You may recall interacting with some of these services using rqt in the [Use turtlesim, ros2, and rqt](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html) tutorial.

### **[3 ros2 service type](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html#id6)**

Services have types that describe how the request and response data of a service is structured. Service types are defined similarly to topic types, except service types have two parts: one message for the request and another for the response.

To find out the type of a service, use the command:

```bash
ros2 service type <service_name>
```

Let’s take a look at turtlesim’s `/clear` service. In a new terminal, enter the command:

```bash
ros2 service type /clear
```

Which should return:

```bash
std_srvs/srv/Empty
```

The `Empty` type means the service call sends no data when making a request and receives no data when receiving a response.

### **3.1 ros2 service list -t[](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html#ros2-service-list-t)**

To see the types of all the active services at the same time, you can append the `--show-types` option, abbreviated as `-t`, to the `list` command:

```bash
ros2 service list -t
```

Which will return:

```bash
/clear [std_srvs/srv/Empty]
/kill [turtlesim/srv/Kill]
/reset [std_srvs/srv/Empty]
/spawn [turtlesim/srv/Spawn]
...
/turtle1/set_pen [turtlesim/srv/SetPen]
/turtle1/teleport_absolute [turtlesim/srv/TeleportAbsolute]
/turtle1/teleport_relative [turtlesim/srv/TeleportRelative]
...
```

### **[4 ros2 service find](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html#id7)**

If you want to find all the services of a specific type, you can use the command:

```bash
ros2 service find <type_name>
```

For example, you can find all the `Empty` typed services like this:

```bash
ros2 service find std_srvs/srv/Empty
```

Which will return:

```bash
/clear
/reset
```

### **[5 ros2 interface show](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html#id8)**

You can call services from the command line, but first you need to know the structure of the input arguments.

```bash
ros2 interface show <type_name>
```

Try this on the `/clear` service’s type, `Empty`:

```bash
ros2 interface show std_srvs/srv/Empty
```

Which will return:

```bash
--
```

The `---` separates the request structure (above) from the response structure (below). But, as you learned earlier, the `Empty` type doesn’t send or receive any data. So, naturally, its structure is blank.

Let’s introspect a service with a type that sends and receives data, like `/spawn`. From the results of `ros2 service list -t`, we know `/spawn`’s type is `turtlesim/srv/Spawn`.

To see the request and response arguments of the `/spawn` service, run the command:

```bash
ros2 interface show turtlesim/srv/Spawn
```

Which will return:

```bash
float32 x
float32 y
float32 theta
string name # Optional.  A unique name will be created and returned if this is empty
---
string name
```

The information above the `---` line tells us the arguments needed to call `/spawn`. `x`, `y` and `theta` determine the 2D pose of the spawned turtle, and `name` is clearly optional.

The information below the line isn’t something you need to know in this case, but it can help you understand the data type of the response you get from the call.

### **[6 ros2 service call](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html#id9)**

Now that you know what a service type is, how to find a service’s type, and how to find the structure of that type’s arguments, you can call a service using:

```bash
ros2 service call <service_name> <service_type> <arguments>
```

The `<arguments>` part is optional. For example, you know that `Empty` typed services don’t have any arguments:

```bash
ros2 service call /clear std_srvs/srv/Empty
```

This command will clear the turtlesim window of any lines your turtle has drawn.

![https://docs.ros.org/en/humble/_images/clear.png](https://docs.ros.org/en/humble/_images/clear.png)

Now let’s spawn a new turtle by calling `/spawn` and setting arguments. Input `<arguments>` in a service call from the command-line need to be in YAML syntax.

Enter the command:

```bash
ros2 service call /spawn turtlesim/srv/Spawn "{x: 2, y: 2, theta: 0.2, name: ''}"
```

You will get this method-style view of what’s happening, and then the service response:

```bash
requester: making request: turtlesim.srv.Spawn_Request(x=2.0, y=2.0, theta=0.2, name='')

response:
turtlesim.srv.Spawn_Response(name='turtle2')
```

Your turtlesim window will update with the newly spawned turtle right away:

![https://docs.ros.org/en/humble/_images/spawn1.png](https://docs.ros.org/en/humble/_images/spawn1.png)

## **[Understanding parameters](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html#understanding-parameters)**

A parameter is a configuration value of a node. You can think of parameters as node settings. A node can store parameters as integers, floats, booleans, strings, and lists. In ROS 2, each node maintains its own parameters. For more background on parameters, please see [the concept document](https://docs.ros.org/en/humble/Concepts/Basic/About-Parameters.html).

### **[1 Setup](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html#id4)**

Start up the two turtlesim nodes, `/turtlesim` and `/teleop_turtle`.

Open a new terminal and run:

```bash
ros2 run turtlesim turtlesim_node
```

Open another terminal and run:

```bash
ros2 run turtlesim turtle_teleop_key
```

### **[2 ros2 param list](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html#id5)**

To see the parameters belonging to your nodes, open a new terminal and enter the command:

```bash
ros2 param list
```

You will see the node namespaces, `/teleop_turtle` and `/turtlesim`, followed by each node’s parameters:

```bash
/teleop_turtle:
  qos_overrides./parameter_events.publisher.depth
  qos_overrides./parameter_events.publisher.durability
  qos_overrides./parameter_events.publisher.history
  qos_overrides./parameter_events.publisher.reliability
  scale_angular
  scale_linear
  use_sim_time
/turtlesim:
  background_b
  background_g
  background_r
  qos_overrides./parameter_events.publisher.depth
  qos_overrides./parameter_events.publisher.durability
  qos_overrides./parameter_events.publisher.history
  qos_overrides./parameter_events.publisher.reliability
  use_sim_time
```

Every node has the parameter `use_sim_time`; it’s not unique to turtlesim.

Based on their names, it looks like `/turtlesim`’s parameters determine the background color of the turtlesim window using RGB color values.

To determine a parameter’s type, you can use `ros2 param get`.

### **[3 ros2 param get](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html#id6)**

To display the type and current value of a parameter, use the command:

```bash
ros2 param get <node_name> <parameter_name>
```

Let’s find out the current value of `/turtlesim`’s parameter `background_g`:

```bash
ros2 param get /turtlesim background_g
```

Which will return the value:

```bash
Integer value is: 86
```

Now you know `background_g` holds an integer value.

If you run the same command on `background_r` and `background_b`, you will get the values `69` and `255`, respectively.

### **[4 ros2 param set](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html#id7)**

To change a parameter’s value at runtime, use the command:

```bash
ros2 param set <node_name> <parameter_name> <value>
```

Let’s change `/turtlesim`’s background color:

```bash
ros2 param set /turtlesim background_r 150
```

Your terminal should return the message:

```bash
Set parameter successful
```

And the background of your turtlesim window should change colors:

![https://docs.ros.org/en/humble/_images/set.png](https://docs.ros.org/en/humble/_images/set.png)

Setting parameters with the `set` command will only change them in your current session, not permanently. However, you can save your settings and reload them the next time you start a node.

### **[5 ros2 param dump](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html#id8)**

You can view all of a node’s current parameter values by using the command:

```bash
ros2 param dump <node_name>
```

The command prints to the standard output (stdout) by default but you can also redirect the parameter values into a file to save them for later. To save your current configuration of `/turtlesim`’s parameters into the file `turtlesim.yaml`, enter the command:

```bash
ros2 param dump /turtlesim > turtlesim.yaml
```

You will find a new file in the current working directory your shell is running in. If you open this file, you’ll see the following content:

```bash
**/turtlesim**:
  **ros__parameters**:
    **background_b**: 255
    **background_g**: 86
    **background_r**: 150
    **qos_overrides**:
      **/parameter_events**:
        **publisher**:
          **depth**: 1000
          **durability**: volatile
          **history**: keep_last
          **reliability**: reliable
    **use_sim_time**: false
```

Dumping parameters comes in handy if you want to reload the node with the same parameters in the future.

### **[6 ros2 param load](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html#id9)**

You can load parameters from a file to a currently running node using the command:

```bash
ros2 param load <node_name> <parameter_file>
```

To load the `turtlesim.yaml` file generated with `ros2 param dump` into `/turtlesim` node’s parameters, enter the command:

```bash
ros2 param load /turtlesim turtlesim.yaml
```

Your terminal will return the message:

```bash
Set parameter background_b successful
Set parameter background_g successful
Set parameter background_r successful
Set parameter qos_overrides./parameter_events.publisher.depth failed: parameter 'qos_overrides./parameter_events.publisher.depth' cannot be set because it is read-only
Set parameter qos_overrides./parameter_events.publisher.durability failed: parameter 'qos_overrides./parameter_events.publisher.durability' cannot be set because it is read-only
Set parameter qos_overrides./parameter_events.publisher.history failed: parameter 'qos_overrides./parameter_events.publisher.history' cannot be set because it is read-only
Set parameter qos_overrides./parameter_events.publisher.reliability failed: parameter 'qos_overrides./parameter_events.publisher.reliability' cannot be set because it is read-only
Set parameter use_sim_time successful
```

**Note**

Read-only parameters can only be modified at startup and not afterwards, that is why there are some warnings for the “qos_overrides” parameters.

### **[7 Load parameter file on node startup](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html#id10)**

To start the same node using your saved parameter values, use:

```bash
ros2 run <package_name> <executable_name> --ros-args --params-file <file_name>
```

This is the same command you always use to start turtlesim, with the added flags `--ros-args` and `--params-file`, followed by the file you want to load.

Stop your running turtlesim node, and try reloading it with your saved parameters, using:

```bash
ros2 run turtlesim turtlesim_node --ros-args --params-file turtlesim.yaml
```

The turtlesim window should appear as usual, but with the purple background you set earlier.

**Note**

When a parameter file is used at node startup, all parameters, including the read-only ones, will be updated.

## **[Understanding actions](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html#understanding-actions)**

Actions are one of the communication types in ROS 2 and are intended for long running tasks. They consist of three parts: a goal, feedback, and a result.

Actions are built on topics and services. Their functionality is similar to services, except actions can be canceled. They also provide steady feedback, as opposed to services which return a single response.

Actions use a client-server model, similar to the publisher-subscriber model (described in the [topics tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html)). An “action client” node sends a goal to an “action server” node that acknowledges the goal and returns a stream of feedback and a result.

![https://docs.ros.org/en/humble/_images/Action-SingleActionClient.gif](https://docs.ros.org/en/humble/_images/Action-SingleActionClient.gif)

### **[1 Setup](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html#id4)**

Start up the two turtlesim nodes, `/turtlesim` and `/teleop_turtle`.

Open a new terminal and run:

```bash
ros2 run turtlesim turtlesim_node
```

Open another terminal and run:

```bash
ros2 run turtlesim turtle_teleop_key
```

### **[2 Use actions](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html#id5)**

When you launch the `/teleop_turtle` node, you will see the following message in your terminal:

```bash
Use arrow keys to move the turtle.
Use G|B|V|C|D|E|R|T keys to rotate to absolute orientations. 'F' to cancel a rotation.
```

Let’s focus on the second line, which corresponds to an action. (The first instruction corresponds to the “cmd_vel” topic, discussed previously in the [topics tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html).)

Notice that the letter keys `G|B|V|C|D|E|R|T` form a “box” around the `F` key on a US QWERTY keyboard (if you are not using a QWERTY keyboard, see [this link](https://upload.wikimedia.org/wikipedia/commons/d/da/KB_United_States.svg) to follow along). Each key’s position around `F` corresponds to that orientation in turtlesim. For example, the `E` will rotate the turtle’s orientation to the upper left corner.

Pay attention to the terminal where the `/turtlesim` node is running. Each time you press one of these keys, you are sending a goal to an action server that is part of the `/turtlesim` node. The goal is to rotate the turtle to face a particular direction. A message relaying the result of the goal should display once the turtle completes its rotation:

```bash
[INFO] [turtlesim]: Rotation goal completed successfully
```

The `F` key will cancel a goal mid-execution.

Try pressing the `C` key, and then pressing the `F` key before the turtle can complete its rotation. In the terminal where the `/turtlesim` node is running, you will see the message:

```bash
[INFO] [turtlesim]: Rotation goal canceled
```

Not only can the client-side (your input in the teleop) stop a goal, but the server-side (the `/turtlesim` node) can as well. When the server-side chooses to stop processing a goal, it is said to “abort” the goal.

Try hitting the `D` key, then the `G` key before the first rotation can complete. In the terminal where the `/turtlesim` node is running, you will see the message:

```bash
[WARN] [turtlesim]: Rotation goal received before a previous goal finished. Aborting previous goal
```

This action server chose to abort the first goal because it got a new one. It could have chosen something else, like reject the new goal or execute the second goal after the first one finished. Don’t assume every action server will choose to abort the current goal when it gets a new one.

### **[3 ros2 node info](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html#id6)**

To see the list of actions a node provides, `/turtlesim` in this case, open a new terminal and run the command:

```bash
ros2 node info /turtlesim
```

Which will return a list of `/turtlesim`’s subscribers, publishers, services, action servers and action clients:

```bash
/turtlesim
  Subscribers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /turtle1/cmd_vel: geometry_msgs/msg/Twist
  Publishers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
    /turtle1/color_sensor: turtlesim/msg/Color
    /turtle1/pose: turtlesim/msg/Pose
  Service Servers:
    /clear: std_srvs/srv/Empty
    /kill: turtlesim/srv/Kill
    /reset: std_srvs/srv/Empty
    /spawn: turtlesim/srv/Spawn
    /turtle1/set_pen: turtlesim/srv/SetPen
    /turtle1/teleport_absolute: turtlesim/srv/TeleportAbsolute
    /turtle1/teleport_relative: turtlesim/srv/TeleportRelative
    /turtlesim/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /turtlesim/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /turtlesim/get_parameters: rcl_interfaces/srv/GetParameters
    /turtlesim/list_parameters: rcl_interfaces/srv/ListParameters
    /turtlesim/set_parameters: rcl_interfaces/srv/SetParameters
    /turtlesim/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
  Service Clients:

  Action Servers:
    /turtle1/rotate_absolute: turtlesim/action/RotateAbsolute
  Action Clients:
```

Notice that the `/turtle1/rotate_absolute` action for `/turtlesim` is under `Action Servers`. This means `/turtlesim` responds to and provides feedback for the `/turtle1/rotate_absolute` action.

The `/teleop_turtle` node has the name `/turtle1/rotate_absolute` under `Action Clients` meaning that it sends goals for that action name. To see that, run:

```bash
ros2 node info /teleop_turtle
```

Which will return:

```bash
/teleop_turtle
  Subscribers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
  Publishers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
    /turtle1/cmd_vel: geometry_msgs/msg/Twist
  Service Servers:
    /teleop_turtle/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /teleop_turtle/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /teleop_turtle/get_parameters: rcl_interfaces/srv/GetParameters
    /teleop_turtle/list_parameters: rcl_interfaces/srv/ListParameters
    /teleop_turtle/set_parameters: rcl_interfaces/srv/SetParameters
    /teleop_turtle/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
  Service Clients:

  Action Servers:

  Action Clients:
    /turtle1/rotate_absolute: turtlesim/action/RotateAbsolute
```

### **[4 ros2 action list](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html#id7)**

To identify all the actions in the ROS graph, run the command:

```bash
ros2 action list
```

Which will return:

```bash
/turtle1/rotate_absolute
```

This is the only action in the ROS graph right now. It controls the turtle’s rotation, as you saw earlier. You also already know that there is one action client (part of `/teleop_turtle`) and one action server (part of `/turtlesim`) for this action from using the `ros2 node info <node_name>` command.

### **4.1 ros2 action list -t**

Actions have types, similar to topics and services. To find `/turtle1/rotate_absolute`’s type, run the command:

```bash
ros2 action list -t
```

Which will return:

```bash
/turtle1/rotate_absolute [turtlesim/action/RotateAbsolute]
```

In brackets to the right of each action name (in this case only `/turtle1/rotate_absolute`) is the action type, `turtlesim/action/RotateAbsolute`. You will need this when you want to execute an action from the command line or from code.

### **[5 ros2 action info](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html#id8)**

You can further introspect the `/turtle1/rotate_absolute` action with the command:

```bash
ros2 action info /turtle1/rotate_absolute
```

Which will return

```bash
Action: /turtle1/rotate_absolute
Action clients: 1
    /teleop_turtle
Action servers: 1
    /turtlesim
```

This tells us what we learned earlier from running `ros2 node info` on each node: The `/teleop_turtle` node has an action client and the `/turtlesim` node has an action server for the `/turtle1/rotate_absolute` action.

### **[6 ros2 interface show](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html#id9)[](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html#ros2-interface-show)**

One more piece of information you will need before sending or executing an action goal yourself is the structure of the action type.

Recall that you identified `/turtle1/rotate_absolute`’s type when running the command `ros2 action list -t`. Enter the following command with the action type in your terminal:

```bash
ros2 interface show turtlesim/action/RotateAbsolute
```

Which will return:

```bash
**#** The desired heading **in** radians
float32 theta
---
**#** The angular displacement **in** radians to the starting position
float32 delta
---
**#** The remaining rotation **in** radians
float32 remaining
```

The section of this message above the first `---` is the structure (data type and name) of the goal request. The next section is the structure of the result. The last section is the structure of the feedback.

### **[7 ros2 action send_goal](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html#id10)**

Now let’s send an action goal from the command line with the following syntax:

```bash
ros2 action send_goal <action_name> <action_type> <values>
```

`<values>` need to be in YAML format.

Keep an eye on the turtlesim window, and enter the following command into your terminal:

```bash
ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: 1.57}"
```

You should see the turtle rotating, as well as the following message in your terminal:

```bash
Waiting for an action server to become available...
Sending goal:
   theta: 1.57

Goal accepted with ID: f8db8f44410849eaa93d3feb747dd444

Result:
  delta: -1.568000316619873

Goal finished with status: SUCCEEDED
```

All goals have a unique ID, shown in the return message. You can also see the result, a field with the name `delta`, which is the displacement to the starting position.

To see the feedback of this goal, add `--feedback` to the `ros2 action send_goal` command:

```bash
ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: -1.57}" --feedback
```

Your terminal will return the message:

```bash
Sending goal:
   theta: -1.57

Goal accepted with ID: e6092c831f994afda92f0086f220da27

Feedback:
  remaining: -3.1268222332000732

Feedback:
  remaining: -3.1108222007751465

…

Result:
  delta: 3.1200008392333984

Goal finished with status: SUCCEEDED
```

You will continue to receive feedback, the remaining radians, until the goal is complete.

## **[Launching nodes](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Launching-Multiple-Nodes/Launching-Multiple-Nodes.html#launching-nodes)**

<aside>
🧠 In most of the introductory tutorials, you have been opening new terminals for every new node you run. As you create more complex systems with more and more nodes running simultaneously, opening terminals and reentering configuration details becomes tedious.

Launch files allow you to start up and configure a number of executables containing ROS 2 nodes simultaneously.

Running a single launch file with the `ros2 launch` command will start up your entire system - all nodes and their configurations - at once.

</aside>

### **[Running a Launch File](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Launching-Multiple-Nodes/Launching-Multiple-Nodes.html#id4)**

Open a new terminal and run:

```bash
ros2 launch turtlesim multisim.launch.py
```

This command will run the following launch file:

```bash
*# turtlesim/launch/multisim.launch.py***from** **launch** **import** LaunchDescription
**import** **launch_ros.actionsdef** generate_launch_description():
    **return** LaunchDescription([
        launch_ros.actions.Node(
            namespace= "turtlesim1", package='turtlesim', executable='turtlesim_node', output='screen'),
        launch_ros.actions.Node(
            namespace= "turtlesim2", package='turtlesim', executable='turtlesim_node', output='screen'),
    ])
```

**Note**

The launch file above is written in Python, but you can also use XML and YAML to create launch files. You can see a comparison of these different ROS 2 launch formats in [Using Python, XML, and YAML for ROS 2 Launch Files](https://docs.ros.org/en/humble/How-To-Guides/Launch-file-different-formats.html).

This will run two turtlesim nodes:

![https://docs.ros.org/en/humble/_images/turtlesim_multisim.png](https://docs.ros.org/en/humble/_images/turtlesim_multisim.png)

For now, don’t worry about the contents of this launch file. You can find more information on ROS 2 launch in the [ROS 2 launch tutorials](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-Main.html).

### **[(Optional) Control the Turtlesim Nodes](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Launching-Multiple-Nodes/Launching-Multiple-Nodes.html#id5)**

Now that these nodes are running, you can control them like any other ROS 2 nodes. For example, you can make the turtles drive in opposite directions by opening up two additional terminals and running the following commands:

In the second terminal:

```bash
ros2 topic pub  /turtlesim1/turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
```

In the third terminal:

```bash
ros2 topic pub  /turtlesim2/turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -1.8}}"
```

After running these commands, you should see something like the following:

![https://docs.ros.org/en/humble/_images/turtlesim_multisim_spin.png](https://docs.ros.org/en/humble/_images/turtlesim_multisim_spin.png)

## **[Recording and playing back data](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html#recording-and-playing-back-data)**

`ros2 bag` is a command line tool for recording data published on topics in your system. It accumulates the data passed on any number of topics and saves it in a database. You can then replay the data to reproduce the results of your tests and experiments. Recording topics is also a great way to share your work and allow others to recreate it.

### **[1 Setup](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html#id4)**

You’ll be recording your keyboard input in the `turtlesim` system to save and replay later on, so begin by starting up the `/turtlesim` and `/teleop_turtle` nodes.

Open a new terminal and run:

```bash
ros2 run turtlesim turtlesim_node
```

Open another terminal and run:

```bash
ros2 run turtlesim turtle_teleop_key
```

Let’s also make a new directory to store our saved recordings, just as good practice:

```bash
mkdir bag_files
cd bag_files
```

### **[2 Choose a topic](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html#id5)**

`ros2 bag` can only record data from published messages in topics. To see the list of your system’s topics, open a new terminal and run the command:

```bash
ros2 topic list
```

Which will return:

```bash
/parameter_events
/rosout
/turtle1/cmd_vel
/turtle1/color_sensor
/turtle1/pose
```

In the topics tutorial, you learned that the `/turtle_teleop` node publishes commands on the `/turtle1/cmd_vel` topic to make the turtle move in turtlesim.

To see the data that `/turtle1/cmd_vel` is publishing, run the command:

```bash
ros2 topic echo /turtle1/cmd_vel
```

Nothing will show up at first because no data is being published by the teleop. Return to the terminal where you ran the teleop and select it so it’s active. Use the arrow keys to move the turtle around, and you will see data being published on the terminal running `ros2 topic echo`.

```bash
linear:
  x: 2.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0
  ---
```

### **[3 ros2 bag record](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html#id6)**

To record the data published to a topic use the command syntax:

```bash
ros2 bag record <topic_name>
```

Before running this command on your chosen topic, open a new terminal and move into the `bag_files` directory you created earlier, because the rosbag file will save in the directory where you run it.

Run the command:

```bash
ros2 bag record /turtle1/cmd_vel
```

You will see the following messages in the terminal (the date and time will be different):

```bash
[INFO] [rosbag2_storage]: Opened database 'rosbag2_2019_10_11-05_18_45'.
[INFO] [rosbag2_transport]: Listening for topics...
[INFO] [rosbag2_transport]: Subscribed to topic '/turtle1/cmd_vel'
[INFO] [rosbag2_transport]: All requested topics are subscribed. Stopping discovery...
```

Now `ros2 bag` is recording the data published on the `/turtle1/cmd_vel` topic. Return to the teleop terminal and move the turtle around again. The movements don’t matter, but try to make a recognizable pattern to see when you replay the data later.

![https://docs.ros.org/en/humble/_images/record.png](https://docs.ros.org/en/humble/_images/record.png)

Press `Ctrl+C` to stop recording.

The data will be accumulated in a new bag directory with a name in the pattern of `rosbag2_year_month_day-hour_minute_second`. This directory will contain a `metadata.yaml` along with the bag file in the recorded format.

### **3.1 Record multiple topics**

You can also record multiple topics, as well as change the name of the file `ros2 bag` saves to.

Run the following command:

```bash
ros2 bag record -o subset /turtle1/cmd_vel /turtle1/pose
```

The `-o` option allows you to choose a unique name for your bag file. The following string, in this case `subset`, is the file name.

To record more than one topic at a time, simply list each topic separated by a space.

You will see the following message, confirming that both topics are being recorded.

```bash
[INFO] [rosbag2_storage]: Opened database 'subset'.
[INFO] [rosbag2_transport]: Listening for topics...
[INFO] [rosbag2_transport]: Subscribed to topic '/turtle1/cmd_vel'
[INFO] [rosbag2_transport]: Subscribed to topic '/turtle1/pose'
[INFO] [rosbag2_transport]: All requested topics are subscribed. Stopping discovery...
```

You can move the turtle around and press `Ctrl+C` when you’re finished.

**Note**

There is another option you can add to the command, `-a`, which records all the topics on your system.

### **[4 ros2 bag info](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html#id7)**

You can see details about your recording by running:

```bash
ros2 bag info <bag_file_name>
```

Running this command on the `subset` bag file will return a list of information on the file:

```bash
ros2 bag info subset
```

```bash
Files:             subset.db3
Bag size:          228.5 KiB
Storage id:        sqlite3
Duration:          48.47s
Start:             Oct 11 2019 06:09:09.12 (1570799349.12)
End                Oct 11 2019 06:09:57.60 (1570799397.60)
Messages:          3013
Topic information: Topic: /turtle1/cmd_vel | Type: geometry_msgs/msg/Twist | Count: 9 | Serialization Format: cdr
                 Topic: /turtle1/pose | Type: turtlesim/msg/Pose | Count: 3004 | Serialization Format: cdr
```

To view the individual messages, you would have to open up the database, in this case sqlite3, to examine it, which is beyond the scope of ROS 2.

### **[5 ros2 bag play](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html#id8)**

Before replaying the bag file, enter `Ctrl+C` in the terminal where the teleop is running. Then make sure your turtlesim window is visible so you can see the bag file in action.

Enter the command:

```bash
ros2 bag play subset
```

The terminal will return the message:

```bash
[INFO] [rosbag2_storage]: Opened database 'subset'.
```

Your turtle will follow the same path you entered while recording (though not 100% exactly; turtlesim is sensitive to small changes in the system’s timing).

![https://docs.ros.org/en/humble/_images/playback.png](https://docs.ros.org/en/humble/_images/playback.png)

Because the `subset` file recorded the `/turtle1/pose` topic, the `ros2 bag play` command won’t quit for as long as you had turtlesim running, even if you weren’t moving.

This is because as long as the `/turtlesim` node is active, it publishes data on the `/turtle1/pose` topic at regular intervals. You may have noticed in the `ros2 bag info` example result above that the `/turtle1/cmd_vel` topic’s `Count` information was only 9; that’s how many times we pressed the arrow keys while recording.

Notice that `/turtle1/pose` has a `Count` value of over 3000; while we were recording, data was published on that topic 3000 times.

To get an idea of how often position data is published, you can run the command:

```bash
ros2 topic hz /turtle1/pose
```
