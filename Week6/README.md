# Week6

# **[Tasks](https://docs.ros.org/en/humble/Tutorials/Advanced/Topic-Statistics-Tutorial/Topic-Statistics-Tutorial.html#id3)**

### **[1 Write the subscriber node with statistics enabled](https://docs.ros.org/en/humble/Tutorials/Advanced/Topic-Statistics-Tutorial/Topic-Statistics-Tutorial.html#id4)[](https://docs.ros.org/en/humble/Tutorials/Advanced/Topic-Statistics-Tutorial/Topic-Statistics-Tutorial.html#write-the-subscriber-node-with-statistics-enabled)**

Navigate into the `ros2_ws/src/cpp_pubsub/src` folder, created in the [previous tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html), and download the example talker code by entering the following command:

**Linux**macOSWindows

`wget -O member_function_with_topic_statistics.cpp https://raw.githubusercontent.com/ros2/examples/humble/rclcpp/topics/minimal_subscriber/member_function_with_topic_statistics.cpp`

Now there will be a new file named `member_function_with_topic_statistics.cpp`. Open the file using your preferred text editor.

`#include *<chrono>*#include *<memory>*#include *"rclcpp/rclcpp.hpp"*#include *"rclcpp/subscription_options.hpp"*#include *"std_msgs/msg/string.hpp"***class** **MinimalSubscriberWithTopicStatistics** : **public** rclcpp::Node
{
**public**:
  MinimalSubscriberWithTopicStatistics()
  : Node("minimal_subscriber_with_topic_statistics")
  {
    *// manually enable topic statistics via options*    **auto** options = rclcpp::SubscriptionOptions();
    options.topic_stats_options.state = rclcpp::TopicStatisticsState::Enable;

    *// configure the collection window and publish period (default 1s)*    options.topic_stats_options.publish_period = std::chrono::seconds(10);

    *// configure the topic name (default '/statistics')*    *// options.topic_stats_options.publish_topic = "/topic_statistics"*    **auto** callback = [**this**](std_msgs::msg::String::SharedPtr msg) {
        **this**->topic_callback(msg);
      };

    subscription_ = **this**->create_subscription<std_msgs::msg::String>(
      "topic", 10, callback, options);
  }

**private**:
  void topic_callback(**const** std_msgs::msg::String::ConstSharedPtr msg) **const**  {
    RCLCPP_INFO(**this**->get_logger(), "I heard: '%s'", msg->data.c_str());
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriberWithTopicStatistics>());
  rclcpp::shutdown();
  **return** 0;
}`

### **[1.1 Examine the code](https://docs.ros.org/en/humble/Tutorials/Advanced/Topic-Statistics-Tutorial/Topic-Statistics-Tutorial.html#id5)[](https://docs.ros.org/en/humble/Tutorials/Advanced/Topic-Statistics-Tutorial/Topic-Statistics-Tutorial.html#examine-the-code)**

As in the [C++](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html) tutorial, we have a subscriber node which receives string messages from the `topic` topic from the `topic_callback` function. However, we’ve now added options to configure the subscription to enable topic statistics with the `rclcpp::SubscriptionOptions()` options struct.

*`// manually enable topic statistics via options***auto** options = rclcpp::SubscriptionOptions();
options.topic_stats_options.state = rclcpp::TopicStatisticsState::Enable;`

Optionally, fields such as the statistics collection/publish period and the topic used to publish statistics can be configured as well.

*`// configure the collection window and publish period (default 1s)*options.topic_stats_options.publish_period = std::chrono::seconds(10);

*// configure the topic name (default '/statistics')// options.topic_stats_options.publish_topic = "/my_topic"*`

The configurable fields are described in the following table:

| Subscription Config Field | Purpose |
| --- | --- |
| topic_stats_options.state | Enable or disable topic statistics (default rclcpp::TopicStatisticsState::Disable) |
| topic_stats_options.publish_period | The period in which to collect statistics data and publish a statistics message (default 1s) |
| topic_stats_options.publish_topic | The topic to use when publishing statistics data (default /statistics) |

### **[1.2 CMakeLists.txt](https://docs.ros.org/en/humble/Tutorials/Advanced/Topic-Statistics-Tutorial/Topic-Statistics-Tutorial.html#id6)[](https://docs.ros.org/en/humble/Tutorials/Advanced/Topic-Statistics-Tutorial/Topic-Statistics-Tutorial.html#cmakelists-txt)**

Now open the `CMakeLists.txt` file.

Add the executable and name it `listener_with_topic_statistics` so you can run your node using `ros2 run`:

`add_executable(listener_with_topic_statistics src/member_function_with_topic_statistics.cpp)
ament_target_dependencies(listener_with_topic_statistics rclcpp std_msgs)

install(TARGETS
  talker
  listener
  listener_with_topic_statistics
  DESTINATION lib/${PROJECT_NAME})`

Make sure to save the file, and then your pub/sub system, with topic statistics enabled, should be ready for use.

### **[2 Build and run](https://docs.ros.org/en/humble/Tutorials/Advanced/Topic-Statistics-Tutorial/Topic-Statistics-Tutorial.html#id7)[](https://docs.ros.org/en/humble/Tutorials/Advanced/Topic-Statistics-Tutorial/Topic-Statistics-Tutorial.html#build-and-run)**

To build, see the [Build and run](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html#cpppubsub-build-and-run) section in the pub/sub tutorial.

Run the subscriber with statistics enabled node:

`ros2 run cpp_pubsub listener_with_topic_statistics`

Now run the talker node:

`ros2 run cpp_pubsub talker`

The terminal should start publishing info messages every 0.5 seconds, like so:

`[INFO] [minimal_publisher]: Publishing: "Hello World: 0"
[INFO] [minimal_publisher]: Publishing: "Hello World: 1"
[INFO] [minimal_publisher]: Publishing: "Hello World: 2"
[INFO] [minimal_publisher]: Publishing: "Hello World: 3"
[INFO] [minimal_publisher]: Publishing: "Hello World: 4"`

The listener will start printing messages to the console, starting at whatever message count the publisher is on at that time, like so:

`[INFO] [minimal_subscriber_with_topic_statistics]: I heard: "Hello World: 10"
[INFO] [minimal_subscriber_with_topic_statistics]: I heard: "Hello World: 11"
[INFO] [minimal_subscriber_with_topic_statistics]: I heard: "Hello World: 12"
[INFO] [minimal_subscriber_with_topic_statistics]: I heard: "Hello World: 13"
[INFO] [minimal_subscriber_with_topic_statistics]: I heard: "Hello World: 14"`

Now that the subscriber node is receiving messages, it will periodically publish statistics messages. We will observe these messages in the next section.

### **[3 Observe published statistic data](https://docs.ros.org/en/humble/Tutorials/Advanced/Topic-Statistics-Tutorial/Topic-Statistics-Tutorial.html#id8)[](https://docs.ros.org/en/humble/Tutorials/Advanced/Topic-Statistics-Tutorial/Topic-Statistics-Tutorial.html#observe-published-statistic-data)**

While the nodes are running, open a new terminal window. Execute the following command:

`ros2 topic list`

This will list all currently active topics. You should see the following:

`/parameter_events
/rosout
/statistics
/topic`

If you optionally changed the `topic_stats_options.publish_topic` field earlier in the tutorial, then you will see that name instead of `/statistics`.

The subscriber node you created is publishing statistics, for the topic `topic`, to the output topic `/statistics`.

We can visualize this using [RQt](https://docs.ros.org/en/humble/Concepts/Intermediate/About-RQt.html)

![https://docs.ros.org/en/humble/_images/topic_stats_rqt.png](https://docs.ros.org/en/humble/_images/topic_stats_rqt.png)

Now we can view the statistics data published to this topic with the following command:

`ros2 topic echo /statistics`

The terminal should start publishing statistics messages every 10 seconds, because the `topic_stats_options.publish_period` subscription configuration was optionally changed earlier in the tutorial.

- `--
measurement_source_name: minimal_subscriber_with_topic_statistics
metrics_source: message_age
unit: ms
window_start: sec: 1594856666 nanosec: 931527366
window_stop: sec: 1594856676 nanosec: 930797670
statistics:
- data_type: 1 data: .nan
- data_type: 3 data: .nan
- data_type: 2 data: .nan
- data_type: 5 data: 0.0
- data_type: 4 data: .nan
---
measurement_source_name: minimal_subscriber_with_topic_statistics
metrics_source: message_period
unit: ms
window_start: sec: 1594856666 nanosec: 931527366
window_stop: sec: 1594856676 nanosec: 930797670
statistics:
- data_type: 1 data: 499.2746365105009
- data_type: 3 data: 500.0
- data_type: 2 data: 499.0
- data_type: 5 data: 619.0
- data_type: 4 data: 0.4463309283488427
---`

From the [message definition](https://github.com/ros2/rcl_interfaces/tree/humble/statistics_msgs) the `data_types` are as follows

| data_type value | statistics |
| --- | --- |
| 1 | average |
| 2 | minimum |
| 3 | maximum |
| 4 | standard deviation |
| 5 | sample count |

Here we see the two currently possible calculated statistics for the `std_msgs::msg::String` message published to `/topic` by the `minimal_publisher`. Because the `std_msgs::msg::String` does not have a message header, the `message_age` calculation cannot be performed, so NaNs are returned. However, the `message_period` can be calculated and we see the statistics populated in the message above.

The `talker-listener` ROS 2 demo creates a `talker` node that publishes a “hello world” message every second, and a `listener` node that listens to these messages.

By [sourcing ROS 2](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html) you will get access to the CLI tool `fastdds`. This tool gives access to the [discovery tool](https://fast-dds.docs.eprosima.com/en/v2.1.0/fastddscli/cli/cli.html#discovery), which can be used to launch a discovery server. This server will manage the discovery process for the nodes that connect to it.

**Important**

Do not forget to [source ROS 2](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html) in every new terminal opened.

### **[Setup Discovery Server](https://docs.ros.org/en/humble/Tutorials/Advanced/Discovery-Server/Discovery-Server.html#id5)[](https://docs.ros.org/en/humble/Tutorials/Advanced/Discovery-Server/Discovery-Server.html#setup-discovery-server)**

Start by launching a discovery server with id 0, port 11811 (default port) and listening on all available interfaces.

Open a new terminal and run:

`fastdds discovery --server-id 0`

### **[Launch listener node](https://docs.ros.org/en/humble/Tutorials/Advanced/Discovery-Server/Discovery-Server.html#id6)[](https://docs.ros.org/en/humble/Tutorials/Advanced/Discovery-Server/Discovery-Server.html#launch-listener-node)**

Execute the listener demo, to listen to the `/chatter` topic.

In a new terminal, set the environment variable `ROS_DISCOVERY_SERVER` to the location of the discovery server. (Do not forget to source ROS 2 in every new terminal)

`export ROS_DISCOVERY_SERVER=127.0.0.1:11811`

Launch the listener node. Use the argument `--remap __node:=listener_discovery_server` to change the node’s name for this tutorial.

`ros2 run demo_nodes_cpp listener --ros-args --remap __node:=listener_discovery_server`

This will create a ROS 2 node, that will automatically create a client for the discovery server and connect to the server created previously to perform discovery, rather than using multicast.

### **[Launch talker node](https://docs.ros.org/en/humble/Tutorials/Advanced/Discovery-Server/Discovery-Server.html#id7)[](https://docs.ros.org/en/humble/Tutorials/Advanced/Discovery-Server/Discovery-Server.html#launch-talker-node)**

Open a new terminal and set the `ROS_DISCOVERY_SERVER` environment variable as before so that the node starts a discovery client.

`export ROS_DISCOVERY_SERVER=127.0.0.1:11811
ros2 run demo_nodes_cpp talker --ros-args --remap __node:=talker_discovery_server`

You should now see the talker publishing “hello world” messages, and the listener receiving these messages.

### **[Demonstrate Discovery Server execution](https://docs.ros.org/en/humble/Tutorials/Advanced/Discovery-Server/Discovery-Server.html#id8)[](https://docs.ros.org/en/humble/Tutorials/Advanced/Discovery-Server/Discovery-Server.html#demonstrate-discovery-server-execution)**

So far, there is no evidence that this example and the standard talker-listener example are running differently. To clearly demonstrate this, run another node that is not connected to the discovery server. Run a new listener (listening in `/chatter` topic by default) in a new terminal and check that it is not connected to the talker already running.

`ros2 run demo_nodes_cpp listener --ros-args --remap __node:=simple_listener`

The new listener node should not be receiving the “hello world” messages.

To finally verify that everything is running correctly, a new talker can be created using the simple discovery protocol (the default DDS distributed discovery mechanism) for discovery.

`ros2 run demo_nodes_cpp talker --ros-args --remap __node:=simple_talker`

Now you should see the `simple_listener` node receiving the “hello world” messages from `simple_talker` but not the other messages from `talker_discovery_server`.

### **[Visualization tool `rqt_graph`](https://docs.ros.org/en/humble/Tutorials/Advanced/Discovery-Server/Discovery-Server.html#id9)[](https://docs.ros.org/en/humble/Tutorials/Advanced/Discovery-Server/Discovery-Server.html#visualization-tool-rqt-graph)**

The `rqt_graph` tool can be used to verify the nodes and structure of this example. Remember, in order to use `rqt_graph` with the discovery server protocol (i.e., to see the `listener_discovery_server` and `talker_discovery_server` nodes) the `ROS_DISCOVERY_SERVER` environment variable must be set before launching it.

# **[Advance use cases](https://docs.ros.org/en/humble/Tutorials/Advanced/Discovery-Server/Discovery-Server.html#id10)[](https://docs.ros.org/en/humble/Tutorials/Advanced/Discovery-Server/Discovery-Server.html#advance-use-cases)**

The following sections show different features of the discovery server that allow you to build a robust discovery server over the network.

### **[Server Redundancy](https://docs.ros.org/en/humble/Tutorials/Advanced/Discovery-Server/Discovery-Server.html#id11)[](https://docs.ros.org/en/humble/Tutorials/Advanced/Discovery-Server/Discovery-Server.html#server-redundancy)**

By using `fastdds` tool, multiple discovery servers can be created. Discovery clients (ROS nodes) can connect to as many servers as desired. This allows to have a redundant network that will work even if some servers or nodes shut down unexpectedly. The figure below shows a simple architecture that provides server redundancy.

![https://docs.ros.org/en/humble/_images/ds_redundancy_example.svg](https://docs.ros.org/en/humble/_images/ds_redundancy_example.svg)

In several terminals, run the following code to establish a communication with redundant servers.

`fastdds discovery --server-id 0 --ip-address 127.0.0.1 --port 11811`

`fastdds discovery --server-id 1 --ip-address 127.0.0.1 --port 11888`

- `-server-id N` means server with id N. When referencing the servers with `ROS_DISCOVERY_SERVER`, server `0` must be in first place and server `1` in second place.

`export ROS_DISCOVERY_SERVER="127.0.0.1:11811;127.0.0.1:11888"
ros2 run demo_nodes_cpp talker --ros-args --remap __node:=talker`

`export ROS_DISCOVERY_SERVER="127.0.0.1:11811;127.0.0.1:11888"
ros2 run demo_nodes_cpp listener --ros-args --remap __node:=listener`

Now, if one of these servers fails, there will still be discovery capability available and nodes will still discover each other.

### **[Backup Server](https://docs.ros.org/en/humble/Tutorials/Advanced/Discovery-Server/Discovery-Server.html#id12)[](https://docs.ros.org/en/humble/Tutorials/Advanced/Discovery-Server/Discovery-Server.html#backup-server)**

The Fast DDS Discovery Server allows creating a server with backup functionality. This allows the server to restore the last state it saved in case of a shutdown.

![https://docs.ros.org/en/humble/_images/ds_backup_example.svg](https://docs.ros.org/en/humble/_images/ds_backup_example.svg)

In different terminals, run the following code to establish a communication with a backed-up server.

`fastdds discovery --server-id 0 --ip-address 127.0.0.1 --port 11811 --backup`

`export ROS_DISCOVERY_SERVER="127.0.0.1:11811"
ros2 run demo_nodes_cpp talker --ros-args --remap __node:=talker`

`export ROS_DISCOVERY_SERVER="127.0.0.1:11811"
ros2 run demo_nodes_cpp listener --ros-args --remap __node:=listener`

Several backup files are created in the discovery server’s working directory (the directory it was launched in). The two `SQLite` files and two `json` files contain the information required to start a new server and restore the failed server’s state in case of failure, avoiding the need for the discovery process to happen again, and without losing information.

### **[Discovery partitions](https://docs.ros.org/en/humble/Tutorials/Advanced/Discovery-Server/Discovery-Server.html#id13)[](https://docs.ros.org/en/humble/Tutorials/Advanced/Discovery-Server/Discovery-Server.html#discovery-partitions)**

Communication with discovery servers can be split to create virtual partitions in the discovery information. This means that two endpoints will only know about each other if there is a shared discovery server or a network of discovery servers between them. We are going to execute an example with two independent servers. The following figure shows the architecture.

![https://docs.ros.org/en/humble/_images/ds_partition_example.svg](https://docs.ros.org/en/humble/_images/ds_partition_example.svg)

With this schema `Listener 1` will be connected to `Talker 1` and `Talker 2`, as they share `Server 1`. `Listener 2` will connect with `Talker 1` as they share `Server 2`. But `Listener 2` will not hear the messages from `Talker 2` because they do not share any discovery server or discovery servers, including indirectly via connections between redundant discovery servers.

Run the first server listening on localhost with the default port of 11811.

`fastdds discovery --server-id 0 --ip-address 127.0.0.1 --port 11811`

In another terminal run the second server listening on localhost using another port, in this case port 11888.

`fastdds discovery --server-id 1 --ip-address 127.0.0.1 --port 11888`

Now, run each node in a different terminal. Use `ROS_DISCOVERY_SERVER` environment variable to decide which server they are connected to. Be aware that the [ids must match](https://fast-dds.docs.eprosima.com/en/v2.1.0/fastdds/env_vars/env_vars.html).

`export ROS_DISCOVERY_SERVER="127.0.0.1:11811;127.0.0.1:11888"
ros2 run demo_nodes_cpp talker --ros-args --remap __node:=talker_1`

`export ROS_DISCOVERY_SERVER="127.0.0.1:11811;127.0.0.1:11888"
ros2 run demo_nodes_cpp listener --ros-args --remap __node:=listener_1`

`export ROS_DISCOVERY_SERVER="127.0.0.1:11811"
ros2 run demo_nodes_cpp talker --ros-args --remap __node:=talker_2`

`export ROS_DISCOVERY_SERVER=";127.0.0.1:11888"
ros2 run demo_nodes_cpp listener --ros-args --remap __node:=listener_2`

We should see how `Listener 1` is receiving messages from both talker nodes, while `Listener 2` is in a different partition from `Talker 2` and so does not receive messages from it.

**Note**

Once two endpoints (ROS nodes) have discovered each other, they do not need the discovery server network between them to listen to each other’s messages.

# **[ROS 2 Introspection](https://docs.ros.org/en/humble/Tutorials/Advanced/Discovery-Server/Discovery-Server.html#id14)[](https://docs.ros.org/en/humble/Tutorials/Advanced/Discovery-Server/Discovery-Server.html#ros-2-introspection)**

The [ROS 2 Command Line Interface](https://github.com/ros2/ros2cli) supports several introspection tools to analyze the behavior of a ROS 2 network. These tools (i.e. `ros2 bag record`, `ros2 topic list`, etc.) are very helpful to understand a ROS 2 working network.

Most of these tools use DDS simple discovery to exchange topic information with every existing participant (using simple discovery, every participant in the network is connected with each other). However, the new Discovery Server v2 implements a network traffic reduction scheme that limits the discovery data between participants that do not share a topic. This means that nodes will only receive topic’s discovery data if it has a writer or a reader for that topic. As most ROS 2 CLIs need a node in the network (some of them rely on a running ROS 2 daemon, and some create their own nodes), using the Discovery Server v2 these nodes will not have all the network information, and thus their functionality will be limited.

The Discovery Server v2 functionality allows every Participant to run as a **Super Client**, a kind of **Client** that connects to a **Server**, from which it receives all the available discovery information (instead of just what it needs). In this sense, ROS 2 introspection tools can be configured as **Super Client**, thus being able to discover every entity that is using the Discovery Server protocol within the network.

**Note**

In this section we use the term *Participant* as a DDS entity. Each DDS *Participant* corresponds with a ROS 2 *Context*, a ROS 2 abstraction over DDS. [Nodes](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html#ros2nodes) are ROS 2 entities that rely on DDS communication interfaces: `DataWriter` and `DataReader`. Each *Participant* can hold multiple ROS 2 Nodes. For further details about these concepts, please visit the [Node to Participant mapping design document](http://design.ros2.org/articles/Node_to_Participant_mapping.html)

### **[Daemon’s related tools](https://docs.ros.org/en/humble/Tutorials/Advanced/Discovery-Server/Discovery-Server.html#id15)[](https://docs.ros.org/en/humble/Tutorials/Advanced/Discovery-Server/Discovery-Server.html#daemon-s-related-tools)**

The ROS 2 Daemon is used in several ROS 2 CLI introspection tools. It creates its own Participant to add a ROS 2 Node to the network graph, in order to receive all the data sent. In order for the ROS 2 CLI to work when using Discovery Server mechanism, the ROS 2 Daemon needs to be configured as **Super Client**. Therefore, this section is devoted to explain how to use ROS 2 CLI with ROS 2 Daemon running as a **Super Client**. This will allow the Daemon to discover the entire Node graph, and to receive all topic and endpoint information. To do so, a Fast DDS XML configuration file is used to configure the ROS 2 Daemon and CLI tools.

Below you can find a XML configuration profile, which for this tutorial should be saved in the working directory as ``super_client_configuration_file.xml`` file. This file will configure every new participant using it, as a **Super Client**.

`<?xml version="1.0" encoding="UTF-8" ?>
 **<dds><profiles** xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles"**><participant** profile_name="super_client_profile" is_default_profile="true"**><rtps><builtin><discovery_config><discoveryProtocol>**SUPER_CLIENT**</discoveryProtocol><discoveryServersList><RemoteServer** prefix="44.53.00.5f.45.50.52.4f.53.49.4d.41"**><metatrafficUnicastLocatorList><locator><udpv4><address>**127.0.0.1**</address><port>**11811**</port></udpv4></locator></metatrafficUnicastLocatorList></RemoteServer></discoveryServersList></discovery_config></builtin></rtps></participant></profiles></dds>**`

**Note**

Under the *RemoteServer* tag, the *prefix* attribute value should be updated according to the server ID passed on the CLI (see [Fast DDS CLI](https://fast-dds.docs.eprosima.com/en/latest/fastddscli/cli/cli.html#discovery)). The value specified in the shown XML snippet corresponds to an ID of value 0.

First of all, instantiate a Discovery Server using [Fast DDS CLI](https://fast-dds.docs.eprosima.com/en/latest/fastddscli/cli/cli.html#discovery) specifying an ID of value 0.

`fastdds discovery -i 0 -l 127.0.0.1 -p 11811`

Run a talker and a listener that will discover each other through the Server (notice that `ROS_DISCOVERY_SERVER` configuration is the same as the one in `super_client_configuration_file.xml`).

`export ROS_DISCOVERY_SERVER="127.0.0.1:11811"
ros2 run demo_nodes_cpp listener --ros-args --remap __node:=listener`

`export ROS_DISCOVERY_SERVER="127.0.0.1:11811"
ros2 run demo_nodes_cpp talker --ros-args --remap __node:=talker`

Then, instantiate a ROS 2 Daemon using the **Super Client** configuration (remember to source ROS 2 installation in every new terminal).

`export FASTRTPS_DEFAULT_PROFILES_FILE=super_client_configuration_file.xml
ros2 daemon stop
ros2 daemon start
ros2 topic list
ros2 node info /talker
ros2 topic info /chatter
ros2 topic echo /chatter`

We can also see the Node’s Graph using the ROS 2 tool `rqt_graph` as follows (you may need to press the refresh button):

`export FASTRTPS_DEFAULT_PROFILES_FILE=super_client_configuration_file.xml
rqt_graph`

### **[No Daemon tools](https://docs.ros.org/en/humble/Tutorials/Advanced/Discovery-Server/Discovery-Server.html#id16)[](https://docs.ros.org/en/humble/Tutorials/Advanced/Discovery-Server/Discovery-Server.html#no-daemon-tools)**

Some ROS 2 CLI tools do not use the ROS 2 Daemon. In order for these tools to connect with a Discovery Server and receive all the topics information they need to be instantiated as a **Super Client** that connects to the **Server**.

Following the previous configuration, build a simple system with a talker and a listener. First, run a **Server**:

`fastdds discovery -i 0 -l 127.0.0.1 -p 11811`

Then, run the talker and listener in separate terminals:

`export ROS_DISCOVERY_SERVER="127.0.0.1:11811"
ros2 run demo_nodes_cpp listener --ros-args --remap __node:=listener`

`export ROS_DISCOVERY_SERVER="127.0.0.1:11811"
ros2 run demo_nodes_cpp talker --ros-args --remap __node:=talker`

Continue using the ROS 2 CLI with `--no-daemon` option with the new configuration. New nodes will connect with the existing Server and will know every topic. Exporting `ROS_DISCOVERY_SERVER` is not needed as the ROS 2 tools will be configured through the `FASTRTPS_DEFAULT_PROFILES_FILE`.

`export FASTRTPS_DEFAULT_PROFILES_FILE=super_client_configuration_file.xml
ros2 topic list --no-daemon
ros2 node info /talker --no-daemon --spin-time 2`

# **[Compare Fast DDS Discovery Server with Simple Discovery Protocol](https://docs.ros.org/en/humble/Tutorials/Advanced/Discovery-Server/Discovery-Server.html#id17)[](https://docs.ros.org/en/humble/Tutorials/Advanced/Discovery-Server/Discovery-Server.html#compare-fast-dds-discovery-server-with-simple-discovery-protocol)**

In order to compare executing nodes using the *Simple Discovery* Protocol (the default DDS mechanism for distributed discovery) or the *Discovery Server*, two scripts that execute a talker and many listeners and analyze the network traffic during this time are provided. For this experiment, `tshark` is required to be installed on your system. The configuration file is mandatory in order to avoid using intraprocess mode.

**Note**

These scripts require a discovery server closure feature that is only available from versions newer than the version provided in ROS 2 Foxy. In order to use this functionality, compile ROS 2 with Fast DDS v2.1.0 or higher.

These scripts’ features are references for advanced purposes and their study is left to the user.

- `[bash network traffic generator](https://docs.ros.org/en/humble/_downloads/4091f7105a6c8d10b26927288ede29fc/generate_discovery_packages.bash)`
    
    `[bash network traffic generator](https://docs.ros.org/en/humble/_downloads/4091f7105a6c8d10b26927288ede29fc/generate_discovery_packages.bash)`
    
- `[python3 graph generator](https://docs.ros.org/en/humble/_downloads/617a6849c029c43a931e24db314cb224/discovery_packets.py)`
    
    `[python3 graph generator](https://docs.ros.org/en/humble/_downloads/617a6849c029c43a931e24db314cb224/discovery_packets.py)`
    
- `[XML configuration](https://docs.ros.org/en/humble/_downloads/358434c24ca02455545e1739688b839b/no_intraprocess_configuration.xml)`
    
    `[XML configuration](https://docs.ros.org/en/humble/_downloads/358434c24ca02455545e1739688b839b/no_intraprocess_configuration.xml)`
    

Run the bash script with the path to `setup.bash` file to source ROS 2 as an argument. This will generate the traffic trace for simple discovery. Execute the same script with second argument `SERVER`. It will generate the trace for using the discovery server.

**Note**

Depending on your configuration of `tcpdump`, this script may require `sudo` privileges to read traffic across your network device.

After both executions are done, run the Python script to generate a graph similar to the one below.

**`$** export FASTRTPS_DEFAULT_PROFILES_FILE="no_intraprocess_configuration.xml"
**$** sudo bash generate_discovery_packages.bash ~/ros2_foxy/install/local_setup.bash
**$** sudo bash generate_discovery_packages.bash ~/ros2_foxy/install/local_setup.bash SERVER
**$** python3 discovery_packets.py`

![https://docs.ros.org/en/humble/_images/discovery_packets.svg](https://docs.ros.org/en/humble/_images/discovery_packets.svg)

This graph is the result of a specific run of the experiment. The reader can execute the scripts and generate their own results for comparison. It can easily be seen that network traffic is reduced when using discovery service.

The reduction in traffic is a result of avoiding every node announcing itself and waiting a response from every other node on the network. This creates a huge amount of traffic in large architectures. The reduction from this method increases with the number of nodes, making this architecture more scalable than the Simple Discovery Protocol approach.

The new Fast DDS Discovery Server v2 is available since *Fast DDS* v2.0.2, replacing the old discovery server. In this new version, those nodes that do not share topics will automatically not discover each other, saving the whole discovery data required to connect them and their endpoints. The experiment above does not show this case, but even so the massive reduction in traffic can be appreciated due to the hidden infrastructure topics of ROS 2 nodes.
