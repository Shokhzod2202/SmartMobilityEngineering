ros2 component types

ros2 run rclcpp_components component_container

ros2 component list

ros2 component load /ComponentManager composition composition::Talker

ros2 component load /ComponentManager composition composition::Listener

ros2 component list

ros2 run rclcpp_components component_container

ros2 run composition manual_composition

ros2 run composition dlopen_composition `ros2 pkg prefix composition`/lib/libtalker_component.so `ros2 pkg prefix composition`/lib/liblistener_component.so

ros2 launch composition composition_demo.launch.py


