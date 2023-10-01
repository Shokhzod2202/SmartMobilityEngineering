python3 fibonacci_action_server.py
echo "run fibonacci_action_server.py"

ros2 action send_goal fibonacci action_tutorials_interfaces/action/Fibonacci "{order: 5}"

ros2 action send_goal --feedback fibonacci action_tutorials_interfaces/action/Fibonacci "{order: 5}"

python3 fibonacci_action_server.py

python3 fibonacci_action_client.py

python3 fibonacci_action_client.py


