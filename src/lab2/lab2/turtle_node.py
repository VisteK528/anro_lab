import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from turtlesim.action import RotateAbsolute
from turtlesim.srv import Spawn
from turtlesim.srv._spawn import Spawn_Request
from turtlesim.action._rotate_absolute import RotateAbsolute_Goal
import numpy as np


class AnroTurtleClient(Node):
    def __init__(self):
        super().__init__("anro_turtle")
        self.declare_parameter("turtle2_name", "hexaturtle")
        self.declare_parameter("turtle3_name", "quadturtle")

        self._default_turtle_rotate = ActionClient(self, RotateAbsolute, "turtle1/rotate_absolute")
        self._spawn_turtle = self.create_client(Spawn, "spawn")

        self._spawned_turtles = 0
        self.spawn = None
        self.task_finished = False

    def rotate_default_turtle(self, angle: float, spawn: bool):
        self.task_finished = False
        self.spawn = spawn

        message = RotateAbsolute_Goal()
        message.theta = np.deg2rad(angle)

        self._default_turtle_rotate.wait_for_server()
        self.get_logger().info(f"Rotating first turtle to theta={angle} deg")
        future = self._default_turtle_rotate.send_goal_async(message)
        future.add_done_callback(self.goal_response_callback)
        return future

    def done_callback(self, future):
        if self.spawn:
            request = Spawn_Request()

            turtle_name = self.get_parameter(
                f"turtle{self._spawned_turtles + 2}_name").get_parameter_value().string_value

            request.name = turtle_name
            request.y = 2.77 + 5.54 * (self._spawned_turtles)
            request.x = 5.54

            self._spawn_turtle.call_async(request)
            self.get_logger().info(f"Spawned a new turtle with name {turtle_name}!")
            self._spawned_turtles += 1

        self.task_finished = True

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.done_callback)


def main(args=None):
    rclpy.init(args=args)

    turtle_node = AnroTurtleClient()
    future2 = turtle_node.rotate_default_turtle(180, True)

    while not turtle_node.task_finished:
        rclpy.spin_once(turtle_node)

    time.sleep(1)
    turtle_node.rotate_default_turtle(30, False)

    while not turtle_node.task_finished:
        rclpy.spin_once(turtle_node)

    turtle_node.rotate_default_turtle(270, True)

    while not turtle_node.task_finished:
        rclpy.spin_once(turtle_node)

    rclpy.spin(turtle_node)


if __name__ == "__main__":
    main()
