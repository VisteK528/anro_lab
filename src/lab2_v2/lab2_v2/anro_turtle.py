import sys

from rclpy.node import Node, ParameterDescriptor
from rclpy.action import ActionClient
from turtlesim.action import RotateAbsolute
from turtlesim.srv import Spawn
from turtlesim.srv._spawn import Spawn_Request
from turtlesim.action._rotate_absolute import RotateAbsolute_Goal
from enum import Enum
import rclpy
import numpy as np
import time

class TurtleStates(Enum):
    ROTATE_180 = 0
    SPAWN = 1
    WAIT = 2
    ROTATE_30 = 3
    ROTATE_270 = 4
    SPAWN2 = 5
    SHUTDOWN = 6

    def next(self):
        cls = self.__class__
        members = list(cls)
        index = min(members.index(self) + 1, len(members)-1)
        return members[index]


class AnroTurtle(Node):
    def __init__(self):
        super().__init__("anro_turtle")
        self.declare_parameter("turtle1_name", value="Skipper",
                               descriptor=ParameterDescriptor(description="Name of the first turtle to be spawned"))
        self.declare_parameter("turtle2_name", value="Rico",
                               descriptor=ParameterDescriptor(description="Name of the second turtle to be spawned"))

        self._default_turtle_rotate_client = ActionClient(self, RotateAbsolute, "turtle1/rotate_absolute")
        self._spawn_turtle_client = self.create_client(Spawn, "spawn")

        self._state = TurtleStates.ROTATE_180
        self._spawn_turtle_gen = self._next_turtle_params()

    def _next_turtle_params(self):
        for i in range(2):
            turtle_name = self.get_parameter(f"turtle{i+1}_name").get_parameter_value().string_value
            y = 2.77 + 5.54 * i
            x = 5.54
            yield turtle_name, x, y

    def _rotate_turtle(self, angle: float):
        message = RotateAbsolute_Goal()
        message.theta = np.deg2rad(angle)

        self._default_turtle_rotate_client.wait_for_server()
        self._send_goal_future = self._default_turtle_rotate_client.send_goal_async(message)
        self._send_goal_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self._done_future = goal_handle.get_result_async()
        self._done_future.add_done_callback(self._done_callback)

    def _done_callback(self, future):
        self._state = self._state.next()
        self.run()

    def _spawn_new_turtle(self, name: str, x: float, y: float):
        request = Spawn_Request(name=name, x=x, y=y)
        self._spawn_future = self._spawn_turtle_client.call_async(request)
        self._spawn_future.add_done_callback(self._done_callback)

    def run(self):
        self.get_logger().info(f"Entering state: {self._state.name}...")
        match self._state:
            case TurtleStates.ROTATE_180:
                self._rotate_turtle(180)
            case TurtleStates.WAIT:
                time.sleep(2)
                self._state = self._state.next()
                self.run()
            case TurtleStates.SPAWN | TurtleStates.SPAWN2:
                try:
                    self._spawn_new_turtle(*next(self._spawn_turtle_gen))
                except StopIteration:
                    self._state = TurtleStates.SHUTDOWN
                    self.run()
            case TurtleStates.ROTATE_30:
                self._rotate_turtle(30)
            case TurtleStates.ROTATE_270:
                self._rotate_turtle(270)
            case TurtleStates.SHUTDOWN:
                self.get_logger().info("All goals completed, starting shutdown...")
                sys.exit()


def main(args=None):
    rclpy.init(args=args)
    node = AnroTurtle()
    node.run()

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
