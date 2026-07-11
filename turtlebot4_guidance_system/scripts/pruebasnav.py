#!/usr/bin/env python3
"""
Benchmark MPPI vs Regulated Pure Pursuit para Nav2.
Uso:
  python3 pruebasnav.py --controller MPPI --goal "2.0 1.0 0.0" --test-name prueba1 --output mppi_results.csv 
  python3 pruebasnav.py --controller RPP  --goal "2.0 1.0 0.0" --test-name prueba2 --output rpp_results.csv

"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
import math, time, csv, os, argparse
from datetime import datetime


class BenchmarkNode(Node):
    def __init__(self, goal_x, goal_y, goal_yaw, controller, test_name):
        super().__init__('nav2_benchmark')
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.goal_yaw = goal_yaw
        self.controller = controller
        self.test_name = test_name

        self.start_pos = None
        self.current_pos = None
        self.path_length = 0.0
        self.deviations = []
        self.replanning_count = 0
        self.start_time = None
        self.elapsed = None
        self.success = False
        self.last_odom_time = None
        self._done = False

        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._odom_sub = self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.get_logger().info(f'Benchmark iniciado: {controller} → goal ({goal_x}, {goal_y})')

    def _odom_cb(self, msg):
        pos = msg.pose.pose.position
        now = time.time()

        if self.start_pos is None:
            self.start_pos = (pos.x, pos.y)

        if self.current_pos is not None and self.start_time is not None:
            dx = pos.x - self.current_pos[0]
            dy = pos.y - self.current_pos[1]
            self.path_length += math.hypot(dx, dy)

            # Desviación respecto a la línea recta origen→goal
            dev = self._cross_track_error(pos.x, pos.y)
            self.deviations.append(abs(dev))

        self.current_pos = (pos.x, pos.y)
        self.last_odom_time = now

    def _cross_track_error(self, x, y):
        if self.start_pos is None:
            return 0.0
        sx, sy = self.start_pos
        ex, ey = self.goal_x, self.goal_y
        dx, dy = ex - sx, ey - sy
        length = math.hypot(dx, dy)
        if length < 1e-6:
            return 0.0
        return ((ey - sy) * (sx - x) - (ex - sx) * (sy - y)) / length

    def send_goal(self):
        self._action_client.wait_for_server()
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = self.goal_x
        goal_msg.pose.pose.position.y = self.goal_y
        goal_msg.pose.pose.orientation.z = math.sin(self.goal_yaw / 2)
        goal_msg.pose.pose.orientation.w = math.cos(self.goal_yaw / 2)

        self.start_time = time.time()
        future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self._feedback_cb)
        future.add_done_callback(self._goal_response_cb)

    def _feedback_cb(self, feedback):
        # Solo contar si el path tiene diferente longitud
        new_len = len(feedback.feedback.current_pose.header.frame_id)
        if not hasattr(self, '_last_path_len') or self._last_path_len != new_len:
            self.replanning_count += 1
            self._last_path_len = new_len
            
    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rechazado')
            self._done = True
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future):
        self.elapsed = time.time() - self.start_time
        result = future.result().result
        # NavigateToPose devuelve error_code=0 si éxito
        self.success = (future.result().status == 4)  # STATUS_SUCCEEDED
        self._done = True
        self.get_logger().info(
            f'Navegación {"OK" if self.success else "FALLIDA"} en {self.elapsed:.2f}s')

    def save_results(self, output_file='nav2_results.csv'):
        mean_dev = (sum(self.deviations) / len(self.deviations)
                    if self.deviations else 0.0)
        straight_dist = math.hypot(
            self.goal_x - (self.start_pos[0] if self.start_pos else 0),
            self.goal_y - (self.start_pos[1] if self.start_pos else 0))

        row = {
            'timestamp':    datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
            'test_name':    self.test_name,
            'controller':   self.controller,
            'goal_x':       self.goal_x,
            'goal_y':       self.goal_y,
            'tiempo_s':     round(self.elapsed or 0, 3),
            'distancia_recorrida_m': round(self.path_length, 3),
            'distancia_directa_m':   round(straight_dist, 3),
            'eficiencia':   round(straight_dist / self.path_length, 3)
                            if self.path_length > 0 else 0,
            'desviacion_media_m':    round(mean_dev, 4),
            'replanning_count':      self.replanning_count,
            'goal_alcanzado':        'Si' if self.success else 'No',
        }

        file_exists = os.path.isfile(output_file)
        with open(output_file, 'a', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=row.keys())
            if not file_exists:
                writer.writeheader()
            writer.writerow(row)

        print('\n' + '='*55)
        print(f'  Controlador : {self.controller}')
        print(f'  Test        : {self.test_name}')
        print(f'  Tiempo      : {row["tiempo_s"]} s')
        print(f'  Distancia   : {row["distancia_recorrida_m"]} m '
              f'(directa: {row["distancia_directa_m"]} m)')
        print(f'  Eficiencia  : {row["eficiencia"]}')
        print(f'  Desviación  : {row["desviacion_media_m"]} m')
        print(f'  Replanning  : {self.replanning_count}')
        print(f'  Goal        : {"✓" if self.success else "✗"}')
        print('='*55)
        print(f'  Guardado en: {output_file}')


def main():
    parser = argparse.ArgumentParser(description='Nav2 Benchmark')
    parser.add_argument('--controller', required=True, choices=['MPPI','RPP'],
                        help='Controlador activo')
    parser.add_argument('--goal', required=True,
                        help='"x y yaw" en metros/rad, ej: "2.0 1.0 0.0"')
    parser.add_argument('--test-name', default='test',
                        help='Nombre identificativo de la prueba')
    parser.add_argument('--output', default='nav2_results.csv',
                        help='Archivo CSV de salida')
    parser.add_argument('--timeout', type=float, default=60.0,
                        help='Timeout en segundos (default: 60)')
    args = parser.parse_args()

    gx, gy, gyaw = [float(v) for v in args.goal.split()]

    rclpy.init()
    node = BenchmarkNode(gx, gy, gyaw, args.controller, args.test_name)
    node.send_goal()

    deadline = time.time() + args.timeout
    while not node._done and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    if not node._done:
        node.get_logger().warn(f'Timeout tras {args.timeout}s')
        node.elapsed = args.timeout
        node.success = False

    node.save_results(args.output)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
