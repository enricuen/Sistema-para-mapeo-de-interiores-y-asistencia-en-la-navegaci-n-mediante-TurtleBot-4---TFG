#!/usr/bin/env python3
from enum import Enum
import time
import threading
import math
import cv2
import os
import queue
import tkinter as tk
from tkinter import scrolledtext

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Quaternion, PoseStamped, PoseWithCovarianceStamped, Twist
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import Spin, NavigateToPose
from irobot_create_msgs.action import Dock, Undock, AudioNoteSequence
from irobot_create_msgs.msg import DockStatus, AudioNote, AudioNoteVector


class TaskResult(Enum):
    UNKNOWN = 0
    SUCCEEDED = 1
    CANCELED = 2
    FAILED = 3

amcl_pose_qos = QoSProfile(
          durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
          reliability=QoSReliabilityPolicy.RELIABLE,
          history=QoSHistoryPolicy.KEEP_LAST,
          depth=1)

class RobotCommander(Node):

    def __init__(self, node_name='robot_commander', namespace=''):
        super().__init__(node_name=node_name, namespace=namespace)
        
        self.pose_frame_id = 'map'
        
        # Flags and helper variables
        self.goal_handle = None
        self.result_future = None
        self.feedback = None
        self.status = None
        self.initial_pose_received = False
        self.is_docked = None

        # Callback opcional para conectar logs con la GUI
        self.status_callback = None

        # ROS2 subscribers
        self.create_subscription(DockStatus, 'dock_status', self._dockCallback, qos_profile_sensor_data)
        self.localization_pose_sub = self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self._amclPoseCallback, amcl_pose_qos)
        
        # ROS2 publishers
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        
        # ROS2 Action clients
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.spin_client = ActionClient(self, Spin, 'spin')
        self.undock_action_client = ActionClient(self, Undock, 'undock')
        self.dock_action_client = ActionClient(self, Dock, 'dock')

        # --- Integración del Beeper ---
        self.interval = 1.0
        self.motion = "stop"
        self._busy = False
        self._last = 0.0
        self.lock = threading.Lock()
        
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, qos_profile_sensor_data)
        self.audio_client = ActionClient(self, AudioNoteSequence, 'audio_note_sequence')
        self.create_timer(0.1, self.tick)
        # ------------------------------

        self.get_logger().info(f"Robot commander has been initialized!")
        
    def destroyNode(self):
        self.nav_to_pose_client.destroy()
        self.audio_client.destroy()
        super().destroy_node()     

    def goToPose(self, pose, behavior_tree=''):
        self.debug("Waiting for 'NavigateToPose' action server")
        while not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.info("'NavigateToPose' action server not available, waiting...")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        goal_msg.behavior_tree = behavior_tree

        self.info('Navigating to goal: ' + str(pose.pose.position.x) + ' ' + str(pose.pose.position.y) + '...')
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg, self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error('Goal rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True

    def spin(self, spin_dist=1.57, time_allowance=10):
        self.debug("Waiting for 'Spin' action server")
        while not self.spin_client.wait_for_server(timeout_sec=1.0):
            self.info("'Spin' action server not available, waiting...")
        goal_msg = Spin.Goal()
        goal_msg.target_yaw = spin_dist
        goal_msg.time_allowance = Duration(sec=time_allowance)

        send_goal_future = self.spin_client.send_goal_async(goal_msg, self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error('Spin request was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True
    
    def undock(self):
        self.info('Undocking...')
        self.undock_send_goal()
        while not self.isUndockComplete():
            time.sleep(0.1)

    def undock_send_goal(self):
        goal_msg = Undock.Goal()
        self.undock_action_client.wait_for_server()
        goal_future = self.undock_action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, goal_future)
        self.undock_goal_handle = goal_future.result()

        if not self.undock_goal_handle.accepted:
            self.error('Undock goal rejected')
            return

        self.undock_result_future = self.undock_goal_handle.get_result_async()

    def isUndockComplete(self):
        if self.undock_result_future is None or not self.undock_result_future:
            return True
        rclpy.spin_until_future_complete(self, self.undock_result_future, timeout_sec=0.1)
        if self.undock_result_future.result():
            self.undock_status = self.undock_result_future.result().status
            if self.undock_status != GoalStatus.STATUS_SUCCEEDED:
                return True
        else:
            return False
        return True

    def cancelTask(self):
        self.info('Canceling current task.')
        if self.result_future:
            future = self.goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, future)
        return

    def isTaskComplete(self):
        if not self.result_future:
            return True
        rclpy.spin_until_future_complete(self, self.result_future, timeout_sec=0.10)
        if self.result_future.result():
            self.status = self.result_future.result().status
            if self.status != GoalStatus.STATUS_SUCCEEDED:
                return True
        else:
            return False
        return True

    def getFeedback(self):
        return self.feedback

    def getResult(self):
        if self.status == GoalStatus.STATUS_SUCCEEDED:
            return TaskResult.SUCCEEDED
        elif self.status == GoalStatus.STATUS_ABORTED:
            return TaskResult.FAILED
        elif self.status == GoalStatus.STATUS_CANCELED:
            return TaskResult.CANCELED
        else:
            return TaskResult.UNKNOWN

    def waitUntilNav2Active(self, navigator='bt_navigator', localizer='amcl'):
        self._waitForNodeToActivate(localizer)
        if not self.initial_pose_received:
            time.sleep(1)
        self._waitForNodeToActivate(navigator)
        self.info('Nav2 is ready for use!')
        return

    def _waitForNodeToActivate(self, node_name):
        self.debug(f'Waiting for {node_name} to become active..')
        node_service = f'{node_name}/get_state'
        state_client = self.create_client(GetState, node_service)
        while not state_client.wait_for_service(timeout_sec=1.0):
            self.info(f'{node_service} service not available, waiting...')

        req = GetState.Request()
        state = 'unknown'
        while state != 'active':
            future = state_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                state = future.result().current_state.label
            time.sleep(2)
        return
    
    def YawToQuaternion(self, angle_z = 0.):
        quat_msg = Quaternion()
        quat_msg.x = 0.0
        quat_msg.y = 0.0
        quat_msg.z = math.sin(angle_z / 2.0)
        quat_msg.w = math.cos(angle_z / 2.0)
        return quat_msg

    def _amclPoseCallback(self, msg):
        self.initial_pose_received = True
        self.current_pose = msg.pose
        return

    def _feedbackCallback(self, msg):
        self.feedback = msg.feedback
        return
    
    def _dockCallback(self, msg: DockStatus):
        self.is_docked = msg.is_docked

    def setInitialPose(self, pose):
        msg = PoseWithCovarianceStamped()
        msg.pose.pose = pose
        msg.header.frame_id = self.pose_frame_id
        msg.header.stamp = 0
        self.initial_pose_pub.publish(msg)
        return

    # --- Logs modificados para GUI ---
    def info(self, msg):
        self.get_logger().info(msg)
        self._emit_status('info', msg)

    def warn(self, msg):
        self.get_logger().warn(msg)
        self._emit_status('warn', msg)

    def error(self, msg):
        self.get_logger().error(msg)
        self._emit_status('error', msg)

    def debug(self, msg):
        self.get_logger().debug(msg)

    def _emit_status(self, level, msg):
        if self.status_callback is not None:
            try:
                self.status_callback(level, msg)
            except Exception:
                pass
    # ----------------------------------

    # --- Lógica del Beeper ---
    def cmd_vel_callback(self, msg):
        if msg.linear.x > 0.05:
            self.motion = "forward"
        elif msg.angular.z > 0.2:
            self.motion = "left"
        elif msg.angular.z < -0.2:
            self.motion = "right"
        else:
            self.motion = "stop"

    def tick(self):
        if self.motion == "stop": return
        if time.monotonic() - self._last < self.interval: return
        with self.lock:
            if self._busy: return
            self._busy = True
            self._last = time.monotonic()
        self.send_beep()

    def send_beep(self):
        if not self.audio_client.wait_for_server(timeout_sec=1.0):
            self._busy = False; return
        goal = AudioNoteSequence.Goal(); goal.iterations = 1
        vec = AudioNoteVector(); vec.append = False; notes = []
        
        def mk(freq, ms):
            n = AudioNote(); n.frequency = freq
            n.max_runtime = Duration(sec=0, nanosec=int(ms * 1e6))
            return n
            
        if self.motion == "forward":
            notes = [mk(900, 100)]
        elif self.motion == "left":
            notes = [mk(650, 70), mk(650, 70)]
        elif self.motion == "right":
            notes = [mk(1300, 220)]
            
        vec.notes = notes; goal.note_sequence = vec
        fut = self.audio_client.send_goal_async(goal)
        fut.add_done_callback(self.beep_goal_cb)

    def beep_goal_cb(self, f):
        gh = f.result()
        if not gh.accepted:
            self._busy = False; return
        gh.get_result_async().add_done_callback(lambda _: self.beep_done())

    def beep_done(self):
        with self.lock:
            self._busy = False
    # --------------------------


def make_pose(rc: RobotCommander, x: float, y: float, yaw: float) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = rc.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation = rc.YawToQuaternion(yaw)
    return pose

def define_goal_poses(rc: RobotCommander) -> dict:
    goals = {}

    goals[1] = {
        'name': 'Aula 1',
        'entry_point':    make_pose(rc, -1.95, -3.75, -1.57),
        'goal':           make_pose(rc, -1.23, -6.57, -1.57),
        'fallback_point': make_pose(rc, 0.56, 0.37,  0.0),
    }

    goals[2] = {
        'name': 'Aula 2',
        'entry_point':    make_pose(rc, -3.0, -2.0, -1.57),
        'goal':           make_pose(rc, -3.14, 0.64, -1.57),
        'fallback_point': make_pose(rc,  0.0,  0.0,  0.0),
    }

    goals[3] = {
        'name': 'Aula 3',
        'entry_point':    make_pose(rc,  2.5,  2.5,  1.57),
        'goal':           make_pose(rc,  4.38,  -0.31,  1.57),
        'fallback_point': make_pose(rc,  0.0,  0.0,  0.0),
    }

    goals[4] = {
        'name': 'Aula 4',
        'entry_point':    make_pose(rc,  2.5, -2.0,  1.57),
        'goal':           make_pose(rc,  4.38, 0.88,  1.57),
        'fallback_point': make_pose(rc,  0.0,  0.0,  0.0),
    }

    return goals


def navigate_and_wait(rc: RobotCommander, pose: PoseStamped, description: str) -> TaskResult:
    pose.header.stamp = rc.get_clock().now().to_msg()
    rc.goToPose(pose)
 
    last_log_time = time.time()
    
    # ── BUCLE NO BLOQUEANTE PARA QUE FUNCIONEN LOS PITIDOS ──
    while not rc.isTaskComplete():
        current_time = time.time()
        if current_time - last_log_time >= 1.0:
            rc.info(f"Navegando: {description}...")
            last_log_time = current_time
 
    return rc.getResult()


def run_navigation_cycle(rc: RobotCommander, aula: dict, aula_name: str, status_queue: queue.Queue):
    """Ejecuta un ciclo completo y reporta el progreso a la GUI."""
    status_queue.put(('state', 'busy', f"Yendo al punto de entrada de {aula_name}..."))
    result = navigate_and_wait(rc, aula['entry_point'], f"punto de entrada {aula_name}")

    if result != TaskResult.SUCCEEDED:
        rc.error(f"No se pudo alcanzar el punto de entrada de {aula_name}. Resultado: {result}")
        status_queue.put(('state', 'error', f"Error al ir al punto de entrada de {aula_name}."))
        return

    rc.info(f"Llegando al punto de entrada de {aula_name}. Iniciando análisis visual...")
    status_queue.put(('state', 'busy', f"Analizando puerta de {aula_name} (3s)..."))
    time.sleep(3.0)
    
    image_path = 'puertaabierta.jpg'
    try:
        if os.path.exists(image_path):
            img = cv2.imread(image_path)
            if img is not None:
                rc.info(f"¡Señal detectada! Mostrando '{image_path}' en pantalla.")
                cv2.imshow('Deteccion del Robot', img)
                cv2.waitKey(4000)
                cv2.destroyAllWindows()
            else:
                rc.error("El archivo existe pero OpenCV no pudo leer la imagen.")
        else:
            rc.warn(f"No se encontró la imagen '{image_path}' en el directorio actual.")
    except Exception as e:
        rc.warn(f"Error gráfico al intentar mostrar la imagen (ignorando): {e}")

    status_queue.put(('state', 'busy', f"La puerta está abierta. Entrando a {aula_name}..."))
    rc.info(f"La puerta de {aula_name} está ABIERTA. Entrando al aula...")
    result = navigate_and_wait(rc, aula['goal'], f"interior {aula_name}")

    if result == TaskResult.SUCCEEDED:
        rc.info(f"¡Destino alcanzado! El robot está dentro de {aula_name}.")
        status_queue.put(('state', 'ready', f"Llegado a {aula_name}. Selecciona otro destino."))
    else:
        rc.error("No se pudo alcanzar el interior del aula.")
        status_queue.put(('state', 'error', f"No se pudo entrar a {aula_name}."))


def ros_worker(rc: RobotCommander, goals: dict, command_queue: queue.Queue, status_queue: queue.Queue):
    """Hilo en segundo plano para ROS 2."""
    status_queue.put(('state', 'init', "Inicializando sistema de navegación..."))
    rc.waitUntilNav2Active()

    while rc.is_docked is None:
        rclpy.spin_once(rc, timeout_sec=0.5)

    if rc.is_docked:
        rc.undock()

    status_queue.put(('state', 'ready', "Listo. Elige un aula de destino."))

    while rclpy.ok():
        try:
            selected = command_queue.get(timeout=0.5)
        except queue.Empty:
            continue

        if selected == 'SHUTDOWN':
            break

        aula = goals[selected]
        run_navigation_cycle(rc, aula, aula['name'], status_queue)


class RobotGuiaGUI:
    """Panel de control visual de Tkinter."""

    STATE_COLORS = {
        'init':  '#2563eb',   # azul
        'busy':  '#d97706',   # naranja
        'ready': '#16a34a',   # verde
        'error': '#dc2626',   # rojo
    }

    def __init__(self, root, goals: dict, command_queue: queue.Queue, status_queue: queue.Queue):
        self.root = root
        self.goals = goals
        self.command_queue = command_queue
        self.status_queue = status_queue
        self.buttons = {}

        root.title("Panel de control - TurtleBot4")
        root.geometry("520x500")
        root.configure(bg="#0f172a")
        root.protocol("WM_DELETE_WINDOW", self.on_close)

        tk.Label(
            root, text="Selecciona destino", font=("Helvetica", 18, "bold"),
            bg="#0f172a", fg="white"
        ).pack(pady=(16, 4))

        self.status_label = tk.Label(
            root, text="Inicializando...", font=("Helvetica", 12),
            bg="#0f172a", fg=self.STATE_COLORS['init'], wraplength=460
        )
        self.status_label.pack(pady=(0, 12))

        btn_frame = tk.Frame(root, bg="#0f172a")
        btn_frame.pack(pady=8)

        for idx, (key, val) in enumerate(goals.items()):
            btn = tk.Button(
                btn_frame, text=val['name'], font=("Helvetica", 13, "bold"),
                width=16, height=2, bg="#1e293b", fg="white",
                activebackground="#334155", relief="flat",
                state="disabled",
                command=lambda k=key: self.on_select(k)
            )
            btn.grid(row=idx // 2, column=idx % 2, padx=8, pady=8)
            self.buttons[key] = btn

        tk.Label(
            root, text="Registro (Logs):", font=("Helvetica", 10),
            bg="#0f172a", fg="#94a3b8"
        ).pack(anchor="w", padx=16)

        self.log_box = scrolledtext.ScrolledText(
            root, height=10, bg="#1e293b", fg="#e2e8f0",
            font=("Courier", 9), relief="flat"
        )
        self.log_box.pack(fill="both", expand=True, padx=16, pady=(4, 16))
        self.log_box.configure(state="disabled")

        self.root.after(100, self.poll_status_queue)

    def on_select(self, key):
        self.set_buttons_enabled(False)
        self.command_queue.put(key)

    def set_buttons_enabled(self, enabled: bool):
        state = "normal" if enabled else "disabled"
        for btn in self.buttons.values():
            btn.config(state=state)

    def append_log(self, text):
        self.log_box.configure(state="normal")
        self.log_box.insert("end", text + "\n")
        self.log_box.see("end")
        self.log_box.configure(state="disabled")

    def poll_status_queue(self):
        try:
            while True:
                event = self.status_queue.get_nowait()
                kind = event[0]

                if kind == 'state':
                    _, level, text = event
                    self.status_label.config(text=text, fg=self.STATE_COLORS.get(level, "white"))
                    self.append_log(f"[{level.upper()}] {text}")
                    if level == 'ready':
                        self.set_buttons_enabled(True)
                    elif level == 'busy':
                        self.set_buttons_enabled(False)

                elif kind == 'log':
                    _, level, text = event
                    self.append_log(f"[{level}] {text}")

        except queue.Empty:
            pass

        self.root.after(100, self.poll_status_queue)

    def on_close(self):
        self.command_queue.put('SHUTDOWN')
        self.root.destroy()


def main(args=None):
    rclpy.init(args=args)
    rc = RobotCommander(namespace='turtlebot4')
    goals = define_goal_poses(rc)

    command_queue = queue.Queue()
    status_queue = queue.Queue()

    # Enviar logs a la GUI
    rc.status_callback = lambda level, msg: status_queue.put(('log', level, msg))

    # Iniciar hilo de ROS 2
    worker = threading.Thread(
        target=ros_worker, args=(rc, goals, command_queue, status_queue), daemon=True
    )
    worker.start()

    # Iniciar interfaz gráfica en el hilo principal
    root = tk.Tk()
    RobotGuiaGUI(root, goals, command_queue, status_queue)
    root.mainloop()

    # Limpieza al cerrar la ventana
    command_queue.put('SHUTDOWN')
    worker.join(timeout=5.0)
    rc.destroyNode()
    rclpy.shutdown()


if __name__ == "__main__":
    main()