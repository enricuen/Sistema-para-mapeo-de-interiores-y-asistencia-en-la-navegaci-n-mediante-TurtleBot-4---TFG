#!/usr/bin/env python3
from enum import Enum
import time
import threading
import queue
import tkinter as tk
from tkinter import scrolledtext

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Quaternion, PoseStamped, PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import Spin, NavigateToPose
#from turtle_tf2_py.turtle_tf2_broadcaster import quaternion_from_euler
import math
from irobot_create_msgs.action import Dock, Undock
from irobot_create_msgs.msg import DockStatus
from std_srvs.srv import Trigger

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration as rclpyDuration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data


class TaskResult(Enum):
    UNKNOWN = 0
    SUCCEEDED = 1
    CANCELED = 2
    FAILED = 3


# Define la pose inicial del robot (base)
# Permite que el robot pueda volver a la base al final
INITIAL_POSE = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}

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

        # Callback que usa la GUI para el registro
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
        self.yolo_check_client = self.create_client(Trigger, '/yolo/check_door')
        self.yolo_enable_client = self.create_client(Trigger, '/yolo/enable_detection')
        self.yolo_disable_client = self.create_client(Trigger, '/yolo/disable_detection')

        self.get_logger().info(f"Robot commander has been initialized!")
        
    def destroyNode(self):
        self.nav_to_pose_client.destroy()
        super().destroy_node()     

    def _call_trigger(self, client, service_name: str):
        # Lamada a servicio Trigger 
        while not client.wait_for_service(timeout_sec=1.0):
            self.info(f"Servicio '{service_name}' no disponible, esperando...")
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def enable_door_detection(self):
        # Activación de la detección con YOLO
        self.info("Activando detección YOLO en el punto de entrada...")
        result = self._call_trigger(self.yolo_enable_client, '/yolo/enable_detection')
        if result is None:
            self.error("Fallo al activar la detección YOLO.")
        else:
            self.info(f"Respuesta: {result.message}")

    def disable_door_detection(self):
        # Desactivación de la detección con YOLO
        result = self._call_trigger(self.yolo_disable_client, '/yolo/disable_detection')
        if result is None:
            self.warn("Fallo al desactivar la detección YOLO (se continúa de todos modos).")
        else:
            self.info(f"Respuesta: {result.message}")

    def check_door_vision(self, wait_time: float = 3.0) -> bool:
        # Activa la detección , espera a tener frames estables y devuelve respuesta
        # Activación de la detección porque el robot está en el punto de entrada
        self.enable_door_detection()

        # El robot se espera para que se estabilice la cámara 
        self.info(f"Robot detenido. Esperando {wait_time} segundos para estabilizar la cámara y procesar frames limpios...")
        time.sleep(wait_time)

        # Resultado final del estado de la puerta
        self.info("Analizando puerta con la cámara...")
        result = self._call_trigger(self.yolo_check_client, '/yolo/check_door')

        # Desactivación de la detección
        self.disable_door_detection()

        if result is not None:
            # success=True es "Vía libre / No hay señal", success=False es "Señal detectada"
            self.info(f"Respuesta de YOLO: {result.message}")
            return result.success
        else:
            self.error("Fallo al comunicarse con el detector YOLO.")
            return False

    def goToPose(self, pose, behavior_tree=''):
        """Send a `NavToPose` action request."""
        self.debug("Waiting for 'NavigateToPose' action server")
        while not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.info("'NavigateToPose' action server not available, waiting...")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        goal_msg.behavior_tree = behavior_tree

        self.info('Navigating to goal: ' + str(pose.pose.position.x) + ' ' +
                  str(pose.pose.position.y) + '...')
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg,
                                                                   self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error('Goal to ' + str(pose.pose.position.x) + ' ' +
                       str(pose.pose.position.y) + ' was rejected!')
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

        self.info(f'Spinning to angle {goal_msg.target_yaw}....')
        send_goal_future = self.spin_client.send_goal_async(goal_msg, self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error('Spin request was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True
    
    def undock(self):
        """Perform Undock action."""
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
        """
        Get status of Undock action.

        :return: ``True`` if undocked, ``False`` otherwise.
        """
        if self.undock_result_future is None or not self.undock_result_future:
            return True

        rclpy.spin_until_future_complete(self, self.undock_result_future, timeout_sec=0.1)

        if self.undock_result_future.result():
            self.undock_status = self.undock_result_future.result().status
            if self.undock_status != GoalStatus.STATUS_SUCCEEDED:
                self.info(f'Goal with failed with status code: {self.status}')
                return True
        else:
            return False

        self.info('Undock succeeded')
        return True

    def cancelTask(self):
        """Cancel pending task request of any type."""
        self.info('Canceling current task.')
        if self.result_future:
            future = self.goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, future)
        return

    def isTaskComplete(self):
        """Check if the task request of any type is complete yet."""
        if not self.result_future:
            # task was cancelled or completed
            return True
        rclpy.spin_until_future_complete(self, self.result_future, timeout_sec=0.10)
        if self.result_future.result():
            self.status = self.result_future.result().status
            if self.status != GoalStatus.STATUS_SUCCEEDED:
                self.debug(f'Task with failed with status code: {self.status}')
                return True
        else:
            # Timed out, still processing, not complete yet
            return False

        self.debug('Task succeeded!')
        return True

    def getFeedback(self):
        """Get the pending action feedback message."""
        return self.feedback

    def getResult(self):
        """Get the pending action result message."""
        if self.status == GoalStatus.STATUS_SUCCEEDED:
            return TaskResult.SUCCEEDED
        elif self.status == GoalStatus.STATUS_ABORTED:
            return TaskResult.FAILED
        elif self.status == GoalStatus.STATUS_CANCELED:
            return TaskResult.CANCELED
        else:
            return TaskResult.UNKNOWN

    def waitUntilNav2Active(self, navigator='bt_navigator', localizer='amcl'):
        """Block until the full navigation system is up and running."""
        self._waitForNodeToActivate(localizer)
        if not self.initial_pose_received:
            time.sleep(1)
        self._waitForNodeToActivate(navigator)
        self.info('Nav2 is ready for use!')
        return

    def _waitForNodeToActivate(self, node_name):
        # Waits for the node within the tester namespace to become active
        self.debug(f'Waiting for {node_name} to become active..')
        node_service = f'{node_name}/get_state'
        state_client = self.create_client(GetState, node_service)
        while not state_client.wait_for_service(timeout_sec=1.0):
            self.info(f'{node_service} service not available, waiting...')

        req = GetState.Request()
        state = 'unknown'
        while state != 'active':
            self.debug(f'Getting {node_name} state...')
            future = state_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                state = future.result().current_state.label
                self.debug(f'Result of get_state: {state}')
            time.sleep(2)
        return
    
    def YawToQuaternion(self, angle_z = 0.):
        # Conversión directa de Yaw (radianes) a Quaternion
        quat_msg = Quaternion()
        quat_msg.x = 0.0
        quat_msg.y = 0.0
        quat_msg.z = math.sin(angle_z / 2.0)
        quat_msg.w = math.cos(angle_z / 2.0)
        
        return quat_msg

    def _amclPoseCallback(self, msg):
        self.debug('Received amcl pose')
        self.initial_pose_received = True
        self.current_pose = msg.pose
        return

    def _feedbackCallback(self, msg):
        self.debug('Received action feedback message')
        self.feedback = msg.feedback
        return
    
    def _dockCallback(self, msg: DockStatus):
        self.is_docked = msg.is_docked

    def setInitialPose(self, pose):
        msg = PoseWithCovarianceStamped()
        msg.pose.pose = pose
        msg.header.frame_id = self.pose_frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        self.info('Publishing Initial Pose')
        self.initial_pose_pub.publish(msg)
        return

    def info(self, msg):
        self.get_logger().info(msg)
        self._emit_status('info', msg)
        return

    def warn(self, msg):
        self.get_logger().warn(msg)
        self._emit_status('warn', msg)
        return

    def error(self, msg):
        self.get_logger().error(msg)
        self._emit_status('error', msg)
        return

    def debug(self, msg):
        self.get_logger().debug(msg)
        return

    def _emit_status(self, level, msg):
        if self.status_callback is not None:
            try:
                self.status_callback(level, msg)
            except Exception:
                pass


def make_pose(rc: RobotCommander, x: float, y: float, yaw: float) -> PoseStamped:
    """Helper to build a PoseStamped from (x, y, yaw)."""
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = rc.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation = rc.YawToQuaternion(yaw)
    return pose

def get_home_pose(rc: RobotCommander) -> PoseStamped:
    # pose inicial del robot usada para volver a la base en cada ejecución
    return make_pose(rc, INITIAL_POSE['x'], INITIAL_POSE['y'], INITIAL_POSE['yaw'])


def define_goal_poses(rc: RobotCommander) -> dict:

    goals = {}

    # Aula 1
    goals[1] = {
        'name': 'Aula 1',
        # Punto de entrada: delante de la puerta del Aula 1
        'entry_point':    make_pose(rc, -3.20,  -0.30, 1.57),
        # Destino final dentro del aula
        'goal':           make_pose(rc, -4.20,  2.30, 1.57),
        # Punto alternativo si la puerta está cerrada
        'fallback_point': make_pose(rc,  11.75,  0.15,  0.0),
    }


    # Aula 2
    goals[2] = {
        'name': 'Aula 2',
        'entry_point':    make_pose(rc, -3.25, 0.70, -1.57),
        'goal':           make_pose(rc, -4.30, -2.00, -1.57),
        'fallback_point': make_pose(rc,  11.75,  0.15,  0.0),
    }

    # Aula 3
    goals[3] = {
        'name': 'Aula 3',
        'entry_point':    make_pose(rc,  4.43,  -0.32,  1.57),
        'goal':           make_pose(rc,  3.00,  2.40,  1.57),
        'fallback_point': make_pose(rc,  11.75,  0.15,  0.0),
    }

    # Aula 4
    goals[4] = {
        'name': 'Aula 4',
        'entry_point':    make_pose(rc,  4.50, 0.90,  -1.57),
        'goal':           make_pose(rc,  3.00, -2.00,  -1.57),
        'fallback_point': make_pose(rc,  11.75,  0.15,  0.0),
    }

    return goals

def navigate_and_wait(rc: RobotCommander, pose: PoseStamped, description: str) -> TaskResult:
    """Send a navigation goal, block until complete, and return the result."""
    # Refresh timestamp right before sending
    pose.header.stamp = rc.get_clock().now().to_msg()
 
    rc.goToPose(pose)
 
    while not rc.isTaskComplete():
        rc.info(f"Navegando: {description}...")
        time.sleep(1)
 
    return rc.getResult()

def return_home(rc: RobotCommander, home_pose: PoseStamped, status_queue: "queue.Queue"):
    # Navegación de vuelta a la base y reporte de estado para la interfaz
    status_queue.put(('state', 'busy', "Volviendo al punto inicial..."))
    result = navigate_and_wait(rc, home_pose, "punto inicial")

    if result == TaskResult.SUCCEEDED:
        rc.info("De vuelta en el punto inicial.")
        status_queue.put(('state', 'ready', "De vuelta en el punto inicial. Elige un aula."))
    elif result == TaskResult.CANCELED:
        rc.warn("El regreso al punto inicial fue cancelado.")
        status_queue.put(('state', 'error', "Regreso al punto inicial cancelado."))
    elif result == TaskResult.FAILED:
        rc.error("No se pudo volver al punto inicial.")
        status_queue.put(('state', 'error', "No se pudo volver al punto inicial."))
    else:
        rc.warn("Resultado desconocido al volver al punto inicial.")
        status_queue.put(('state', 'error', "Resultado desconocido al volver al punto inicial."))


def run_navigation_cycle(rc: RobotCommander, aula: dict, aula_name: str, home_pose: PoseStamped, status_queue: "queue.Queue"):
    # Ejecución de un ciclo completo de ir al punto de entrada, comprobar el estado de la puerta y entrar o ir al punto alternativo

    status_queue.put(('state', 'busy', f"Navegando al punto de entrada de {aula_name}..."))
    result = navigate_and_wait(rc, aula['entry_point'], f"punto de entrada {aula_name}")

    if result != TaskResult.SUCCEEDED:
        rc.error(f"No se pudo alcanzar el punto de entrada de {aula_name}. Resultado: {result}")
        status_queue.put(('state', 'error', f"No se pudo llegar al punto de entrada de {aula_name}."))
        return

    rc.info(f"Llegando al punto de entrada de {aula_name}.")
    status_queue.put(('state', 'busy', f"Comprobando la puerta de {aula_name}..."))

    door_open = rc.check_door_vision()

    if door_open:
        status_queue.put(('state', 'busy', f"Puerta abierta. Entrando en {aula_name}..."))
        result = navigate_and_wait(rc, aula['goal'], f"interior {aula_name}")

        if result == TaskResult.SUCCEEDED:
            rc.info(f"¡Destino alcanzado! → {aula_name}")
            status_queue.put(('state', 'ready', f"Llegado a {aula_name}."))
            return_home(rc, home_pose, status_queue)
        elif result == TaskResult.CANCELED:
            rc.warn("La navegación fue cancelada.")
            status_queue.put(('state', 'error', "Navegación cancelada."))
        elif result == TaskResult.FAILED:
            rc.error("La navegación falló.")
            status_queue.put(('state', 'error', "La navegación falló."))
        else:
            rc.warn("Resultado desconocido.")
            status_queue.put(('state', 'error', "Resultado desconocido."))
    else:
        rc.warn("Puerta cerrada. Dirigiéndose al punto alternativo...")
        status_queue.put(('state', 'busy', "Puerta cerrada. Yendo al punto alternativo..."))
        result = navigate_and_wait(rc, aula['fallback_point'], "punto alternativo")

        if result == TaskResult.SUCCEEDED:
            rc.info("Llegado al punto alternativo.")
            status_queue.put(('state', 'ready', "Puerta cerrada. Llegado a punto alternativo."))
            return_home(rc, home_pose, status_queue)
        elif result == TaskResult.CANCELED:
            rc.warn("La navegación al punto alternativo fue cancelada.")
            status_queue.put(('state', 'error', "Navegación al punto alternativo cancelada."))
        elif result == TaskResult.FAILED:
            rc.error("No se pudo alcanzar el punto alternativo.")
            status_queue.put(('state', 'error', "No se pudo alcanzar el punto alternativo."))
        else:
            rc.warn("Resultado desconocido.")
            status_queue.put(('state', 'error', "Resultado desconocido."))


def ros_worker(rc: RobotCommander, goals: dict, command_queue: "queue.Queue", status_queue: "queue.Queue"):
    # Hilo secundario donde se ejecuta la lógica de navegación. La interfaz intercambia datos por colas con rclpy
    status_queue.put(('state', 'init', "Inicializando sistema de navegación..."))

    # Punto de origen del robot
    home_pose = get_home_pose(rc)
    rc.setInitialPose(home_pose.pose)

    rc.waitUntilNav2Active()
    
    
    while rc.is_docked is None:
        rclpy.spin_once(rc, timeout_sec=0.5)
    # Si está en el dock, sacarlo
    if rc.is_docked:
        rc.undock()

    status_queue.put(('state', 'ready', "Listo. Elige un aula."))

    # Bucle principal que procesa las peticiones de la interfaz y queda a la espera cuando termina
    while rclpy.ok():
        try:
            selected = command_queue.get(timeout=0.5)
        except queue.Empty:
            continue

        if selected == 'SHUTDOWN':
            break

        aula = goals[selected]
        run_navigation_cycle(rc, aula, aula['name'], home_pose, status_queue)
        status_queue.put(('state', 'ready', "Listo. Elige un aula."))


class RobotGuiaGUI:
    # Configuración del panel de control: botones para elegir aula, estado actual y el registro
    STATE_COLORS = {
        'init':  '#2563eb',   # azul
        'busy':  '#d97706',   # naranja
        'ready': '#16a34a',   # verde
        'error': '#dc2626',   # rojo
    }

    def __init__(self, root, goals: dict, command_queue: "queue.Queue", status_queue: "queue.Queue"):
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
            root, text="Registro:", font=("Helvetica", 10),
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

    # Inicializa el sistema de comunicaciones de ros
    rclpy.init(args=args)

    rc = RobotCommander()
    goals = define_goal_poses(rc)

    # Cola para enviar comandos desde la interfaz al hilo de ros
    command_queue = queue.Queue()
    status_queue = queue.Queue()

    # Se mandan los info, error y warn a la interfaz
    rc.status_callback = lambda level, msg: status_queue.put(('log', level, msg))

    # Hilo independiente para ejecución de ros
    worker = threading.Thread(
        target=ros_worker, args=(rc, goals, command_queue, status_queue), daemon=True
    )
    worker.start()

    # Creación de la ventana principal de la interfaz gráfica
    root = tk.Tk()
    # Inicialización de la GUI
    RobotGuiaGUI(root, goals, command_queue, status_queue)
    root.mainloop()

    # Cierre limpio y ordenado de la aplicación
    command_queue.put('SHUTDOWN')
    worker.join(timeout=5.0)
    rc.destroyNode()
    rclpy.shutdown()


if __name__ == "__main__":
    main()