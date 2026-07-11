import rclpy
import os
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory
from std_srvs.srv import Trigger

class YoloDetectorViz(Node):
    def __init__(self):
        super().__init__('yolo_detector_viz')

        # Puente para convertir mensajes Image de ros a imágenes de opencv
        self.bridge = CvBridge()

        # Carga el modelo de YOLOv8 con los pesos entrenados para la deteccion de la señal NoPasar
        package_share_directory = get_package_share_directory('yolobot')
        model_path = os.path.join(package_share_directory, 'models', 'best-noentry.pt')
        self.model = YOLO(model_path)
        self.get_logger().info(f"=== CLASES DEL MODELO: {self.model.names} ===")

        self.signal_detected = False
        self.latest_msg = None

        # booleano para activar la deteccion cuando mande robot_guia_gui
        self.detection_active = False

        #suscripción al topic de la cámara RGB del robot
        self.image_sub = self.create_subscription(
            Image,
            '/oakd/rgb/preview/image_raw',
            self.image_callback,
            10
        )

        # Publicador para las imágenes anotadas
        self.viz_pub = self.create_publisher(Image, '/yolo/visualization', 10)

        # servicio que el robot_guia llama para consultar el resultado
        self.check_door_srv = self.create_service(
            Trigger,
            '/yolo/check_door',
            self.analyze_door_callback
        )

        # servicios que el robot_guia llama para activar y desactivar la detección
        self.enable_detection_srv = self.create_service(
            Trigger,
            '/yolo/enable_detection',
            self.enable_detection_callback
        )
        self.disable_detection_srv = self.create_service(
            Trigger,
            '/yolo/disable_detection',
            self.disable_detection_callback
        )

        self.get_logger().info("YOLO detector with visualization started (detección desactivada por defecto)")

    def image_callback(self, msg):
        # callback de la camara que se ejecuta en cada imagen que recibe
        self.latest_msg = msg

        # si no está activa la detección no ejecuta nada
        if not self.detection_active:
            return

        try:
            # se convierte el mensaje de ros al frame BGR de opencv
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

            # se ejecuta la detección
            results = self.model(cv_image, classes=[0], conf=0.8, iou=0.45, verbose=False)

            # se evalúa si se ha detectado el objeto
            num_boxes = len(results[0].boxes)
            self.signal_detected = num_boxes > 0

            # se dibujan los bounding boxes en la salida
            annotated = results[0].plot()
            msg_out = self.bridge.cv2_to_imgmsg(annotated, 'bgr8')
            self.viz_pub.publish(msg_out)

        except Exception as e:
            self.get_logger().error(f"Error procesando frame: {str(e)}")

    def enable_detection_callback(self, request, response):
        # activación de la detección cuando robot_guia_gui llega al punto de entrada 
        self.signal_detected = False
        self.detection_active = True
        self.get_logger().info("Detección YOLO ACTIVADA por robot_guia_gui.py.")
        response.success = True
        response.message = "Detección activada."
        return response

    def disable_detection_callback(self, request, response):
        # desactivación de la detección continua para bajar la carga computacional
        self.detection_active = False
        self.get_logger().info("Detección YOLO DESACTIVADA por robot_guia_gui.py.")
        response.success = True
        response.message = "Detección desactivada."
        return response

    def analyze_door_callback(self, request, response):
        # callback del servicio para responder con el estado de la puerta
        if self.latest_msg is None:
            response.success = False
            response.message = "Error: Aún no se han recibido imágenes de la cámara."
            return response

        if not self.detection_active:
            response.success = False
            response.message = "Error: La detección no está activa. Llama primero a /yolo/enable_detection."
            return response

        self.get_logger().info("Servicio consultado. Respondiendo con el estado actual de la puerta...")

        if self.signal_detected:
            response.success = False
            response.message = "Señal de NO PASAR detectada. Puerta bloqueada."
        else:
            response.success = True
            response.message = "No se detecta señal. Vía libre."

        return response


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorViz()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()