#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import Empty, Bool

class DroneConnector(Node):
    """Nodo que emite el estado de conexion usando una fuente Bool o pulsos Empty."""

    def __init__(self):
        super().__init__('drone_connector')

        # Configura parametros de entrada y salida
        self.declare_parameter('source_topic', '/tello/connected')
        self.declare_parameter('source_type', 'bool')
        self.declare_parameter('status_topic', '/tello/connection_status')
        self.declare_parameter('timeout_sec', 3.0)
        self.declare_parameter('status_rate_hz', 1.0)

        source_topic   = self.get_parameter('source_topic').get_parameter_value().string_value
        source_type    = self.get_parameter('source_type').get_parameter_value().string_value.lower().strip()
        status_topic   = self.get_parameter('status_topic').get_parameter_value().string_value
        self.timeout   = Duration(seconds=float(self.get_parameter('timeout_sec').value))
        status_period  = 1.0 / max(0.1, float(self.get_parameter('status_rate_hz').value))

        # Ajusta perfiles QoS para la fuente y el estado
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        status_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # Publica el estado de conexion con durabilidad transiente
        self.pub_status = self.create_publisher(Bool, status_topic, status_qos)

        # Inicializa estado interno para monitorear la vida del enlace
        self._last_life: Time = Time()
        self._timer = None

        if source_type == 'bool':
            # En modo bool replica la senal sin procesarla
            self.create_subscription(Bool, source_topic, self._on_connected_bool, sensor_qos)
            self.get_logger().info(f"[connector] Modo BOOL: {source_topic} (Bool) -> {status_topic} (Bool)")
        else:
            # En modo empty convierte pulsos en un estado segun el timeout
            self.create_subscription(Empty, source_topic, self._on_state_pulse, sensor_qos)
            self._timer = self.create_timer(status_period, self._tick_status_from_timeout)
            self.get_logger().info(
                f"[connector] Modo EMPTY: {source_topic} (Empty) -> {status_topic} (Bool) "
                f"con timeout={self.timeout.nanoseconds/1e9:.1f}s @ {1.0/status_period:.1f} Hz"
            )

    def _on_connected_bool(self, msg: Bool):
        # Publica el estado de conexion recibido desde la fuente Bool
        self.pub_status.publish(msg)

    def _on_state_pulse(self, _msg: Empty):
        # Registra el instante del ultimo pulso para evaluar el timeout
        self._last_life = self.get_clock().now()

    def _tick_status_from_timeout(self):
        if self._last_life.nanoseconds == 0:
            connected = False
        else:
            connected = (self.get_clock().now() - self._last_life) < self.timeout
        # Difunde el resultado calculado por el temporizador
        self.pub_status.publish(Bool(data=connected))

def main(args=None):
    rclpy.init(args=args)
    node = DroneConnector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

