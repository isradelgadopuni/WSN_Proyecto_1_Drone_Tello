#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from std_msgs.msg import Bool, String, UInt8, Int32
from geometry_msgs.msg import Vector3


def qos_best_effort(depth=10):
    # Ajusta QoS para datos publicados por el driver
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth
    )

def qos_reliable(depth=10):
    # Define QoS confiable para los indicadores de estado
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth
    )

def qos_latched():
    # Genera QoS latched para el resumen opcional
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        durability=DurabilityPolicy.TRANSIENT_LOCAL
    )


class TelemetryMonitor(Node):
    """Nodo que reporta conexion, video y modulos auxiliares."""

    def __init__(self):
        super().__init__('telemetry_monitor')

        # Configuracion de parametros de salida
        self.declare_parameter('print_rate_hz', 1.0)
        self.declare_parameter('stale_sec', 3.0)
        self.declare_parameter('pub_status', True)
        self.declare_parameter('status_topic', '/telemetry_monitor/status')

        rate_hz = float(self.get_parameter('print_rate_hz').value)
        self.stale_sec = float(self.get_parameter('stale_sec').value)
        self.publish_status = bool(self.get_parameter('pub_status').value)
        self.status_topic = self.get_parameter('status_topic').get_parameter_value().string_value

        # Variables de estado con marcas de tiempo
        now = self.get_clock().now()
        self.conn: Optional[bool] = None;     self.t_conn = now
        self.stream_on: Optional[bool] = None;self.t_stream = now
        self.bat: Optional[int] = None;       self.t_bat = now
        self.h_cm: Optional[int] = None;      self.t_h = now
        self.vgx: Optional[float] = None
        self.vgy: Optional[float] = None
        self.vgz: Optional[float] = None;     self.t_vel = now

        self.failsafe: Optional[str] = None;  self.t_failsafe = now
        self.mission: Optional[str] = None;   self.t_mission = now
        self.viewer: Optional[str] = None;    self.t_viewer = now
        self.detector: Optional[str] = None;  self.t_detector = now

        # Configuracion de suscripciones

        # Datos del driver con QoS best effort
        self.create_subscription(UInt8, '/tello/battery', self._on_battery, qos_best_effort())
        self.create_subscription(Int32, '/tello/height', self._on_height, qos_best_effort())
        self.create_subscription(Vector3, '/tello/velocity', self._on_velocity, qos_best_effort())

        # Estado del stream recibido en el topico principal
        # Texto libre reportado por el driver sobre el stream
        self.create_subscription(String, '/tello/stream/state', self._on_stream_str, qos_best_effort())

        # Estado alterno del stream habilitado via parametro
        self.declare_parameter('stream_bool_topic', '')
        stream_bool_topic = self.get_parameter('stream_bool_topic').get_parameter_value().string_value
        if stream_bool_topic:
            self.create_subscription(Bool, stream_bool_topic, self._on_stream_bool_opt, qos_best_effort())

        # Estados auxiliares con QoS confiable
        self.create_subscription(Bool, '/tello/connection_status', self._on_conn, qos_reliable())
        self.create_subscription(String, '/battery_failsafe/status', self._on_failsafe, qos_reliable())
        self.create_subscription(String, '/mission/status', self._on_mission, qos_reliable())
        self.create_subscription(String, '/video_viewer/status', self._on_viewer, qos_reliable())
        self.create_subscription(String, '/object_detector/status', self._on_detector, qos_reliable())


        # Publicador opcional del estado compacto
        self.pub_status = None
        if self.publish_status:
            self.pub_status = self.create_publisher(String, self.status_topic, qos_latched())
            self.pub_status.publish(String(data='running'))

        # Temporizador que imprime y publica el resumen
        period = 1.0 / max(0.1, rate_hz)
        self.create_timer(period, self._print_tick)

        self.get_logger().info('[telemetry_monitor] listo (imprime telemetria a ~1 Hz)')

    # Seccion de callbacks de suscripcion
    def _on_conn(self, msg: Bool):
        self.conn = bool(msg.data); self.t_conn = self.get_clock().now()

    def _on_stream_str(self, msg: String):
        txt = (msg.data or '').strip().lower()
        if 'stream:off' in txt:
            self.stream_on = False
        elif 'stream:on' in txt or 'video_fps' in txt:
            self.stream_on = True
        # probe:ok no altera el estado del stream
        self.t_stream = self.get_clock().now()

    def _on_stream_bool_opt(self, msg: Bool):
        # Solo activo si se declara stream_bool_topic
        self.stream_on = bool(msg.data)
        self.t_stream = self.get_clock().now()

    def _on_battery(self, msg: UInt8):
        self.bat = int(msg.data); self.t_bat = self.get_clock().now()

    def _on_height(self, msg: Int32):
        self.h_cm = int(msg.data); self.t_h = self.get_clock().now()

    def _on_velocity(self, msg: Vector3):
        # Driver entrega vgx vgy vgz como Vector3 en cm por segundo
        self.vgx = float(msg.x); self.vgy = float(msg.y); self.vgz = float(msg.z)
        self.t_vel = self.get_clock().now()

    def _on_failsafe(self, msg: String):
        self.failsafe = msg.data or ''; self.t_failsafe = self.get_clock().now()

    def _on_mission(self, msg: String):
        self.mission = msg.data or ''; self.t_mission = self.get_clock().now()

    def _on_viewer(self, msg: String):
        self.viewer = msg.data or ''; self.t_viewer = self.get_clock().now()

    def _on_detector(self, msg: String):
        self.detector = msg.data or ''; self.t_detector = self.get_clock().now()

    # Rutinas para formatear la salida
    def _fresh(self, t_last) -> bool:
        if t_last is None:
            return False
        age = (self.get_clock().now() - t_last).nanoseconds / 1e9
        return age <= self.stale_sec

    def _fmt(self, val, ok=True, suffix=''):
        if val is None or not ok:
            return '--'
        return f'{val}{suffix}'

    def _print_tick(self):
        ok_conn   = self._fresh(self.t_conn)
        ok_stream = self._fresh(self.t_stream)
        ok_bat    = self._fresh(self.t_bat)
        ok_h      = self._fresh(self.t_h)
        ok_vel    = self._fresh(self.t_vel)
        ok_fs     = self._fresh(self.t_failsafe)
        ok_ms     = self._fresh(self.t_mission)
        ok_vw     = self._fresh(self.t_viewer)
        ok_det    = self._fresh(self.t_detector)

        conn_s   = 'OK' if (self.conn is True and ok_conn) else ('NO' if (self.conn is False and ok_conn) else '--')
        stream_s = 'ON' if (self.stream_on is True and ok_stream) else ('OFF' if (self.stream_on is False and ok_stream) else '--')
        bat_s    = self._fmt(self.bat, ok_bat, '%')
        h_s      = self._fmt(self.h_cm, ok_h, ' cm')

        if ok_vel and None not in (self.vgx, self.vgy, self.vgz):
            vel_s = f'{int(self.vgx)}/{int(self.vgy)}/{int(self.vgz)} cm/s'
        else:
            vel_s = '--'

        fs_s  = self.failsafe if ok_fs else '--'
        ms_s  = self.mission if ok_ms else '--'
        vw_s  = self.viewer  if ok_vw else '--'
        det_s = self.detector if ok_det else '--'

        # Construye un panel textual legible para la consola
        lines = [
            '+-- Telemetria / Estados --------------------------------------+',
            f'| Conexion: {conn_s}   | Stream: {stream_s}',
            f'| Bateria:  {bat_s}    | Altura: {h_s}    | Vel: {vel_s}',
            f'| Failsafe: {fs_s}',
            f'| Mision:   {ms_s}',
            f'| Viewer:   {vw_s}',
            f'| Detector: {det_s}',
            '+---------------------------------------------------------------',
        ]
        print('\n'.join(lines))

        # Publica el resumen si el topico esta habilitado
        if self.pub_status:
            # Formato compacto con todos los datos relevantes
            compact = (
                f'{conn_s}/{stream_s} {bat_s} {h_s} '
                f'v={vel_s} fs={fs_s} ms={ms_s} vw={vw_s} det={det_s}'
            )
            self.pub_status.publish(String(data=compact))


def main():
    rclpy.init()
    node = TelemetryMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

