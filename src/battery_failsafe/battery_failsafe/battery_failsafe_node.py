#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.duration import Duration

from std_msgs.msg import Bool, String, UInt8, Int32
from std_srvs.srv import SetBool, Trigger


def sensor_qos(depth=10, reliable=False):
    return QoSProfile(
        reliability=(ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT),
        history=HistoryPolicy.KEEP_LAST,
        depth=depth
    )


class BatteryFailsafeNode(Node):
    def __init__(self):
        super().__init__("battery_failsafe_node")

        # parametros base para vigilar bateria y conexion
        self.declare_parameter("battery_topic", "/tello/battery")
        self.declare_parameter("battery_type", "uint8")
        self.declare_parameter("conn_topic", "/tello/connection_status")
        self.declare_parameter("pub_critical_topic", "/battery/critical")
        self.declare_parameter("pub_status_topic", "/battery_failsafe/status")

        # umbrales y ritmos de publicacion
        self.declare_parameter("warn_threshold", 50)
        self.declare_parameter("critical_threshold", 30)
        self.declare_parameter("min_pct_for_takeoff", 30)
        self.declare_parameter("require_connection", True)
        self.declare_parameter("min_interval_s", 0.5)
        self.declare_parameter("grace_sec_before_land", 0.0)

        # servicios que controla en modo failsafe
        self.declare_parameter("srv_stream_set", "/tello/stream/set")
        self.declare_parameter("srv_stream_off_fallback", "/tello/stream_off")
        self.declare_parameter("srv_land", "/tello/land")

        # leer parametros de trabajo
        battery_topic = self.get_parameter("battery_topic").get_parameter_value().string_value
        battery_type = self.get_parameter("battery_type").get_parameter_value().string_value.strip().lower()
        conn_topic = self.get_parameter("conn_topic").get_parameter_value().string_value
        pub_critical_topic = self.get_parameter("pub_critical_topic").get_parameter_value().string_value
        pub_status_topic = self.get_parameter("pub_status_topic").get_parameter_value().string_value

        self.warn_th = int(self.get_parameter("warn_threshold").value)
        self.crit_th = int(self.get_parameter("critical_threshold").value)
        self.min_takeoff = int(self.get_parameter("min_pct_for_takeoff").value)
        self.require_conn = bool(self.get_parameter("require_connection").value)
        self.min_interval_s = float(self.get_parameter("min_interval_s").value)
        self.grace_before_land = float(self.get_parameter("grace_sec_before_land").value)

        self.srv_stream_set_name = self.get_parameter("srv_stream_set").get_parameter_value().string_value
        self.srv_stream_off_fb = self.get_parameter("srv_stream_off_fallback").get_parameter_value().string_value
        self.srv_land_name = self.get_parameter("srv_land").get_parameter_value().string_value

        # estado interno con memoria de bateria y banderas
        self._battery_pct = None
        self._connected = False
        self._critical_state = False
        self._landing_requested = False
        now = self.get_clock().now()
        interval = Duration(seconds=self.min_interval_s)
        self._last_pub_status_t = now - interval
        self._last_pub_critical_t = now - interval

        # publicaciones latched para que los nuevos nodos reciban estado actual
        latched_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.pub_critical = self.create_publisher(Bool, pub_critical_topic, latched_qos)
        self.pub_status = self.create_publisher(String, pub_status_topic, latched_qos)

        # suscripcion de bateria en el tipo configurado
        if battery_type == "int32":
            self.create_subscription(Int32, battery_topic, self._on_battery_int32, sensor_qos(depth=10))
        else:
            self.create_subscription(UInt8, battery_topic, self._on_battery_uint8, sensor_qos(depth=10))

        # conexion con qos fiable para detectar cortes
        self.create_subscription(Bool, conn_topic, self._on_connection, sensor_qos(depth=10, reliable=True))

        # clientes hacia driver para cortar stream y aterrizar
        self.cli_stream_set = self.create_client(SetBool, self.srv_stream_set_name)
        self.cli_stream_off_fb = self.create_client(Trigger, self.srv_stream_off_fb)
        self.cli_land = self.create_client(Trigger, self.srv_land_name)

        # servicio que autoriza despegues segun bateria
        self.srv_ok_to_takeoff = self.create_service(Trigger, "/battery_failsafe/ok_to_takeoff", self._srv_ok_to_takeoff)

        self.get_logger().info(
            f"[battery_failsafe] warn={self.warn_th}%, critical={self.crit_th}%, "
            f"min_takeoff={self.min_takeoff}%, battery=({battery_topic}:{battery_type}), "
            f"conn={conn_topic}, stream_set={self.srv_stream_set_name}, "
            f"stream_off_fb={self.srv_stream_off_fb}, land={self.srv_land_name}"
        )

        # publicar estado inicial estable
        self._publish_status("ok")
        self._publish_critical(False)

    # ---------- Callbacks ----------
    def _on_battery_uint8(self, msg: UInt8):
        self._on_battery_pct(int(msg.data))

    def _on_battery_int32(self, msg: Int32):
        self._on_battery_pct(int(msg.data))

    def _on_battery_pct(self, pct: int):
        self._battery_pct = max(0, min(100, pct))
        self._evaluate()

    def _on_connection(self, msg: Bool):
        self._connected = bool(msg.data)
        if not self._connected:
            self._publish_status("disconnected")

    # validacion de servicio de despegue
    def _srv_ok_to_takeoff(self, _req, _resp):
        res = Trigger.Response()
        if self.require_conn and not self._connected:
            res.success = False
            res.message = "no conectado"
            return res
        if self._battery_pct is None:
            res.success = False
            res.message = "sin lectura de bateria"
            return res
        if self._battery_pct < self.min_takeoff:
            res.success = False
            res.message = f"bateria {self._battery_pct}% < minimo {self.min_takeoff}%"
            return res
        res.success = True
        res.message = f"ok para despegar bateria {self._battery_pct}%"
        return res

    # logica central de evaluacion de bateria
    def _evaluate(self):
        if self._battery_pct is None:
            return
        if self.require_conn and not self._connected:
            return

        # nivel critico inicia aterrizaje
        if self._battery_pct <= self.crit_th:
            if not self._critical_state:
                self._critical_state = True
                self._publish_critical(True)
                self._publish_status(f"critical_{self.crit_th}")
                self._start_landing_sequence()
            else:
                # repetir marca para nodos nuevos
                self._publish_critical(True)
            return

        # nivel de aviso mantiene vuelo sin aterrizar
        if self._battery_pct <= self.warn_th:
            self._critical_state = False
            self._publish_critical(False)
            self._publish_status(f"warn_{self.warn_th}")
            return

        # nivel seguro libera la bandera de aterrizaje
        self._critical_state = False
        self._landing_requested = False
        self._publish_critical(False)
        self._publish_status("ok")

    # control de envio de mensajes de estado
    def _publish_status(self, text: str):
        now = self.get_clock().now()
        if (now - self._last_pub_status_t) < Duration(seconds=self.min_interval_s):
            return
        self._last_pub_status_t = now
        self.pub_status.publish(String(data=text))

    def _publish_critical(self, state: bool):
        now = self.get_clock().now()
        if (state != self._critical_state) or ((now - self._last_pub_critical_t) >= Duration(seconds=self.min_interval_s)):
            self._last_pub_critical_t = now
            self.pub_critical.publish(Bool(data=state))

    # secuencia asincrona para aterrizar
    def _start_landing_sequence(self):
        if self._landing_requested:
            return
        self._landing_requested = True

        # usar hilo para evitar bloqueos de callback
        threading.Thread(target=self._landing_thread, daemon=True).start()

    def _landing_thread(self):
        # apagar stream y aterrizar sin bloquear hilos ros
        if not self._call_stream_set(False):
            self._call_stream_off_fallback()

        if self.grace_before_land > 0:
            time.sleep(self.grace_before_land)

        self._call_land()

        self._publish_status(f"critical_{self.crit_th}_landing")

    # helpers para clientes de servicio
    def _call_stream_set(self, enable: bool) -> bool:
        try:
            if not self.cli_stream_set.service_is_ready():
                self.cli_stream_set.wait_for_service(timeout_sec=0.8)
            if not self.cli_stream_set.service_is_ready():
                self.get_logger().warn(f"stream_set no disponible: {self.srv_stream_set_name}")
                return False

            req = SetBool.Request()
            req.data = bool(enable)

            fut = self.cli_stream_set.call_async(req)
            def _done(f):
                try:
                    res = f.result()
                    self.get_logger().info(f"stream_set({enable}) -> success={res.success} msg='{res.message}'")
                except Exception as e:
                    self.get_logger().warn(f"stream_set({enable}) error en callback: {e}")

            fut.add_done_callback(_done)
            return True  # solicitud enviada OK
        except Exception as e:
            self.get_logger().warn(f"stream_set({enable}) error: {e}")
            return False


    def _call_stream_off_fallback(self):
        try:
            if not self.cli_stream_off_fb.service_is_ready():
                self.cli_stream_off_fb.wait_for_service(timeout_sec=0.8)
            if not self.cli_stream_off_fb.service_is_ready():
                self.get_logger().warn(f"stream_off no disponible: {self.srv_stream_off_fb}")
                return False
            req = Trigger.Request()
            fut = self.cli_stream_off_fb.call_async(req)
            fut.add_done_callback(lambda _f: self.get_logger().info("stream_off fallback enviado"))
            return True
        except Exception as e:
            self.get_logger().warn(f"stream_off fallback error: {e}")
            return False

    def _call_land(self):
        try:
            if not self.cli_land.service_is_ready():
                self.cli_land.wait_for_service(timeout_sec=1.0)
            if not self.cli_land.service_is_ready():
                self.get_logger().warn(f"land no disponible: {self.srv_land_name}")
                return False
            req = Trigger.Request()
            fut = self.cli_land.call_async(req)
            fut.add_done_callback(lambda _f: self.get_logger().info("land enviado"))
            return True
        except Exception as e:
            self.get_logger().warn(f"land error: {e}")
            return False


def main():
    rclpy.init()
    node = BatteryFailsafeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

