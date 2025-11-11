#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
os.environ.setdefault("QT_QPA_PLATFORM", "xcb")
os.environ.setdefault("LIBGL_ALWAYS_SOFTWARE", "1")
os.environ.setdefault("XDG_RUNTIME_DIR", "/tmp")

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.time import Time
from rclpy.duration import Duration

from std_msgs.msg import String, UInt8, Int32, Bool
from geometry_msgs.msg import Vector3, Twist
from sensor_msgs.msg import CompressedImage
from std_srvs.srv import Trigger, SetBool

import cv2
import threading
import time
import numpy as np
import queue

from djitellopy import Tello


def sensor_qos(depth=10, best_effort=True):
    return QoSProfile(
        reliability=(ReliabilityPolicy.BEST_EFFORT if best_effort else ReliabilityPolicy.RELIABLE),
        history=HistoryPolicy.KEEP_LAST,
        depth=depth
    )


class TelloDriver(Node):
    def __init__(self):
        super().__init__('tello_driver')

        # Ajustes generales de temporizacion
        self.declare_parameter('video_enabled', False)
        self.declare_parameter('telemetry_rate_hz', 1)
        self.declare_parameter('timeout_sec', 3.0)
        self.declare_parameter('probe_period_s', 1.0)
        self.declare_parameter('status_rate_hz', 1.0)
        self.declare_parameter('verbose_state', True)

        # Parametros de video comprimido publicados por este nodo
        self.declare_parameter('video_fps_limit', 30.0)
        self.declare_parameter('video_resize_hw', [0, 0])
        self.declare_parameter('video_frame_id', 'tello_camera')
        self.declare_parameter('video_publish_compressed', True)
        self.declare_parameter('jpeg_quality', 90)

        self.telemetry_dt = 1.0 / max(1, int(self.get_parameter('telemetry_rate_hz').value))
        self.timeout = Duration(seconds=float(self.get_parameter('timeout_sec').value))
        self.probe_period = float(self.get_parameter('probe_period_s').value)
        self.status_dt = 1.0 / max(0.1, float(self.get_parameter('status_rate_hz').value))

        self.video_fps_limit = float(self.get_parameter('video_fps_limit').value)
        vr = self.get_parameter('video_resize_hw').get_parameter_value().integer_array_value
        self.resize_h = int(vr[0]) if len(vr) > 0 else 0
        self.resize_w = int(vr[1]) if len(vr) > 1 else 0
        self.video_frame_id = self.get_parameter('video_frame_id').get_parameter_value().string_value
        self.pub_comp = bool(self.get_parameter('video_publish_compressed').value)
        self.jpeg_quality = int(self.get_parameter('jpeg_quality').value)

        # Publicadores que alimentan al resto del sistema
        self.pub_image_comp = self.create_publisher(CompressedImage, '/tello/image_raw/compressed', sensor_qos(depth=1, best_effort=True))
        self.pub_state   = self.create_publisher(String, '/tello/stream/state', sensor_qos(depth=10, best_effort=True))
        self.pub_battery = self.create_publisher(UInt8,  '/tello/battery',      sensor_qos(depth=10, best_effort=True))
        self.pub_height  = self.create_publisher(Int32,  '/tello/height',       sensor_qos(depth=10, best_effort=True))
        self.pub_vel     = self.create_publisher(Vector3,'/tello/velocity',     sensor_qos(depth=10, best_effort=True))
        self.pub_att     = self.create_publisher(Vector3,'/tello/attitude',     sensor_qos(depth=10, best_effort=True))
        self.pub_conn    = self.create_publisher(Bool,   '/tello/connected',    sensor_qos(depth=1,  best_effort=False))

        # Suscripcion de control de velocidad desde el planner
        self.sub_cmd_vel = self.create_subscription(Twist, '/tello/cmd_vel', self.on_cmd_vel, 10)

        # Servicios que exponen acciones directas sobre el dron
        self.srv_takeoff    = self.create_service(Trigger, '/tello/takeoff',    self.srv_takeoff_cb)
        self.srv_land       = self.create_service(Trigger, '/tello/land',       self.srv_land_cb)
        self.srv_stream_on  = self.create_service(Trigger, '/tello/stream_on',  self.srv_stream_on_cb)
        self.srv_stream_off = self.create_service(Trigger, '/tello/stream_off', self.srv_stream_off_cb)
        self.srv_emergency  = self.create_service(Trigger, '/tello/emergency',  self.srv_emergency_cb)
        self.srv_sdk_cmd    = self.create_service(Trigger, '/tello/sdk_command', self.srv_sdk_cmd_cb)
        self.srv_stream_set = self.create_service(SetBool, '/tello/stream/set', self.srv_stream_set_cb)

        # Estado interno para video comprimido y telemetria
        self.tello = Tello()
        self._streaming = False
        self._video_thread = None
        self._enc_thread = None
        self._frame_q = queue.Queue(maxsize=1)

        self._stop_evt = threading.Event()
        self._last_life: Time = Time()
        self._current_status: bool = False

        self._last_pub_t = 0.0
        self._fps_accum = 0
        self._fps_last_report = 0.0
        self._min_pub_period = 1.0 / max(1e-3, self.video_fps_limit)

        try:
            cv2.setNumThreads(1)
        except Exception:
            pass

        # Conexion inicial al dron y publicacion de estado
        try:
            self.tello.connect()
            self._mark_life()
            self.get_logger().info("Conectado a Tello (boot).")
        except Exception as e:
            self.get_logger().warn(f"Inicio sin conexion: {e}")

        # Temporizadores ROS y el hilo de vigilancia de enlace
        self.telemetry_timer = self.create_timer(self.telemetry_dt, self.telemetry_tick)
        self.status_timer = self.create_timer(self.status_dt, self.status_tick)

        self._probe_thread = threading.Thread(target=self._probe_loop, name='tello_probe', daemon=True)
        self._probe_thread.start()

        self.get_logger().info("driver_tello listo (stream OFF por defecto).")

    # Utilidades para vigilar la vida del enlace
    def _mark_life(self):
        self._last_life = self.get_clock().now()

    def _alive_recent(self) -> bool:
        if self._last_life.nanoseconds == 0:
            return False
        return (self.get_clock().now() - self._last_life) < self.timeout

    # Hilo que reenvia el comando command para sostener el enlace
    def _probe_loop(self):
        while not self._stop_evt.is_set():
            start = time.perf_counter()
            try:
                resp = self.tello.send_control_command('command')
                ok = False
                if isinstance(resp, bool):
                    ok = resp
                elif isinstance(resp, (bytes, bytearray)):
                    try:
                        ok = (bytes(resp).decode(errors='ignore').strip().lower() == 'ok')
                    except Exception:
                        ok = False
                elif isinstance(resp, str):
                    ok = (resp.strip().lower() == 'ok')

                if ok:
                    if self.get_parameter('verbose_state').value:
                        self.pub_state.publish(String(data="probe:ok"))
                    self._mark_life()
                else:
                    self.get_logger().warn(f"Probe 'command' respuesta no valida: {resp}")
            except Exception as e:
                self.get_logger().warn(f"Probe 'command' fallo: {e}")

            elapsed = time.perf_counter() - start
            sleep_s = max(0.0, self.probe_period - elapsed)
            end_by = time.perf_counter() + sleep_s
            while not self._stop_evt.is_set() and time.perf_counter() < end_by:
                time.sleep(0.05)

    # Publicacion de /tello/connected con el ultimo estado conocido
    def status_tick(self):
        connected = self._alive_recent()
        if connected != self._current_status:
            self.get_logger().info(f'/tello/connected = {connected}')
            self._current_status = connected
        self.pub_conn.publish(Bool(data=connected))

    # Muestreo periodico de telemetria SDK
    def telemetry_tick(self):
        if not self._alive_recent():
            return
        try:
            state = getattr(self.tello, "get_current_state", lambda: {})() or {}
            if self.get_parameter('verbose_state').value:
                kv = [f"{k}:{state.get(k)}" for k in (
                    'bat', 'h', 'vgx', 'vgy', 'vgz', 'pitch', 'roll', 'yaw'
                ) if k in state]
                if kv:
                    self.pub_state.publish(String(data=",".join(kv)))

            bat = state.get('bat')
            if bat is not None:
                self.pub_battery.publish(UInt8(data=int(bat)))

            h = state.get('h')
            if h is not None:
                self.pub_height.publish(Int32(data=int(h)))

            vgx, vgy, vgz = state.get('vgx'), state.get('vgy'), state.get('vgz')
            if None not in (vgx, vgy, vgz):
                self.pub_vel.publish(Vector3(x=float(vgx), y=float(vgy), z=float(vgz)))

            pitch, roll, yaw = state.get('pitch'), state.get('roll'), state.get('yaw')
            if None not in (pitch, roll, yaw):
                self.pub_att.publish(Vector3(x=float(pitch), y=float(roll), z=float(yaw)))
        except Exception as e:
            self.get_logger().warn(f"telemetry_tick error: {e}")

    # Gestion del stream de video comprimido
    def _start_video_thread(self):
        if self._video_thread and self._video_thread.is_alive():
            return
        try:
            self.tello.streamon()
            self._streaming = True
            self._last_pub_t = 0.0
            self._fps_accum = 0
            self._fps_last_report = time.perf_counter()
            self._min_pub_period = 1.0 / max(1e-3, float(self.get_parameter('video_fps_limit').value))

            self._video_thread = threading.Thread(target=self._video_loop, name='tello_video', daemon=True)
            self._video_thread.start()

            self._enc_thread = threading.Thread(target=self._encoder_loop, name='tello_encoder', daemon=True)
            self._enc_thread.start()

            self.get_logger().info("Stream de video ON (encoder opencv).")
        except Exception as e:
            self.get_logger().error(f"No se pudo iniciar el stream: {e}")

    def _stop_video(self):
        if not self._streaming:
            return
        try:
            self.tello.streamoff()
        except Exception:
            pass
        self._streaming = False

        try:
            while not self._frame_q.empty():
                self._frame_q.get_nowait()
        except Exception:
            pass

        try:
            if self._video_thread and self._video_thread.is_alive():
                self._video_thread.join(timeout=1.0)
        except Exception:
            pass
        try:
            if self._enc_thread and self._enc_thread.is_alive():
                self._enc_thread.join(timeout=1.0)
        except Exception:
            pass

        self.get_logger().info("Stream de video OFF.")

    def srv_stream_set_cb(self, request, response):
        try:
            if bool(request.data):
                self._start_video_thread()
                response.success = True
                response.message = "streamon"
            else:
                self._stop_video()
                response.success = True
                response.message = "streamoff"
        except Exception as e:
            response.success = False
            response.message = f"stream_set: {e}"
        return response

    # Hilo lector que ajusta tamano y ritmo de cuadros
    def _video_loop(self):
        try:
            frame_reader = self.tello.get_frame_read()
            while rclpy.ok() and not self._stop_evt.is_set() and self._streaming:
                if not self._alive_recent():
                    time.sleep(0.02)
                    continue

                frame = frame_reader.frame
                if frame is None:
                    time.sleep(0.005)
                    continue

                now = time.perf_counter()
                if self._last_pub_t and (now - self._last_pub_t) < self._min_pub_period:
                    time.sleep(0.001)
                    continue

                out = frame
                if self.resize_h > 0 and self.resize_w > 0:
                    try:
                        out = cv2.resize(out, (self.resize_w, self.resize_h), interpolation=cv2.INTER_AREA)
                    except Exception as e:
                        self.get_logger().warn(f"resize fallo: {e}")
                        out = frame

                stamp = self.get_clock().now().to_msg()
                frame_id = self.video_frame_id

                try:
                    if self._frame_q.full():
                        try:
                            self._frame_q.get_nowait()
                        except Exception:
                            pass
                    self._frame_q.put_nowait((out, stamp, frame_id))
                    self._last_pub_t = now
                    self._fps_accum += 1
                except Exception:
                    pass

                if (now - self._fps_last_report) >= 1.0 and self.get_parameter('verbose_state').value:
                    eff = self._fps_accum / (now - self._fps_last_report)
                    self._fps_last_report = now
                    self._fps_accum = 0
                    self.pub_state.publish(String(data=f"video_fps:{eff:.1f}"))
        except Exception as e:
            self.get_logger().warn(f"Video loop detenido: {e}")

    # Hilo encoder que publica JPEG en el topico compartido
    def _encoder_loop(self):
        while rclpy.ok() and not self._stop_evt.is_set() and self._streaming:
            try:
                out, stamp, frame_id = self._frame_q.get(timeout=0.1)
            except Exception:
                continue

            if self.pub_comp:
                try:
                    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), int(self.jpeg_quality)]
                    ok, buf = cv2.imencode('.jpg', out, encode_param)
                    if ok:
                        cm = CompressedImage()
                        cm.header.stamp = stamp
                        cm.header.frame_id = frame_id
                        cm.format = 'jpeg'
                        cm.data = np.asarray(buf).tobytes()
                        self.pub_image_comp.publish(cm)
                except Exception as e:
                    self.get_logger().warn(f"jpeg encode fallo: {e}")

    # Conversion directa de Twist a comandos rc
    def on_cmd_vel(self, msg: Twist):
        if not self._alive_recent():
            return

        def clamp100(x): return max(-100, min(100, int(x)))
        a = clamp100(msg.linear.y * 100.0)
        b = clamp100(msg.linear.x * 100.0)
        c = clamp100(msg.linear.z * 100.0)
        d = clamp100(msg.angular.z * 100.0)
        try:
            self.tello.send_rc_control(a, b, c, d)
        except Exception as e:
            self.get_logger().warn(f"send_rc_control fallo: {e}")

    # Respuesta Trigger reutilizable
    def _ok(self, ok=True, message="ok"):
        res = Trigger.Response()
        res.success = bool(ok)
        res.message = str(message)
        return res

    def srv_takeoff_cb(self, req, resp):
        try:
            self.tello.takeoff()
            self._mark_life()
            return self._ok(True)
        except Exception as e:
            return self._ok(False, f"takeoff: {e}")

    def srv_land_cb(self, req, resp):
        try:
            self.tello.land()
            self._mark_life()
            return self._ok(True)
        except Exception as e:
            return self._ok(False, f"land: {e}")

    def srv_stream_on_cb(self, req, resp):
        try:
            self._start_video_thread()
            return self._ok(True)
        except Exception as e:
            return self._ok(False, f"stream_on: {e}")

    def srv_stream_off_cb(self, req, resp):
        try:
            self._stop_video()
            return self._ok(True)
        except Exception as e:
            return self._ok(False, f"stream_off: {e}")

    def srv_emergency_cb(self, req, resp):
        try:
            self.tello.emergency()
            self._mark_life()
            return self._ok(True)
        except Exception as e:
            return self._ok(False, f"emergency: {e}")

    def srv_sdk_cmd_cb(self, req, resp):
        try:
            self.tello.send_control_command("command")
            self._mark_life()
            return self._ok(True)
        except Exception as e:
            return self._ok(False, f"sdk_command: {e}")

    # Limpieza cuando el nodo se destruye
    def destroy_node(self):
        self._stop_evt.set()
        try:
            self._stop_video()
        except Exception:
            pass
        try:
            if self._probe_thread.is_alive():
                self._probe_thread.join(timeout=1.0)
        except Exception:
            pass
        try:
            self.tello.end()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = TelloDriver()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

